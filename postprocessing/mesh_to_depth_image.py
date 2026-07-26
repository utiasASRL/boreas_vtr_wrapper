#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
from pathlib import Path
from time import perf_counter

import numpy as np
import torch
from pyboreas import BoreasDataset
from pyboreas.utils.odometry import interpolate_poses
from pyboreas.utils.utils import get_inverse_tf
from pylgmath import Transformation
from scipy.interpolate import interp1d
from scipy.ndimage import gaussian_filter1d, shift
from vtr_pose_graph.graph_iterators import TemporalIterator
import vtr_pose_graph.graph_utils as g_utils
from vtr_utils.bag_file_parsing import Rosbag2GraphFactory
from vtr_utils.plot_utils import convert_points_to_frame
from perturbation_cost_tests.perturbation_utils import make_delta_T

from localization_dfo.optix_backend import OptixDepthBackend


def get_submap_vertices(graph_dir):
    factory = Rosbag2GraphFactory(graph_dir)
    graph = factory.buildGraph()
    print(
        f"Graph {graph} has {graph.number_of_vertices} vertices and "
        f"{graph.number_of_edges} edges"
    )

    g_utils.set_world_frame(graph, graph.root)
    v_start = graph.get_vertex((0, 0))
    submap_vertices = []
    curr_submap_vid = None
    for vertex, _ in TemporalIterator(v_start):
        map_ptr = vertex.get_data("pointmap_ptr")
        teach_v = graph.get_vertex(map_ptr.map_vid)
        if curr_submap_vid != teach_v.id:
            submap_vertices.append(teach_v)
            curr_submap_vid = teach_v.id

    return graph, submap_vertices


def build_patch_config(
    fov_deg,
    res_deg,
    min_range=0.0,
    max_uv_edge_length=None,
    max_depth_jump=2.0,
):
    hfov = np.deg2rad(fov_deg)
    vfov = np.deg2rad(fov_deg)
    dtheta = np.deg2rad(res_deg)
    dphi = np.deg2rad(res_deg)
    width = int(np.round(hfov / dtheta)) + 1
    height = int(np.round(vfov / dphi)) + 1
    if width % 2 != 1 or height % 2 != 1:
        raise ValueError("Patch dimensions must be odd so there is an exact center pixel.")

    uu, vv = np.meshgrid(
        np.arange(width, dtype=np.float32),
        np.arange(height, dtype=np.float32),
    )
    return {
        "hfov": hfov,
        "vfov": vfov,
        "theta_min": -0.5 * hfov,
        "theta_max": 0.5 * hfov,
        "phi_min": -0.5 * vfov,
        "phi_max": 0.5 * vfov,
        "dtheta": dtheta,
        "dphi": dphi,
        "width": width,
        "height": height,
        "query_uv": np.stack([uu.ravel(), vv.ravel()], axis=1),
        "min_range": min_range,
        "max_uv_edge_length": max_uv_edge_length,
        "max_depth_jump": max_depth_jump,
        "fill_value": 0.0,
    }


def build_lidar_to_robot_transform(seq):
    """Return the static transform that maps robot-frame points into LiDAR."""
    T_wheel_robot = Transformation(
        T_ba=np.array(
            [
                [0.0, -1.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
    )
    T_applanix_wheel = Transformation(T_ba=seq.calib.T_applanix_wheel)
    T_applanix_lidar = Transformation(T_ba=seq.calib.T_applanix_lidar)
    return T_applanix_lidar.inverse() * T_applanix_wheel * T_wheel_robot


def correct_offsets(radar_frame, radar_frame_idx, seq):
    prev_radar_frame = seq.radar_frames[radar_frame_idx - 1]
    next_radar_frame = seq.radar_frames[radar_frame_idx + 1]
    body_rates = [
        prev_radar_frame.body_rate,
        radar_frame.body_rate,
        next_radar_frame.body_rate,
    ]
    times_us = [
        prev_radar_frame.timestamp_micro,
        radar_frame.timestamp_micro,
        next_radar_frame.timestamp_micro,
    ]

    azimuth_timestamps = radar_frame.timestamps.flatten()
    f = interp1d(times_us, body_rates, axis=0, kind="quadratic")
    azimuth_body_rates = f(azimuth_timestamps)

    shifted_polar = shift(
        radar_frame.polar,
        shift=(0, seq.calib.radar_offset / radar_frame.resolution),
        order=3,
        mode="nearest",
    )

    vx = -azimuth_body_rates[:, 0]
    vy = -azimuth_body_rates[:, 1]
    u = vx * np.cos(radar_frame.azimuths) + vy * np.sin(radar_frame.azimuths)
    beta = seq.calib.radar_doppler_beta
    delta_r_d = beta * u
    chirp_sign = np.where(radar_frame.chirp_type == 0, -1, radar_frame.chirp_type)
    doppler_shift = chirp_sign * delta_r_d / radar_frame.resolution

    for idx in range(len(shifted_polar)):
        shifted_polar[idx] = shift(
            shifted_polar[idx],
            shift=-doppler_shift[idx],
            order=3,
            mode="nearest",
        )

    return shifted_polar


def cen_filter_2d(polar_image, sigma_gauss=15.0, z_q=2.5, noise_scale=0.5):
    mean_val = np.mean(polar_image, axis=1, keepdims=True)
    q = polar_image - mean_val

    p = gaussian_filter1d(q, sigma=sigma_gauss, axis=1, mode="reflect")

    neg_mask = q < 0
    count = np.sum(neg_mask, axis=1, keepdims=True)
    q_neg_sq = (q * neg_mask) ** 2
    sum_q_neg_sq = np.sum(q_neg_sq, axis=1, keepdims=True)
    sigma_q_sq = np.divide(
        2.0 * sum_q_neg_sq,
        count,
        out=np.zeros_like(count, dtype=float),
        where=count != 0,
    )
    sigma_q = noise_scale * np.sqrt(sigma_q_sq)
    sigma_q[count == 0] = 0.034

    threshold = z_q * sigma_q
    eps = 1e-8
    pow_p = (p / (sigma_q + eps)) ** 2
    pow_qp = ((q - p) / (sigma_q + eps)) ** 2
    npp = np.exp(-0.5 * pow_p)
    nqp = np.exp(-0.5 * pow_qp)

    y = q * (1.0 - nqp) + p * (nqp - npp)
    y[y <= threshold] = 0.0
    return y


def save_patches(seq_dir, patches, fov_deg, radar_frame, method):
    output_dir = Path(seq_dir) / "input" / f"patch_arrays_{fov_deg}_deg_{method}"
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / f"{radar_frame}.npy"
    np.save(output_path, np.rint(patches * 100).astype(np.uint16))
    return output_path


def save_labels(seq_dir, folder_name, polar, radar_frame):
    labels_dir = Path(seq_dir) / folder_name
    labels_dir.mkdir(parents=True, exist_ok=True)
    output_path = labels_dir / f"{radar_frame}.npy"
    np.save(output_path, polar)
    return output_path


def load_submap_mesh_to_enu(
    mesh_root,
    sequence_id,
    submap,
    T_lidar_robot,
    lidar_pose,
    device,
):
    """Load one saved robot-frame mesh and upload its ENU representation."""
    submap_stamp_us = submap.stamp // 1000
    mesh_dir = Path(mesh_root) / sequence_id / str(submap_stamp_us)
    vertices_path = mesh_dir / "vertices.npy"
    triangles_path = mesh_dir / "triangles.npy"
    metadata_path = mesh_dir / "metadata.npz"

    missing = [
        path
        for path in (vertices_path, triangles_path, metadata_path)
        if not path.is_file()
    ]
    if missing:
        missing_text = ", ".join(str(path) for path in missing)
        raise FileNotFoundError(
            f"Saved mesh for submap {submap_stamp_us} is incomplete: {missing_text}"
        )

    t0 = perf_counter()
    vertices_robot = np.load(vertices_path)
    triangles = np.load(triangles_path)
    with np.load(metadata_path) as metadata:
        coordinate_frame = str(metadata["coordinate_frame"].item())
        metadata_stamp_us = int(metadata["submap_stamp_us"].item())

    if coordinate_frame != "submap_robot":
        raise ValueError(
            f"Expected mesh coordinate frame 'submap_robot', got {coordinate_frame!r}."
        )
    if metadata_stamp_us != submap_stamp_us:
        raise ValueError(
            f"Mesh metadata stamp {metadata_stamp_us} does not match "
            f"submap stamp {submap_stamp_us}."
        )
    if vertices_robot.ndim != 2 or vertices_robot.shape[1] != 3:
        raise ValueError(f"Mesh vertices must have shape (N, 3), got {vertices_robot.shape}.")
    if triangles.ndim != 2 or triangles.shape[1] != 3:
        raise ValueError(f"Mesh triangles must have shape (F, 3), got {triangles.shape}.")
    if len(triangles) == 0 or np.min(triangles) < 0 or np.max(triangles) >= len(vertices_robot):
        raise ValueError("Mesh triangles do not validly index the saved vertices.")
    disk_load_time = perf_counter() - t0

    t0 = perf_counter()
    vertices_lidar = convert_points_to_frame(vertices_robot.T, T_lidar_robot)
    T_enu_lidar = Transformation(T_ba=lidar_pose)
    vertices_enu = convert_points_to_frame(vertices_lidar, T_enu_lidar)
    vertices_gpu = torch.as_tensor(
        vertices_enu.T,
        device=device,
        dtype=torch.float32,
    ).contiguous()
    triangles_gpu = torch.as_tensor(
        triangles,
        device=device,
        dtype=torch.int32,
    ).contiguous()
    if device.type == "cuda":
        torch.cuda.synchronize(device)
    transform_upload_time = perf_counter() - t0

    print(
        f"Loaded submap mesh {submap_stamp_us}: "
        f"{len(vertices_robot)} vertices, {len(triangles)} faces"
    )
    return vertices_gpu, triangles_gpu, {
        "mesh_disk_load": disk_load_time,
        "mesh_robot_to_enu_upload": transform_upload_time,
    }


def extract_depth_patches_optix(tracer, current_transform):
    t0 = perf_counter()
    depth_gpu = tracer.trace(current_transform)
    depth_patches = depth_gpu.cpu().numpy()
    return depth_patches, {
        "optix_wall": perf_counter() - t0,
        "hit_pixels": int(np.count_nonzero(depth_patches > 0.0)),
    }


def extract_augmented_depth_patches_optix(
    tracer,
    rng,
    num_augments,
    aug_trans_max_m,
    aug_rot_max_deg,
):
    all_patches = []
    stats = {"optix_wall": 0.0, "hit_pixels": 0}

    for aug_idx in range(num_augments + 1):
        if aug_idx == 0:
            current_transform = np.eye(4)
        else:
            translation = rng.uniform(-aug_trans_max_m, aug_trans_max_m, size=3)
            rpy_deg = rng.uniform(-aug_rot_max_deg, aug_rot_max_deg, size=3)
            current_transform = make_delta_T(translation=translation, rpy_deg=rpy_deg)

        patches, patch_stats = extract_depth_patches_optix(tracer, current_transform)
        all_patches.append(patches)
        for key, value in patch_stats.items():
            stats[key] += value

    return np.stack(all_patches, axis=1).astype(np.float32), dict(stats)


def run_sequence(
    seq,
    lidar_results_dir,
    output_root,
    mesh_root,
    radar_start_frame,
    radar_end_frame,
    patch_config,
    device,
    method_suffix,
    data_augment=False,
    num_augments=4,
    aug_trans_max_m=0.005,
    aug_rot_max_deg=0.02,
    aug_seed=0,
):
    print(f"SequenceID: {seq.ID}")
    print(f"Number of Radar Frames: {len(seq.radar_frames)}")

    end_frame = len(seq.radar_frames) - 2 if radar_end_frame is None else min(
        radar_end_frame,
        len(seq.radar_frames) - 2,
    )
    radar_frame_idx = max(radar_start_frame, 1)

    graph_dir = os.path.join(lidar_results_dir, seq.ID, seq.ID, "graph")
    _, submap_vertices = get_submap_vertices(graph_dir=graph_dir)
    T_lidar_robot = build_lidar_to_robot_transform(seq)

    lidar_frame_idx = 0
    submap_vertices_idx = 0
    loaded_submap_stamp_us = None
    mesh_vertices_gpu = None
    mesh_triangles_gpu = None
    aug_rng = np.random.default_rng(aug_seed)
    tracer = OptixDepthBackend(patch_config, device)

    while (
        submap_vertices_idx < len(submap_vertices) - 1
        and lidar_frame_idx < len(seq.lidar_frames)
        and radar_frame_idx <= end_frame
    ):
        curr_submap = submap_vertices[submap_vertices_idx]
        next_submap = submap_vertices[submap_vertices_idx + 1]
        radar_frame_meta = seq.radar_frames[radar_frame_idx]
        lidar_frame = seq.lidar_frames[lidar_frame_idx]

        if int(radar_frame_meta.frame) < curr_submap.stamp // 1000:
            radar_frame_idx += 1
            continue
        if next_submap.stamp // 1000 < int(radar_frame_meta.frame):
            submap_vertices_idx += 1
            continue
        if int(lidar_frame.frame) != curr_submap.stamp // 1000:
            lidar_frame_idx += 1
            continue

        submap_stamp_us = curr_submap.stamp // 1000
        mesh_load_stats = {"mesh_disk_load": 0.0, "mesh_robot_to_enu_upload": 0.0}
        if loaded_submap_stamp_us != submap_stamp_us:
            del mesh_vertices_gpu, mesh_triangles_gpu
            mesh_vertices_gpu, mesh_triangles_gpu, mesh_load_stats = (
                load_submap_mesh_to_enu(
                    mesh_root=mesh_root,
                    sequence_id=seq.ID,
                    submap=curr_submap,
                    T_lidar_robot=T_lidar_robot,
                    lidar_pose=lidar_frame.pose,
                    device=device,
                )
            )
            tracer.set_mesh(mesh_vertices_gpu, mesh_triangles_gpu)
            loaded_submap_stamp_us = submap_stamp_us

        frame_start = perf_counter()
        radar_frame = seq.get_radar(radar_frame_idx)

        t0 = perf_counter()
        poses = [
            get_inverse_tf(rad_frame.pose)
            for rad_frame in seq.radar_frames[radar_frame_idx - 1:radar_frame_idx + 2]
        ]
        times = [
            rad_frame.timestamp_micro
            for rad_frame in seq.radar_frames[radar_frame_idx - 1:radar_frame_idx + 2]
        ]
        query_times = radar_frame.timestamps.flatten().tolist()
        azimuth_poses = np.asarray(interpolate_poses(poses, times, query_times))
        radar_azimuths = np.asarray(radar_frame.azimuths).reshape(-1)
        pose_time = perf_counter() - t0
        tracer.set_scan(azimuth_poses, radar_azimuths)

        t0 = perf_counter()
        if data_augment:
            mesh_patches, patch_stats = extract_augmented_depth_patches_optix(
                tracer=tracer,
                rng=aug_rng,
                num_augments=num_augments,
                aug_trans_max_m=aug_trans_max_m,
                aug_rot_max_deg=aug_rot_max_deg,
            )
            assert mesh_patches.ndim == 4
            assert mesh_patches.shape[1] == 1 + num_augments
        else:
            mesh_patches, patch_stats = extract_depth_patches_optix(tracer, np.eye(4))
            assert mesh_patches.ndim == 3
        patch_time = perf_counter() - t0

        t0 = perf_counter()
        shifted_polar = correct_offsets(radar_frame, radar_frame_idx, seq)
        filtered_polar = cen_filter_2d(
            shifted_polar,
            sigma_gauss=15.0,
            z_q=2.5,
            noise_scale=0.5,
        )
        polar_time = perf_counter() - t0

        t0 = perf_counter()
        suffix = f"_{method_suffix}" if method_suffix else ""
        method = f"optix{suffix}_aug" if data_augment else f"optix{suffix}"
        sequence_output_dir = Path(output_root) / seq.ID
        mesh_path = save_patches(
            seq_dir=sequence_output_dir,
            patches=mesh_patches,
            fov_deg=patch_config["fov_deg"],
            radar_frame=radar_frame.frame,
            method=method,
        )
        labels_path = save_labels(
            seq_dir=sequence_output_dir,
            folder_name="filtered_labels",
            polar=filtered_polar,
            radar_frame=radar_frame.frame,
        )
        save_time = perf_counter() - t0

        num_patches = max(
            mesh_patches.shape[0] * (mesh_patches.shape[1] if mesh_patches.ndim == 4 else 1),
            1,
        )
        print(
            f"Frame {radar_frame.frame} | total {perf_counter() - frame_start:.3f}s | "
            f"poses {pose_time:.3f}s | patches {patch_time:.3f}s | "
            f"radar postprocessing {polar_time:.3f}s | save {save_time:.3f}s"
        )
        if mesh_load_stats["mesh_disk_load"] > 0.0:
            print(
                f"  mesh load {mesh_load_stats['mesh_disk_load']:.3f}s | "
                f"robot->ENU/GPU {mesh_load_stats['mesh_robot_to_enu_upload']:.3f}s"
            )
        print(
            f"  OptiX/copy wall {patch_stats['optix_wall']:.3f}s | "
            f"avg hit pixels {patch_stats['hit_pixels'] / num_patches:.1f}"
        )
        print(f"  saved: {mesh_path}")
        print(f"  saved: {labels_path}")
        if data_augment:
            print(
                f"  saved augmented patches: shape {list(mesh_patches.shape)} | "
                f"augmentation ranges: translation +/-{aug_trans_max_m} m, "
                f"rotation +/-{aug_rot_max_deg} deg"
            )

        radar_frame.unload_data()
        radar_frame_idx += 1


def main():
    parser = argparse.ArgumentParser(
        description="Generate radar-azimuth depth patches from cached submap meshes."
    )
    parser.add_argument("--radar-start-frame", type=int, default=1)
    parser.add_argument("--radar-end-frame", type=int, default=None)
    parser.add_argument("--sequence-id", default=None)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--fov-deg", type=float, default=6.0)
    parser.add_argument("--res-deg", type=float, default=0.1)
    parser.add_argument("--data-augment", action="store_true")
    parser.add_argument("--num-augments", type=int, default=4)
    parser.add_argument("--aug-trans-max-m", type=float, default=0.01)
    parser.add_argument("--aug-rot-max-deg", type=float, default=0.05)
    parser.add_argument("--aug-seed", type=int, default=0)
    parser.add_argument(
        "--mesh-root",
        type=Path,
        default=Path(__file__).resolve().parent / "submap_meshes",
    )
    parser.add_argument(
        "--method-suffix",
        default="",
        help="Suffix for the output patch folder.",
    )
    args = parser.parse_args()
    if args.num_augments < 0:
        raise ValueError("--num-augments must be non-negative.")
    if args.aug_trans_max_m < 0.0:
        raise ValueError("--aug-trans-max-m must be non-negative.")
    if args.aug_rot_max_deg < 0.0:
        raise ValueError("--aug-rot-max-deg must be non-negative.")

    device = torch.device(args.device)
    if device.type != "cuda" or not torch.cuda.is_available():
        raise RuntimeError("OptiX depth generation requires a CUDA device.")

    boreas_vtr_wrapper_dir = os.getenv("VTRROOT")
    boreas_data = os.getenv("VTRRDATA")
    boreas_output = os.getenv("BOREAS_OUTPUT_ROOT")
    vtr_results = os.getenv("VTRRESULT")
    if boreas_vtr_wrapper_dir is None:
        raise RuntimeError("VTRROOT must be set.")
    if boreas_data is None:
        raise RuntimeError("VTRRDATA must be set.")
    if boreas_output is None:
        raise RuntimeError("BOREAS_OUTPUT_ROOT must be set.")

    lidar_results_dir = os.path.join(
        vtr_results or os.path.join(boreas_vtr_wrapper_dir, "results"),
        "lidar",
    )
    dataset = BoreasDataset(
        boreas_data,
        [[args.sequence_id]] if args.sequence_id is not None else None,
    )
    patch_config = build_patch_config(
        fov_deg=args.fov_deg,
        res_deg=args.res_deg,
        min_range=0.0,
        max_uv_edge_length=None,
        max_depth_jump=None,
    )
    patch_config["fov_deg"] = args.fov_deg

    matched_sequence = False
    for seq in dataset.sequences:
        if args.sequence_id is not None and seq.ID != args.sequence_id:
            continue
        matched_sequence = True
        run_sequence(
            seq=seq,
            lidar_results_dir=lidar_results_dir,
            output_root=boreas_output,
            mesh_root=args.mesh_root,
            radar_start_frame=args.radar_start_frame,
            radar_end_frame=args.radar_end_frame,
            patch_config=patch_config,
            device=device,
            method_suffix=args.method_suffix,
            data_augment=args.data_augment,
            num_augments=args.num_augments,
            aug_trans_max_m=args.aug_trans_max_m,
            aug_rot_max_deg=args.aug_rot_max_deg,
            aug_seed=args.aug_seed,
        )
        if args.sequence_id is None:
            break

    if not matched_sequence:
        raise ValueError(f"Sequence not found: {args.sequence_id}")


if __name__ == "__main__":
    main()
