#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter

import numpy as np
import torch
from pyboreas import BoreasDataset
from pyboreas.utils.odometry import interpolate_poses
from pyboreas.utils.utils import get_inverse_tf
from pylgmath import Transformation
from vtr_pose_graph.graph_iterators import TemporalIterator
import vtr_pose_graph.graph_utils as g_utils
from vtr_utils.bag_file_parsing import Rosbag2GraphFactory
from vtr_utils.plot_utils import convert_points_to_frame, extract_points_from_vertex


@dataclass
class GeometryParams:
    theta_min: float
    theta_max: float
    phi_min: float
    phi_max: float
    dtheta: float
    dphi: float
    width: int
    height: int
    max_uv_edge_length: float | None = None
    max_depth_jump: float | None = None
    fill_value: float = 0.0


def get_submap_vertices(graph_dir):
    factory = Rosbag2GraphFactory(graph_dir)
    test_graph = factory.buildGraph()
    print(
        f"Graph {test_graph} {test_graph.number_of_vertices} vertices "
        f"{test_graph.number_of_edges} edges"
    )

    g_utils.set_world_frame(test_graph, test_graph.root)
    v_start = test_graph.get_vertex((0, 0))

    submap_vertices = []
    curr_submap_vid = None
    for vertex, _ in TemporalIterator(v_start):
        map_ptr = vertex.get_data("pointmap_ptr")
        teach_v = test_graph.get_vertex(map_ptr.map_vid)

        if curr_submap_vid != teach_v.id:
            submap_vertices.append(teach_v)
            curr_submap_vid = teach_v.id

    return test_graph, submap_vertices


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
        raise ValueError("Patch dimensions must be odd for an exact center pixel.")

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
        "min_range": min_range,
        "max_uv_edge_length": max_uv_edge_length,
        "max_depth_jump": max_depth_jump,
        "fill_value": 0.0,
    }


def build_lidar_to_robot_transform(seq):
    T_wheel_robot_np = np.array(
        [
            [0.0, -1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    T_wheel_robot = Transformation(T_ba=T_wheel_robot_np)
    T_applanix_wheel = Transformation(T_ba=seq.calib.T_applanix_wheel)
    T_applanix_lidar = Transformation(T_ba=seq.calib.T_applanix_lidar)
    T_lidar_wheel = T_applanix_lidar.inverse() * T_applanix_wheel
    return T_lidar_wheel * T_wheel_robot


def make_delta_T(translation=None, rpy_deg=None):
    translation = np.zeros(3) if translation is None else np.asarray(translation, dtype=float)
    roll, pitch, yaw = np.deg2rad(
        np.zeros(3) if rpy_deg is None else np.asarray(rpy_deg, dtype=float)
    )
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    delta_T = np.eye(4)
    delta_T[:3, :3] = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
    delta_T[:3, 3] = translation
    return delta_T


def save_patches(seq_dir, patches, fov_deg, radar_frame, method):
    output_dir = Path(seq_dir) / "input" / f"patch_arrays_{fov_deg}_deg_{method}"
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / f"{radar_frame}.npy"
    np.save(output_path, patches)
    return output_path


def geometry_params_from_patch_config(config):
    return GeometryParams(
        theta_min=config["theta_min"],
        theta_max=config["theta_max"],
        phi_min=config["phi_min"],
        phi_max=config["phi_max"],
        dtheta=config["dtheta"],
        dphi=config["dphi"],
        width=config["width"],
        height=config["height"],
        max_uv_edge_length=config["max_uv_edge_length"],
        max_depth_jump=config["max_depth_jump"],
        fill_value=config["fill_value"],
    )


def map_points_xyz_torch(P_v, device):
    if torch.is_tensor(P_v):
        points = P_v
        if points.ndim == 3 and points.shape[-2:] == (4, 1):
            points = points[:, :3, 0]
        if points.ndim == 2 and points.shape[1] == 4:
            points = points[:, :3]
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError(f"Unsupported Torch point shape: {tuple(points.shape)}")
        return points.to(device=device, dtype=torch.float32)

    points_np = np.asarray(P_v)
    if points_np.ndim == 3 and points_np.shape[-2:] == (4, 1):
        points_np = points_np[:, :3, 0]
    if points_np.ndim == 2 and points_np.shape[1] == 4:
        points_np = points_np[:, :3]
    if points_np.ndim != 2 or points_np.shape[1] != 3:
        raise ValueError(f"Unsupported NumPy point shape: {points_np.shape}")
    return torch.as_tensor(points_np, device=device, dtype=torch.float32)


def validate_symmetric_geometry(geom: GeometryParams, atol=1e-12):
    if not np.isclose(geom.theta_min, -geom.theta_max, atol=atol):
        raise ValueError("Frustum projection expects symmetric theta bounds.")
    if not np.isclose(geom.phi_min, -geom.phi_max, atol=atol):
        raise ValueError("Frustum projection expects symmetric phi bounds.")
    if geom.theta_max <= 0.0 or geom.theta_max >= 0.5 * np.pi:
        raise ValueError("Frustum projection expects 0 < theta_max < pi/2.")
    if geom.phi_max <= 0.0 or geom.phi_max >= 0.5 * np.pi:
        raise ValueError("Frustum projection expects 0 < phi_max < pi/2.")


def hard_round_depth_image_torch(samples, batch_ids, batch_size, geom):
    height = int(geom.height)
    width = int(geom.width)
    pixels_per_image = height * width
    depth_img = torch.full(
        (batch_size * pixels_per_image,),
        torch.inf,
        device=samples.device,
        dtype=samples.dtype,
    )
    if samples.numel() == 0:
        return depth_img.reshape(batch_size, height, width).fill_(0.0)

    u = torch.round(samples[:, 1]).to(torch.long)
    v = torch.round(samples[:, 2]).to(torch.long)
    valid = (
        (u >= 0)
        & (u < width)
        & (v >= 0)
        & (v < height)
        & torch.isfinite(samples[:, 0])
    )
    if not torch.any(valid):
        return depth_img.reshape(batch_size, height, width).fill_(0.0)

    linear = batch_ids[valid] * pixels_per_image + v[valid] * width + u[valid]
    depth_img.scatter_reduce_(0, linear, samples[valid, 0], reduce="amin")
    depth_img[torch.isinf(depth_img)] = float(geom.fill_value)
    return depth_img.reshape(batch_size, height, width)


def project_hard_round_depth_batch_gpu(
    P_v,
    odom_batch,
    radar_azimuth_batch,
    geom,
    device,
):
    validate_symmetric_geometry(geom)
    device = torch.device(device)
    points = map_points_xyz_torch(P_v, device)
    T_batch = torch.as_tensor(np.asarray(odom_batch), device=device, dtype=points.dtype)
    azimuths = torch.as_tensor(radar_azimuth_batch, device=device, dtype=points.dtype)

    use_cuda_timing = device.type == "cuda"
    events = []

    def mark():
        if use_cuda_timing:
            event = torch.cuda.Event(enable_timing=True)
            event.record()
            events.append(event)
        else:
            events.append(perf_counter())

    mark()
    R = T_batch[:, :3, :3]
    t = T_batch[:, :3, 3]
    P_r_batch = torch.einsum("bij,nj->bni", R, points) + t[:, None, :]
    mark()

    cos_a = torch.cos(azimuths)[:, None]
    sin_a = torch.sin(azimuths)[:, None]
    x = P_r_batch[:, :, 0]
    y = P_r_batch[:, :, 1]
    z = P_r_batch[:, :, 2]
    x_local = cos_a * x + sin_a * y
    y_local = -sin_a * x + cos_a * y
    rho2 = x_local * x_local + y_local * y_local
    tan_theta = float(np.tan(geom.theta_max))
    tan_phi = float(np.tan(geom.phi_max))
    masks = (
        (x_local > 0.0)
        & (torch.abs(y_local) <= x_local * tan_theta)
        & (z.abs() * z.abs() <= rho2 * (tan_phi * tan_phi))
    )
    mark()

    batch_ids, point_ids = torch.nonzero(masks, as_tuple=True)
    selected_counts = torch.bincount(batch_ids, minlength=P_r_batch.shape[0])
    xl = x_local[batch_ids, point_ids]
    yl = y_local[batch_ids, point_ids]
    zl = z[batch_ids, point_ids]

    rho2_kept = xl * xl + yl * yl
    rho = torch.sqrt(rho2_kept)
    d = torch.sqrt(rho2_kept + zl * zl)
    theta = torch.atan2(yl, xl)
    phi = torch.atan2(zl, rho)

    samples = torch.empty((len(xl), 3), device=device, dtype=points.dtype)
    samples[:, 0] = d
    samples[:, 1] = (theta - geom.theta_min) / geom.dtheta
    samples[:, 2] = (geom.phi_max - phi) / geom.dphi
    patches = hard_round_depth_image_torch(samples, batch_ids, len(odom_batch), geom)
    mark()

    patches_np = patches.detach().cpu().numpy()
    selected_counts_np = selected_counts.detach().cpu().numpy()
    mark()

    if use_cuda_timing:
        torch.cuda.synchronize(device)
        elapsed = [
            events[i].elapsed_time(events[i + 1]) * 1e-3
            for i in range(len(events) - 1)
        ]
    else:
        elapsed = [events[i + 1] - events[i] for i in range(len(events) - 1)]

    batch_size = max(len(odom_batch), 1)
    timing = {
        "point_transform_time_s": elapsed[0] / batch_size,
        "cartesian_frustum_mask_time_s": elapsed[1] / batch_size,
        "hard_round_time_s": elapsed[2] / batch_size,
        "patch_to_cpu_time_s": elapsed[3] / batch_size,
    }
    return patches_np, selected_counts_np, timing


def extract_depth_patches_gpu(
    map_points_gpu,
    azimuth_poses,
    radar_azimuths,
    patch_config,
    geometry_batch_size,
    device,
):
    geom = geometry_params_from_patch_config(patch_config)
    num_azimuths = len(azimuth_poses)
    hard_round_patches = np.empty(
        (num_azimuths, patch_config["height"], patch_config["width"]),
        dtype=np.float32,
    )
    stats = defaultdict(float)

    for start in range(0, num_azimuths, geometry_batch_size):
        stop = min(start + geometry_batch_size, num_azimuths)
        batch_indices = slice(start, stop)

        t0 = perf_counter()
        batch_patches, selected_counts, gpu_timing = project_hard_round_depth_batch_gpu(
            P_v=map_points_gpu,
            odom_batch=azimuth_poses[batch_indices],
            radar_azimuth_batch=radar_azimuths[batch_indices],
            geom=geom,
            device=device,
        )
        stats["gpu_projection_wall"] += perf_counter() - t0
        hard_round_patches[batch_indices] = batch_patches

        batch_count = stop - start
        for key, value in gpu_timing.items():
            stats[key] += value * batch_count
        stats["selected_points"] += float(np.sum(selected_counts))

    return hard_round_patches, dict(stats)


def extract_augmented_depth_patches_gpu(
    map_points_gpu,
    azimuth_poses,
    radar_azimuths,
    patch_config,
    geometry_batch_size,
    device,
    rng,
    num_augments=4,
    aug_trans_max_m=0.005,
    aug_rot_max_deg=0.02,
):
    all_patches = []
    stats = defaultdict(float)

    for aug_idx in range(num_augments + 1):
        if aug_idx == 0:
            poses = azimuth_poses
        else:
            translation = rng.uniform(-aug_trans_max_m, aug_trans_max_m, size=3)
            rpy_deg = rng.uniform(-aug_rot_max_deg, aug_rot_max_deg, size=3)
            delta_T = make_delta_T(translation=translation, rpy_deg=rpy_deg)
            poses = np.asarray([delta_T @ T for T in azimuth_poses])

        patches, patch_stats = extract_depth_patches_gpu(
            map_points_gpu=map_points_gpu,
            azimuth_poses=poses,
            radar_azimuths=radar_azimuths,
            patch_config=patch_config,
            geometry_batch_size=geometry_batch_size,
            device=device,
        )
        all_patches.append(patches)
        for key, value in patch_stats.items():
            stats[key] += value

    return np.stack(all_patches, axis=1).astype(np.float32), dict(stats)


def run_sequence(
    seq,
    lidar_results_dir,
    radar_start_frame,
    radar_end_frame,
    patch_config,
    geometry_batch_size,
    device,
    method_suffix,
    data_augment=False,
    num_augments=4,
    aug_trans_max_m=0.005,
    aug_rot_max_deg=0.02,
    aug_seed=0,
):
    print(f"SequenceID: {seq.ID}")
    print(f"Number Radar Frames: {len(seq.radar_frames)}")

    end_frame = (
        len(seq.radar_frames) - 2
        if radar_end_frame is None
        else min(radar_end_frame, len(seq.radar_frames) - 2)
    )
    radar_frame_idx = max(radar_start_frame, 1)

    graph_dir = os.path.join(lidar_results_dir, seq.ID, seq.ID, "graph")
    _, submap_vertices = get_submap_vertices(graph_dir=graph_dir)
    T_lidar_robot = build_lidar_to_robot_transform(seq)

    lidar_frame_idx = 0
    submap_vertices_idx = 0
    aug_rng = np.random.default_rng(aug_seed)

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

        frame_start = perf_counter()
        radar_frame = seq.get_radar(radar_frame_idx)

        t0 = perf_counter()
        poses = [
            get_inverse_tf(rad_frame.pose)
            for rad_frame in seq.radar_frames[radar_frame_idx - 1 : radar_frame_idx + 2]
        ]
        times = [
            rad_frame.timestamp_micro
            for rad_frame in seq.radar_frames[radar_frame_idx - 1 : radar_frame_idx + 2]
        ]
        query_times = radar_frame.timestamps.flatten().tolist()
        azimuth_poses = np.asarray(interpolate_poses(poses, times, query_times))
        radar_azimuths = np.asarray(radar_frame.azimuths).reshape(-1)
        pose_time = perf_counter() - t0

        t0 = perf_counter()
        map_pts_robot = extract_points_from_vertex(curr_submap, msg="pointmap")
        map_pts_lidar = convert_points_to_frame(map_pts_robot, T_lidar_robot)
        T_enu_lidar = Transformation(T_ba=lidar_frame.pose)
        map_pts_enu = convert_points_to_frame(map_pts_lidar, T_enu_lidar)
        map_points_gpu = torch.as_tensor(
            map_pts_enu.T,
            device=device,
            dtype=torch.float32,
        )
        map_time = perf_counter() - t0

        t0 = perf_counter()
        if data_augment:
            hard_round_patches, patch_stats = extract_augmented_depth_patches_gpu(
                map_points_gpu=map_points_gpu,
                azimuth_poses=azimuth_poses,
                radar_azimuths=radar_azimuths,
                patch_config=patch_config,
                geometry_batch_size=geometry_batch_size,
                device=device,
                rng=aug_rng,
                num_augments=num_augments,
                aug_trans_max_m=aug_trans_max_m,
                aug_rot_max_deg=aug_rot_max_deg,
            )
        else:
            hard_round_patches, patch_stats = extract_depth_patches_gpu(
                map_points_gpu=map_points_gpu,
                azimuth_poses=azimuth_poses,
                radar_azimuths=radar_azimuths,
                patch_config=patch_config,
                geometry_batch_size=geometry_batch_size,
                device=device,
            )
        patch_time = perf_counter() - t0

        t0 = perf_counter()
        suffix = f"_{method_suffix}" if method_suffix else ""
        method = f"hard_round{suffix}_aug" if data_augment else f"hard_round{suffix}"
        hard_round_path = save_patches(
            seq_dir=seq.seq_root,
            patches=hard_round_patches,
            fov_deg=patch_config["fov_deg"],
            radar_frame=radar_frame.frame,
            method=method,
        )
        save_time = perf_counter() - t0

        num_patches = max(
            hard_round_patches.shape[0]
            * (hard_round_patches.shape[1] if hard_round_patches.ndim == 4 else 1),
            1,
        )
        avg_selected = patch_stats["selected_points"] / num_patches
        print(
            f"Frame {radar_frame.frame} | total {perf_counter() - frame_start:.3f}s | "
            f"poses {pose_time:.3f}s | map->GPU {map_time:.3f}s | "
            f"patches {patch_time:.3f}s | save {save_time:.3f}s"
        )
        print(
            f" GPU projection wall {patch_stats['gpu_projection_wall']:.3f}s | "
            f"transform {patch_stats['point_transform_time_s']:.3f}s | "
            f"frustum {patch_stats['cartesian_frustum_mask_time_s']:.3f}s | "
            f"hard round {patch_stats['hard_round_time_s']:.3f}s | "
            f"GPU->CPU {patch_stats['patch_to_cpu_time_s']:.3f}s | "
            f"avg selected points {avg_selected:.1f}"
        )
        print(f" saved: {hard_round_path}")

        radar_frame.unload_data()
        del map_points_gpu
        radar_frame_idx += 1


def _self_check_hard_round():
    geom = GeometryParams(
        theta_min=-1.0,
        theta_max=1.0,
        phi_min=-1.0,
        phi_max=1.0,
        dtheta=1.0,
        dphi=1.0,
        width=4,
        height=3,
    )
    samples = torch.tensor(
        [[5.0, 1.2, 1.0], [2.0, 1.4, 1.0], [7.0, 3.0, 2.0], [1.0, -1.0, 0.0]]
    )
    batch_ids = torch.zeros(len(samples), dtype=torch.long)
    out = hard_round_depth_image_torch(samples, batch_ids, 1, geom).cpu().numpy()[0]
    expected = np.zeros((3, 4), dtype=np.float32)
    expected[1, 1] = 2.0
    expected[2, 3] = 7.0
    np.testing.assert_allclose(out, expected)


def main():
    parser = argparse.ArgumentParser(
        description="Generate submap depth patches with batched GPU hard rounding."
    )
    parser.add_argument("--radar-start-frame", type=int, default=65)
    parser.add_argument("--radar-end-frame", type=int, default=None)
    parser.add_argument("--sequence-id", default=None)
    parser.add_argument("--geometry-batch-size", type=int, default=400)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--fov-deg", type=float, default=6.0)
    parser.add_argument("--res-deg", type=float, default=0.1)
    parser.add_argument("--data-augment", action="store_true")
    parser.add_argument("--num-augments", type=int, default=8)
    parser.add_argument("--aug-trans-max-m", type=float, default=0.01)
    parser.add_argument("--aug-rot-max-deg", type=float, default=0.05)
    parser.add_argument("--aug-seed", type=int, default=0)
    parser.add_argument(
        "--method-suffix",
        default="gpu",
        help="Suffix output patch folder.",
    )
    parser.add_argument(
        "--self-check",
        action="store_true",
        help="Run a tiny hard-round rasterization check and exit.",
    )
    args = parser.parse_args()

    if args.self_check:
        _self_check_hard_round()
        return

    if args.geometry_batch_size < 1:
        raise ValueError("--geometry-batch-size must be at least 1.")
    if args.num_augments < 0:
        raise ValueError("--num-augments must be non-negative.")
    if args.aug_trans_max_m < 0.0:
        raise ValueError("--aug-trans-max-m must be non-negative.")
    if args.aug_rot_max_deg < 0.0:
        raise ValueError("--aug-rot-max-deg must be non-negative.")

    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA requested but is not available.")

    boreas_vtr_wrapper_dir = os.getenv("VTRROOT")
    boreas_data = os.getenv("VTRRDATA")
    if boreas_vtr_wrapper_dir is None:
        raise RuntimeError("VTRROOT must be set.")
    if boreas_data is None:
        raise RuntimeError("VTRRDATA must be set.")

    lidar_results_dir = os.path.join(boreas_vtr_wrapper_dir, "results/lidar")
    dataset = BoreasDataset(boreas_data)
    patch_config = build_patch_config(
        fov_deg=args.fov_deg,
        res_deg=args.res_deg,
        min_range=0.0,
        max_uv_edge_length=None,
        max_depth_jump=2.0,
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
            radar_start_frame=args.radar_start_frame,
            radar_end_frame=args.radar_end_frame,
            patch_config=patch_config,
            geometry_batch_size=args.geometry_batch_size,
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
