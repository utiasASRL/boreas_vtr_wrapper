#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
from collections import defaultdict
from pathlib import Path
from time import perf_counter

import numpy as np
import torch
from pyboreas import BoreasDataset
from pyboreas.utils.odometry import interpolate_poses
from pyboreas.utils.utils import get_inverse_tf
from pylgmath import Transformation
from vtr_utils.plot_utils import convert_points_to_frame

from localization_fast.gauss_newton_localization_fast import (
    GeometryParams,
    torch_mesh_depth_geometry_batch,
)
from postprocessing.submap_to_depth_image_multiprocess_refactored import (
    build_lidar_to_robot_transform,
    build_patch_config,
    get_submap_vertices,
)


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
    )
    triangles_gpu = torch.as_tensor(
        triangles,
        device=device,
        dtype=torch.long,
    )
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


def extract_depth_patches_mesh_gpu(
    mesh_vertices_gpu,
    mesh_triangles_gpu,
    azimuth_poses,
    radar_azimuths,
    patch_config,
    geometry_batch_size,
    device,
):
    geom = geometry_params_from_patch_config(patch_config)
    num_azimuths = len(azimuth_poses)
    mesh_patches = np.empty(
        (num_azimuths, patch_config["height"], patch_config["width"]),
        dtype=np.float32,
    )
    stats = defaultdict(float)

    for start in range(0, num_azimuths, geometry_batch_size):
        stop = min(start + geometry_batch_size, num_azimuths)
        batch_indices = slice(start, stop)

        t0 = perf_counter()
        mesh_outputs, gpu_timing = torch_mesh_depth_geometry_batch(
            P_v=mesh_vertices_gpu,
            triangles=mesh_triangles_gpu,
            T=np.eye(4),
            odom_batch=azimuth_poses[batch_indices],
            radar_azimuth_batch=radar_azimuths[batch_indices],
            geom=geom,
            device=device,
            with_jacobian=False,
        )
        stats["gpu_mesh_wall"] += perf_counter() - t0

        batch_count = stop - start
        for key, value in gpu_timing.items():
            stats[key] += value * batch_count

        for local_idx, (depth_patch, _, mesh_stats) in enumerate(mesh_outputs):
            output_idx = start + local_idx
            mesh_patches[output_idx] = depth_patch
            for key, value in mesh_stats.items():
                stats[key] += value

    return mesh_patches, dict(stats)


def run_sequence(
    seq,
    lidar_results_dir,
    mesh_root,
    radar_start_frame,
    radar_end_frame,
    patch_config,
    geometry_batch_size,
    device,
    method_suffix,
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

        t0 = perf_counter()
        mesh_patches, patch_stats = extract_depth_patches_mesh_gpu(
            mesh_vertices_gpu=mesh_vertices_gpu,
            mesh_triangles_gpu=mesh_triangles_gpu,
            azimuth_poses=azimuth_poses,
            radar_azimuths=radar_azimuths,
            patch_config=patch_config,
            geometry_batch_size=geometry_batch_size,
            device=device,
        )
        patch_time = perf_counter() - t0

        t0 = perf_counter()
        suffix = f"_{method_suffix}" if method_suffix else ""
        mesh_path = save_patches(
            seq_dir=seq.seq_root,
            patches=mesh_patches,
            fov_deg=patch_config["fov_deg"],
            radar_frame=radar_frame.frame,
            method=f"mesh{suffix}",
        )
        save_time = perf_counter() - t0

        num_patches = max(len(azimuth_poses), 1)
        print(
            f"Frame {radar_frame.frame} | total {perf_counter() - frame_start:.3f}s | "
            f"poses {pose_time:.3f}s | patches {patch_time:.3f}s | "
            f"save {save_time:.3f}s"
        )
        if mesh_load_stats["mesh_disk_load"] > 0.0:
            print(
                f"  mesh load {mesh_load_stats['mesh_disk_load']:.3f}s | "
                f"robot->ENU/GPU {mesh_load_stats['mesh_robot_to_enu_upload']:.3f}s"
            )
        print(
            f"  mesh GPU wall {patch_stats['gpu_mesh_wall']:.3f}s | "
            f"transform {patch_stats['point_transform_time_s']:.3f}s | "
            f"frustum {patch_stats['cartesian_frustum_mask_time_s']:.3f}s | "
            f"face select {patch_stats['candidate_face_selection_time_s']:.3f}s | "
            f"projection {patch_stats['selected_geometry_time_s']:.3f}s | "
            f"raster {patch_stats['mesh_rasterization_time_s']:.3f}s | "
            f"GPU->CPU {patch_stats['geometry_to_cpu_time_s']:.3f}s"
        )
        print(
            f"  avg selected vertices {patch_stats['selected_vertices'] / num_patches:.1f} | "
            f"avg candidate faces {patch_stats['candidate_faces'] / num_patches:.1f} | "
            f"avg triangle-pixel tests "
            f"{patch_stats['triangle_pixel_tests'] / num_patches:.1f} | "
            f"avg covered pixels {patch_stats['covered_pixels'] / num_patches:.1f}"
        )
        print(f"  saved: {mesh_path}")

        radar_frame.unload_data()
        radar_frame_idx += 1


def main():
    parser = argparse.ArgumentParser(
        description="Generate radar-azimuth depth patches from cached submap meshes."
    )
    parser.add_argument("--radar-start-frame", type=int, default=65)
    parser.add_argument("--radar-end-frame", type=int, default=None)
    parser.add_argument("--sequence-id", default=None)
    parser.add_argument("--geometry-batch-size", type=int, default=400)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--fov-deg", type=float, default=6.0)
    parser.add_argument("--res-deg", type=float, default=0.1)
    parser.add_argument(
        "--mesh-root",
        type=Path,
        default=Path(__file__).resolve().parent / "submap_meshes",
    )
    parser.add_argument(
        "--method-suffix",
        default="gpu",
        help="Suffix for the output patch folder.",
    )
    args = parser.parse_args()
    if args.geometry_batch_size < 1:
        raise ValueError("--geometry-batch-size must be at least 1.")

    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested but is not available.")

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
            mesh_root=args.mesh_root,
            radar_start_frame=args.radar_start_frame,
            radar_end_frame=args.radar_end_frame,
            patch_config=patch_config,
            geometry_batch_size=args.geometry_batch_size,
            device=device,
            method_suffix=args.method_suffix,
        )
        if args.sequence_id is None:
            break

    if not matched_sequence:
        raise ValueError(f"Sequence not found: {args.sequence_id}")


if __name__ == "__main__":
    main()
