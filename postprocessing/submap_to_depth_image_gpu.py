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
from vtr_utils.plot_utils import convert_points_to_frame, extract_points_from_vertex

from localization_fast.gauss_newton_localization_fast import (
    GeometryParams,
    torch_frustum_pixel_geometry_batch,
)
from postprocessing.submap_to_depth_image_multiprocess_refactored import (
    barycentric_interpolate_depth_image,
    build_lidar_to_robot_transform,
    build_patch_config,
    get_submap_vertices,
    hard_round_depth_image,
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
    delaunay_patches = np.empty(
        (num_azimuths, patch_config["height"], patch_config["width"]),
        dtype=np.float32,
    )
    hard_round_patches = np.empty_like(delaunay_patches)
    stats = defaultdict(float)

    for start in range(0, num_azimuths, geometry_batch_size):
        stop = min(start + geometry_batch_size, num_azimuths)
        batch_indices = slice(start, stop)

        t0 = perf_counter()
        pixel_outputs, gpu_timing = torch_frustum_pixel_geometry_batch(
            P_v=map_points_gpu,
            T=np.eye(4),
            odom_batch=azimuth_poses[batch_indices],
            radar_azimuth_batch=radar_azimuths[batch_indices],
            geom=geom,
            device=device,
            with_jacobian=False,
        )
        stats["gpu_projection_wall"] += perf_counter() - t0

        batch_count = stop - start
        for key, value in gpu_timing.items():
            stats[key] += value * batch_count

        for local_idx, (I, _, selected_count) in enumerate(pixel_outputs):
            output_idx = start + local_idx
            samples = I[:, :, 0]

            t0 = perf_counter()
            hard_round_patches[output_idx] = hard_round_depth_image(samples, patch_config)
            stats["hard_round"] += perf_counter() - t0

            t0 = perf_counter()
            delaunay_patches[output_idx] = barycentric_interpolate_depth_image(samples, patch_config)
            stats["delaunay"] += perf_counter() - t0
            stats["selected_points"] += selected_count

    return delaunay_patches, hard_round_patches, dict(stats)


def run_sequence(
    seq,
    lidar_results_dir,
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
        delaunay_patches, hard_round_patches, patch_stats = extract_depth_patches_gpu(
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
        delaunay_path = save_patches(
            seq_dir=seq.seq_root,
            patches=delaunay_patches,
            fov_deg=patch_config["fov_deg"],
            radar_frame=radar_frame.frame,
            method=f"delaunay{suffix}",
        )
        hard_round_path = save_patches(
            seq_dir=seq.seq_root,
            patches=hard_round_patches,
            fov_deg=patch_config["fov_deg"],
            radar_frame=radar_frame.frame,
            method=f"hard_round{suffix}",
        )
        save_time = perf_counter() - t0

        num_patches = max(len(azimuth_poses), 1)
        avg_selected = patch_stats["selected_points"] / num_patches
        print(
            f"Frame {radar_frame.frame} | total {perf_counter() - frame_start:.3f}s | "
            f"poses {pose_time:.3f}s | map->GPU {map_time:.3f}s | "
            f"patches {patch_time:.3f}s | save {save_time:.3f}s"
        )
        print(
            f"  GPU projection wall {patch_stats['gpu_projection_wall']:.3f}s | "
            f"transform {patch_stats['point_transform_time_s']:.3f}s | "
            f"frustum {patch_stats['cartesian_frustum_mask_time_s']:.3f}s | "
            f"selected geometry {patch_stats['selected_geometry_time_s']:.3f}s | "
            f"GPU->CPU {patch_stats['geometry_to_cpu_time_s']:.3f}s"
        )
        print(
            f"  hard round {patch_stats['hard_round']:.3f}s | "
            f"Delaunay {patch_stats['delaunay']:.3f}s | "
            f"avg selected points {avg_selected:.1f}"
        )
        print(f"  saved: {delaunay_path}")
        print(f"  saved: {hard_round_path}")

        radar_frame.unload_data()
        del map_points_gpu
        radar_frame_idx += 1


def main():
    parser = argparse.ArgumentParser(
        description="Generate submap depth patches with batched GPU frustum projection."
    )
    parser.add_argument("--radar-start-frame", type=int, default=65)
    parser.add_argument("--radar-end-frame", type=int, default=None)
    parser.add_argument("--sequence-id", default=None)
    parser.add_argument("--geometry-batch-size", type=int, default=16)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--fov-deg", type=float, default=6.0)
    parser.add_argument("--res-deg", type=float, default=0.1)
    parser.add_argument(
        "--method-suffix",
        default="gpu",
        help="Suffix for output patch folders. Use an empty string to match legacy folder names.",
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
        )
        if args.sequence_id is None:
            break

    if not matched_sequence:
        raise ValueError(f"Sequence not found: {args.sequence_id}")


if __name__ == "__main__":
    main()
