import argparse
import csv
import os
from pathlib import Path
from time import perf_counter

import numpy as np
import torch
from pyboreas import BoreasDataset
from pyboreas.utils.odometry import interpolate_poses
from pyboreas.utils.utils import get_inverse_tf
from scipy.spatial.transform import Rotation as R

from localization_dfo.io_utils import (
    build_lidar_to_robot_transform,
    build_patch_config,
    cen_filter_2d,
    correct_offsets,
    get_submap_vertices,
    load_submap_mesh_to_enu,
)
from localization_dfo.mesh_cost import (
    GeometryParams,
    ResidualBuildOptions,
    compute_radar_cost_only_fast,
    left_se3_retract,
)
from localization_dfo.optimizer import ObjectiveLogger, extract_best_from_result, run_imfil_direct
from localization_dfo.radar_translator_cnn import RadarTranslatorCNN
from localization_dfo.transforms import make_delta_T


RESULT_FIELDNAMES = [
    "sequence_id",
    "frame_id",
    "initial_x_m",
    "initial_y_m",
    "initial_z_m",
    "initial_roll_deg",
    "initial_pitch_deg",
    "initial_yaw_deg",
    "final_x_m",
    "final_y_m",
    "final_z_m",
    "final_roll_deg",
    "final_pitch_deg",
    "final_yaw_deg",
    "initial_cost",
    "final_cost",
    "iterations",
]

MESH_TIMING_KEYS = [
    ("point_transform_time_s", "point transform"),
    ("cartesian_frustum_mask_time_s", "cartesian frustum mask"),
    ("candidate_face_selection_time_s", "candidate face selection"),
    ("selected_geometry_time_s", "selected geometry projection"),
    ("mesh_rasterization_time_s", "mesh rasterization"),
    ("geometry_to_cpu_time_s", "geometry GPU to CPU"),
]

RASTER_TIMING_KEYS = [
    ("raster_face_filter_time_s", "face filter"),
    ("raster_bbox_time_s", "pixel bounding boxes"),
    ("raster_pair_generation_time_s", "face-pixel pair generation"),
    ("raster_barycentric_time_s", "barycentric weights"),
    ("raster_depth_interpolation_time_s", "depth interpolation"),
    ("raster_zbuffer_time_s", "z-buffer scatter"),
    ("raster_finalize_time_s", "finalize output/stats"),
]


def load_radar_translator_model(weights_path, device):
    model = RadarTranslatorCNN().to(device)
    checkpoint = torch.load(weights_path, map_location=device)
    if isinstance(checkpoint, dict) and "model_state_dict" in checkpoint:
        state_dict = checkpoint["model_state_dict"]
    elif isinstance(checkpoint, dict) and "state_dict" in checkpoint:
        state_dict = checkpoint["state_dict"]
    else:
        state_dict = checkpoint
    if isinstance(state_dict, dict) and any(key.startswith("module.") for key in state_dict):
        state_dict = {key.removeprefix("module."): value for key, value in state_dict.items()}
    model.load_state_dict(state_dict)
    model.eval()
    return model


def pose_error_xyz_rpy(T_est, T_gt):
    T_error = T_est @ np.linalg.inv(T_gt)
    xyz = T_error[:3, 3]
    rpy_deg = R.from_matrix(T_error[:3, :3]).as_euler("xyz", degrees=True)
    return xyz, rpy_deg


def build_localization_result_row(sequence_id, frame_id, T_init, T_hat, T_gt, history):
    init_xyz, init_rpy_deg = pose_error_xyz_rpy(T_init, T_gt)
    final_xyz, final_rpy_deg = pose_error_xyz_rpy(T_hat, T_gt)
    has_history = len(history) > 0
    idx = int(np.argmin(history)) if has_history else 0

    return {
        "sequence_id": sequence_id,
        "frame_id": frame_id,
        "initial_x_m": float(init_xyz[0]),
        "initial_y_m": float(init_xyz[1]),
        "initial_z_m": float(init_xyz[2]),
        "initial_roll_deg": float(init_rpy_deg[0]),
        "initial_pitch_deg": float(init_rpy_deg[1]),
        "initial_yaw_deg": float(init_rpy_deg[2]),
        "final_x_m": float(final_xyz[0]),
        "final_y_m": float(final_xyz[1]),
        "final_z_m": float(final_xyz[2]),
        "final_roll_deg": float(final_rpy_deg[0]),
        "final_pitch_deg": float(final_rpy_deg[1]),
        "final_yaw_deg": float(final_rpy_deg[2]),
        "initial_cost": float(history[0]) if has_history else float("nan"),
        "final_cost": float(history[idx]) if has_history else float("nan"),
        "iterations": len(history),
    }


def initialize_results_csv(csv_path, append=False, overwrite=False):
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    if csv_path.exists():
        if overwrite:
            csv_path.unlink()
        elif not append:
            raise FileExistsError(
                f"Results CSV already exists: {csv_path}. Use --append or --overwrite explicitly."
            )

    if not csv_path.exists():
        with csv_path.open("w", newline="") as f:
            csv.DictWriter(f, fieldnames=RESULT_FIELDNAMES).writeheader()


def append_localization_result(csv_path, result_row):
    with csv_path.open("a", newline="") as f:
        csv.DictWriter(f, fieldnames=RESULT_FIELDNAMES).writerow(result_row)


def print_dfo_timing_report(optimization_time_s, logger, cost_call_diagnostics):
    cost_calls = len(logger.history_f)
    print("DFO timing")
    print(f"  optimization wall time [s]: {optimization_time_s:.6f}")
    print(f"  cost function calls: {cost_calls}")
    if cost_calls:
        print(f"  mean wall time / cost call [s]: {optimization_time_s / cost_calls:.6f}")

    cost_wall_time_s = sum(call["wall_time_s"] for call in cost_call_diagnostics)
    if cost_wall_time_s:
        print(f"  total forward cost wall time [s]: {cost_wall_time_s:.6f}")
    if not cost_call_diagnostics:
        return

    top_buckets = [
        ("depth_patch_generation_time_s", "depth patch generation"),
        ("model_inference_time_s", "model inference"),
        ("cost_compute_time_s", "cost computation"),
    ]
    totals = {key: sum(call[key] for call in cost_call_diagnostics) for key, _ in top_buckets}
    mesh_totals = {
        key: sum(call["depth_patch_breakdown"].get(key, 0.0) for call in cost_call_diagnostics)
        for key, _ in MESH_TIMING_KEYS
    }
    raster_totals = {
        key: sum(call["raster_breakdown"].get(key, 0.0) for call in cost_call_diagnostics)
        for key, _ in RASTER_TIMING_KEYS
    }
    first = cost_call_diagnostics[0]

    print("  first forward cost call [s]:")
    print(f"    depth patches: {first['patch_count']}")
    for key, label in top_buckets:
        print(f"    {label}: {first[key]:.6f}")
    print("    depth patch generation breakdown:")
    for key, label in MESH_TIMING_KEYS:
        print(f"      {label}: {first['depth_patch_breakdown'].get(key, 0.0):.6f}")
    print("    mesh rasterization breakdown:")
    for key, label in RASTER_TIMING_KEYS:
        print(f"      {label}: {first['raster_breakdown'].get(key, 0.0):.6f}")

    print("  total forward cost breakdown [s]:")
    for key, label in top_buckets:
        mean_s = totals[key] / max(cost_calls, 1)
        print(f"    {label}: {totals[key]:.6f} total, {mean_s:.6f}/call")
    print("  total depth patch generation breakdown [s]:")
    for key, label in MESH_TIMING_KEYS:
        mean_s = mesh_totals[key] / max(cost_calls, 1)
        print(f"    {label}: {mesh_totals[key]:.6f} total, {mean_s:.6f}/call")
    print("  total mesh rasterization breakdown [s]:")
    for key, label in RASTER_TIMING_KEYS:
        mean_s = raster_totals[key] / max(cost_calls, 1)
        print(f"    {label}: {raster_totals[key]:.6f} total, {mean_s:.6f}/call")


def print_localization_error_report(T_init, T_hat, T_gt):
    init_xyz, init_rpy_deg = pose_error_xyz_rpy(T_init, T_gt)
    final_xyz, final_rpy_deg = pose_error_xyz_rpy(T_hat, T_gt)

    print("Localization error")
    print(f"  initial xyz [m]:        {init_xyz}")
    print(f"  initial rpy [deg]:      {init_rpy_deg}")
    print(f"  final xyz [m]:          {final_xyz}")
    print(f"  final rpy [deg]:        {final_rpy_deg}")
    print(f"  initial trans norm [m]: {np.linalg.norm(init_xyz):.6e}")
    print(f"  final trans norm [m]:   {np.linalg.norm(final_xyz):.6e}")
    print(f"  initial rot norm [deg]: {np.linalg.norm(init_rpy_deg):.6e}")
    print(f"  final rot norm [deg]:   {np.linalg.norm(final_rpy_deg):.6e}")


def geometry_params_from_config(patch_config):
    return GeometryParams(
        theta_min=patch_config["theta_min"],
        theta_max=patch_config["theta_max"],
        phi_min=patch_config["phi_min"],
        phi_max=patch_config["phi_max"],
        dtheta=patch_config["dtheta"],
        dphi=patch_config["dphi"],
        width=patch_config["width"],
        height=patch_config["height"],
        max_uv_edge_length=patch_config["max_uv_edge_length"],
        max_depth_jump=patch_config["max_depth_jump"],
        fill_value=patch_config["fill_value"],
    )


def summarize_cost_call(result, wall_time_s):
    breakdown = {key: 0.0 for key, _ in MESH_TIMING_KEYS}
    raster_breakdown = {key: 0.0 for key, _ in RASTER_TIMING_KEYS}
    depth_patch_generation_time_s = 0.0
    model_inference_time_s = 0.0
    cost_compute_time_s = 0.0

    for diag in result.diagnostics:
        timing = diag.get("timing", {})
        depth_patch_generation_time_s += float(timing.get("cost_forward_time_s", 0.0))
        model_inference_time_s += float(timing.get("model_forward_time_s", 0.0))
        cost_compute_time_s += float(timing.get("cost_compute_time_s", 0.0))
        for key in breakdown:
            breakdown[key] += float(timing.get(key, 0.0))
        for key in raster_breakdown:
            raster_breakdown[key] += float(timing.get(key, 0.0))

    return {
        "wall_time_s": wall_time_s,
        "cost": result.cost,
        "patch_count": len(result.diagnostics),
        "depth_patch_generation_time_s": depth_patch_generation_time_s,
        "depth_patch_breakdown": breakdown,
        "raster_breakdown": raster_breakdown,
        "model_inference_time_s": model_inference_time_s,
        "cost_compute_time_s": cost_compute_time_s,
    }


def run_sequence(
    seq,
    lidar_results_dir,
    radar_start_frame,
    radar_end_frame,
    patch_config,
    model,
    device,
    results_csv_path,
    mesh_root,
):
    print(f"SequenceID: {seq.ID}")
    print(f"Number of Radar Frames: {len(seq.radar_frames)}")

    end_frame = len(seq.radar_frames) - 2 if radar_end_frame is None else min(radar_end_frame, len(seq.radar_frames) - 2)
    radar_start_frame = max(radar_start_frame, 1)

    graph_dir = os.path.join(lidar_results_dir, seq.ID, seq.ID, "graph")
    _, submap_vertices = get_submap_vertices(graph_dir=graph_dir)
    T_lidar_robot = build_lidar_to_robot_transform(seq)

    lidar_frame_idx = 0
    submap_vertices_idx = 0
    radar_frame_idx = radar_start_frame
    loaded_mesh_submap_stamp_us = None
    mesh_vertices_gpu = None
    mesh_triangles_gpu = None

    while (
        submap_vertices_idx < len(submap_vertices) - 1
        and lidar_frame_idx < len(seq.lidar_frames)
        and radar_frame_idx < end_frame + 1
    ):
        curr_submap = submap_vertices[submap_vertices_idx]
        next_submap = submap_vertices[submap_vertices_idx + 1]
        radar_frame = seq.radar_frames[radar_frame_idx]
        lidar_frame = seq.lidar_frames[lidar_frame_idx]

        if int(radar_frame.frame) < curr_submap.stamp // 1000:
            radar_frame_idx += 1
            continue
        if next_submap.stamp // 1000 < int(radar_frame.frame):
            submap_vertices_idx += 1
            continue
        if int(lidar_frame.frame) != curr_submap.stamp // 1000:
            lidar_frame_idx += 1
            continue

        radar_frame = seq.get_radar(radar_frame_idx)
        poses = [
            get_inverse_tf(rad_frame.pose)
            for rad_frame in seq.radar_frames[radar_frame_idx - 1:radar_frame_idx + 2]
        ]
        times = [
            rad_frame.timestamp_micro
            for rad_frame in seq.radar_frames[radar_frame_idx - 1:radar_frame_idx + 2]
        ]
        azimuth_poses = interpolate_poses(poses, times, radar_frame.timestamps.flatten().tolist())
        radar_azimuths = radar_frame.azimuths.flatten()

        T_enu_radar = radar_frame.pose
        odom_transforms = np.array([T_enu_radar @ T_i for T_i in azimuth_poses])

        submap_stamp_us = curr_submap.stamp // 1000
        if loaded_mesh_submap_stamp_us != submap_stamp_us:
            mesh_vertices_gpu, mesh_triangles_gpu, _ = load_submap_mesh_to_enu(
                mesh_root=mesh_root,
                sequence_id=seq.ID,
                submap=curr_submap,
                T_lidar_robot=T_lidar_robot,
                lidar_pose=lidar_frame.pose,
                device=device,
            )
            loaded_mesh_submap_stamp_us = submap_stamp_us

        shifted_polar = correct_offsets(radar_frame, radar_frame_idx, seq)
        filtered_polar = cen_filter_2d(shifted_polar, sigma_gauss=15.0, z_q=2.5, noise_scale=0.5)

        T_offset = make_delta_T(translation=np.array([0.2, 0.2, 0.2]), rpy_deg=np.array([2.0, 2.0, 2.0]))
        T_gt = np.linalg.inv(T_enu_radar)
        T_init = T_offset @ T_gt

        radar_polar_cropped = filtered_polar[:, :2736] / 0.5613
        geom = geometry_params_from_config(patch_config)
        residual_options = ResidualBuildOptions(
            device=str(device),
            model_output_activation="sigmoid",
            candidate_batch_size=400,
            geometry_batch_size=400,
        )

        cost_call_diagnostics = []

        def cost_fn(eps):
            t_call = perf_counter()
            T_current = left_se3_retract(T=T_init, delta=eps)
            result = compute_radar_cost_only_fast(
                P_v=mesh_vertices_gpu,
                mesh_triangles=mesh_triangles_gpu,
                T=T_current,
                odom_transforms=odom_transforms,
                m_obs_all=radar_polar_cropped,
                model=model,
                geom=geom,
                options=residual_options,
                radar_azimuths=radar_azimuths,
            )
            cost_call_diagnostics.append(summarize_cost_call(result, perf_counter() - t_call))
            return result.cost

        logger = ObjectiveLogger(cost_fn)
        eps0 = np.zeros(6)
        bounds = np.array(
            [
                [-0.2, 0.2],
                [-0.2, 0.2],
                [-0.2, 0.2],
                [-2.0, 2.0],
                [-2.0, 2.0],
                [-2.0, 2.0],
            ],
            dtype=float,
        )
        bounds[3:] = np.deg2rad(bounds[3:])

        t_opt = perf_counter()
        result, _ = run_imfil_direct("imfil", logger, eps0, bounds, 120)
        optimization_time_s = perf_counter() - t_opt
        eps_best, _ = extract_best_from_result(result, logger)
        T_hat = left_se3_retract(T=T_init, delta=eps_best)

        print_dfo_timing_report(optimization_time_s, logger, cost_call_diagnostics)
        print_localization_error_report(T_init, T_hat, T_gt)
        append_localization_result(
            results_csv_path,
            build_localization_result_row(seq.ID, radar_frame.frame, T_init, T_hat, T_gt, logger.history_f),
        )
        print(f"Saved localization result: {results_csv_path}")

        radar_frame.unload_data()
        print("radar frame unloaded!")
        radar_frame_idx += 100


def main():
    parser = argparse.ArgumentParser(description="Run torch-mesh DFO radar-lidar localization.")
    parser.add_argument("--experiment-name", required=True)
    output_mode = parser.add_mutually_exclusive_group()
    output_mode.add_argument("--append", action="store_true")
    output_mode.add_argument("--overwrite", action="store_true")
    parser.add_argument(
        "--mesh-root",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "postprocessing" / "submap_meshes",
    )
    args = parser.parse_args()

    experiment_name = Path(args.experiment_name).name
    if experiment_name != args.experiment_name or experiment_name in {"", ".", ".."}:
        raise ValueError("--experiment-name must be a simple filename-safe name.")

    boreas_vtr_wrapper_dir = os.getenv("VTRROOT")
    boreas_data = os.getenv("VTRRDATA")
    if boreas_vtr_wrapper_dir is None:
        raise RuntimeError("VTRROOT must be set.")
    if boreas_data is None:
        raise RuntimeError("VTRRDATA must be set.")

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    lidar_results_dir = os.path.join(boreas_vtr_wrapper_dir, "results/lidar")
    weights_path = os.path.join(
        boreas_vtr_wrapper_dir,
        "model_dev/model_weights/6_deg_attentional_skip_bigger/best.pth",
    )
    model = load_radar_translator_model(weights_path, device)
    dataset = BoreasDataset(boreas_data)

    patch_config = build_patch_config(
        fov_deg=6.0,
        res_deg=0.1,
        min_range=0.0,
        max_uv_edge_length=None,
        max_depth_jump=2.0,
    )
    patch_config["fov_deg"] = 6.0

    for seq in dataset.sequences:
        results_csv_path = (
            Path(boreas_vtr_wrapper_dir)
            / "localization_dfo"
            / "results"
            / seq.ID
            / f"{experiment_name}.csv"
        )
        initialize_results_csv(results_csv_path, append=args.append, overwrite=args.overwrite)
        run_sequence(
            seq=seq,
            lidar_results_dir=lidar_results_dir,
            radar_start_frame=65,
            radar_end_frame=None,
            patch_config=patch_config,
            model=model,
            device=device,
            results_csv_path=results_csv_path,
            mesh_root=args.mesh_root,
        )
        break


if __name__ == "__main__":
    main()
