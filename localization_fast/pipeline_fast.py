import argparse
import csv
import os
from pathlib import Path

import numpy as np
import torch
from pyboreas import BoreasDataset
from pyboreas.utils.odometry import interpolate_poses
from pyboreas.utils.utils import get_inverse_tf
from pylgmath import Transformation
from scipy.spatial.transform import Rotation as R
from vtr_utils.plot_utils import convert_points_to_frame, extract_points_from_vertex

from localization.pipeline import (
    build_lidar_to_robot_transform,
    build_patch_config,
    cen_filter_2d,
    correct_offsets,
    get_submap_vertices,
)
from localization_fast.gauss_newton_localization_fast import (
    GeometryParams,
    ResidualBuildOptions,
    run_radar_lidar_localization_gn_fast,
)
from perturbation_cost_tests.perturbation_utils import make_delta_T
from perturbation_cost_tests.radar_translator_cnn import RadarTranslatorCNN
from postprocessing.mesh_to_depth_image import load_submap_mesh_to_enu


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
    "accepted_steps",
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
    return xyz, rpy_deg, T_error


def build_localization_result_row(sequence_id, frame_id, T_init, T_hat, T_gt, history):
    init_xyz, init_rpy_deg, _ = pose_error_xyz_rpy(T_init, T_gt)
    final_xyz, final_rpy_deg, _ = pose_error_xyz_rpy(T_hat, T_gt)

    accepted = [row for row in history if row["accepted"]]
    initial_cost = float(history[0]["cost"]) if history else float("nan")
    final_cost = float(history[-1].get("best_cost", initial_cost)) if history else initial_cost

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
        "initial_cost": initial_cost,
        "final_cost": final_cost,
        "iterations": len(history),
        "accepted_steps": len(accepted),
    }


def initialize_results_csv(csv_path, append=False, overwrite=False):
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    if csv_path.exists():
        if overwrite:
            csv_path.unlink()
        elif not append:
            raise FileExistsError(
                f"Results CSV already exists: {csv_path}. "
                "Use --append or --overwrite explicitly."
            )

    if not csv_path.exists():
        with csv_path.open("w", newline="") as f:
            csv.DictWriter(f, fieldnames=RESULT_FIELDNAMES).writeheader()


def append_localization_result(csv_path, result_row):
    with csv_path.open("a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=RESULT_FIELDNAMES)
        writer.writerow(result_row)


def print_localization_error_report(T_init, T_hat, T_gt, history):
    init_xyz, init_rpy_deg, _ = pose_error_xyz_rpy(T_init, T_gt)
    final_xyz, final_rpy_deg, _ = pose_error_xyz_rpy(T_hat, T_gt)

    print("Localization error")
    print(f"  initial xyz [m]:        {init_xyz}")
    print(f"  initial rpy [deg]:      {init_rpy_deg}")
    print(f"  final xyz [m]:          {final_xyz}")
    print(f"  final rpy [deg]:        {final_rpy_deg}")
    print(f"  initial trans norm [m]: {np.linalg.norm(init_xyz):.6e}")
    print(f"  final trans norm [m]:   {np.linalg.norm(final_xyz):.6e}")
    print(f"  initial rot norm [deg]: {np.linalg.norm(init_rpy_deg):.6e}")
    print(f"  final rot norm [deg]:   {np.linalg.norm(final_rpy_deg):.6e}")

    if history:
        accepted = [row for row in history if row["accepted"]]
        final_cost = history[-1].get("best_cost", history[-1]["cost"])
        print(f"  initial cost:           {history[0]['cost']:.6e}")
        print(f"  final accepted cost:    {final_cost:.6e}")
        print(f"  accepted steps:         {len(accepted)} / {len(history)}")


def run_sequence(
    seq,
    lidar_results_dir,
    radar_start_frame,
    radar_end_frame,
    patch_config,
    model,
    device,
    results_csv_path,
    geometry_backend,
    geometry_batch_size,
    mesh_root,
):
    print(f"SequenceID: {seq.ID}")
    print(f"Number of Radar Frames: {len(seq.radar_frames)}")

    end_frame = radar_end_frame
    if end_frame is None:
        end_frame = len(seq.radar_frames) - 2
    else:
        end_frame = min(end_frame, len(seq.radar_frames) - 2)

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
        query_times = radar_frame.timestamps.flatten().tolist()
        azimuth_poses = interpolate_poses(poses, times, query_times)
        radar_azimuths = radar_frame.azimuths.flatten()

        T_enu_radar = radar_frame.pose
        odom_transforms = np.array([
            T_enu_radar @ T_i
            for T_i in azimuth_poses
        ])

        mesh_triangles = None
        if geometry_backend == "torch_mesh":
            submap_stamp_us = curr_submap.stamp // 1000
            if loaded_mesh_submap_stamp_us != submap_stamp_us:
                del mesh_vertices_gpu, mesh_triangles_gpu
                mesh_vertices_gpu, mesh_triangles_gpu, _ = load_submap_mesh_to_enu(
                    mesh_root=mesh_root,
                    sequence_id=seq.ID,
                    submap=curr_submap,
                    T_lidar_robot=T_lidar_robot,
                    lidar_pose=lidar_frame.pose,
                    device=device,
                )
                loaded_mesh_submap_stamp_us = submap_stamp_us
            P_v = mesh_vertices_gpu
            mesh_triangles = mesh_triangles_gpu
        else:
            map_pts_robot = extract_points_from_vertex(curr_submap, msg="pointmap")
            map_pts_lidar = convert_points_to_frame(map_pts_robot, T_lidar_robot)

            T_enu_lidar = Transformation(T_ba=lidar_frame.pose)
            map_pts_enu = convert_points_to_frame(map_pts_lidar, T_enu_lidar)

            if geometry_backend == "torch_frustum":
                P_v = torch.as_tensor(
                    map_pts_enu.T,
                    device=device,
                    dtype=torch.float32,
                )
            else:
                N = map_pts_enu.shape[1]
                P_v = np.ones((N, 4, 1))
                P_v[:, :3, 0] = map_pts_enu.T

        shifted_polar = correct_offsets(radar_frame, radar_frame_idx, seq)
        filtered_polar = cen_filter_2d(shifted_polar, sigma_gauss=15.0, z_q=2.5, noise_scale=0.5)

        # Current debug setup: yaw-only localization from a known yaw offset.
        # T_offset = make_delta_T(rpy_deg=np.array([2.0, 2.0, 2.0]))
        T_offset = make_delta_T(translation=np.array([0.3, 0.3, 0.3]))
        T_gt = np.linalg.inv(T_enu_radar)
        T_init = T_offset @ T_gt

        norm_factor = 0.5613
        radar_polar_cropped = filtered_polar[:, :2736] / norm_factor

        geom = GeometryParams(
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

        active_dims = [0, 1, 2]  # [x, y, z, roll, pitch, yaw] -> yaw only.
        residual_options = ResidualBuildOptions(
            device=str(device),
            model_output_activation="sigmoid",
            active_dims=active_dims,
            linearization_batch_size=100,
            candidate_batch_size=100,
            geometry_backend=geometry_backend,
            geometry_batch_size=geometry_batch_size,
        )

        result = run_radar_lidar_localization_gn_fast(
            P_v=P_v,
            mesh_triangles=mesh_triangles,
            T_init=T_init,
            odom_transforms=odom_transforms,
            m_obs_all=radar_polar_cropped,
            model=model,
            geom=geom,
            options=residual_options,
            initial_damping=0.0,
            damping_mode="identity",
            active_dims=active_dims,
            use_alpha_line_search=False,
            # initial_alpha=100.0,
            initial_alpha=25.0,
            alpha_shrink=0.75,
            use_momentum=True,
            # momentum_beta=0.9,
            momentum_beta=0.9,
            max_cost_increase_ratio=1e-1, # TODO change this
            max_iters_without_best_improvement=2,
            radar_azimuths=radar_azimuths,
            max_iters=20,
            verbose=True,
        )

        print_localization_error_report(
            T_init=T_init,
            T_hat=result.state,
            T_gt=T_gt,
            history=result.history,
        )
        result_row = build_localization_result_row(
            sequence_id=seq.ID,
            frame_id=radar_frame.frame,
            T_init=T_init,
            T_hat=result.state,
            T_gt=T_gt,
            history=result.history,
        )
        append_localization_result(results_csv_path, result_row)
        print(f"Saved localization result: {results_csv_path}")

        radar_frame.unload_data()
        print("radar frame unloaded!")
        radar_frame_idx += 100


def main():
    parser = argparse.ArgumentParser(description="Run the fast radar-lidar localization pipeline.")
    parser.add_argument(
        "--experiment-name",
        required=True,
        help="Name used for the result CSV, for example yaw_2deg.",
    )
    output_mode = parser.add_mutually_exclusive_group()
    output_mode.add_argument(
        "--append",
        action="store_true",
        help="Append results to an existing experiment CSV.",
    )
    output_mode.add_argument(
        "--overwrite",
        action="store_true",
        help="Replace an existing experiment CSV.",
    )
    parser.add_argument(
        "--geometry-backend",
        choices=["numpy_spherical", "torch_frustum", "torch_mesh"],
        default="torch_frustum",
        help="Geometry projection backend.",
    )
    parser.add_argument(
        "--geometry-batch-size",
        type=int,
        default=16,
        help="Number of azimuths processed together by the Torch frustum backend.",
    )
    parser.add_argument(
        "--mesh-root",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "postprocessing" / "submap_meshes",
        help="Root directory containing saved per-submap meshes.",
    )
    args = parser.parse_args()
    if args.geometry_batch_size < 1:
        raise ValueError("--geometry-batch-size must be at least 1.")

    experiment_name = Path(args.experiment_name).name
    if experiment_name != args.experiment_name or experiment_name in {"", ".", ".."}:
        raise ValueError("--experiment-name must be a simple filename-safe name.")

    boreas_vtr_wrapper_dir = os.getenv("VTRROOT")
    boreas_data = os.getenv("VTRRDATA")
    if boreas_vtr_wrapper_dir is None:
        raise RuntimeError("VTRROOT must be set.")
    if boreas_data is None:
        raise RuntimeError("VTRRDATA must be set.")

    lidar_results_dir = os.path.join(boreas_vtr_wrapper_dir, "results/lidar")
    bd = BoreasDataset(boreas_data)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    weights_path = os.path.join(
        boreas_vtr_wrapper_dir,
        "model_dev/model_weights/6_deg_attentional_skip_bigger/best.pth",
    )
    model = load_radar_translator_model(weights_path, device)

    radar_start_frame = 65 # 365 # 2065
    radar_end_frame = None
    fov_deg = 6.0
    res_deg = 0.1
    geometry_backend = args.geometry_backend
    geometry_batch_size = args.geometry_batch_size

    patch_config = build_patch_config(
        fov_deg=fov_deg,
        res_deg=res_deg,
        min_range=0.0,
        max_uv_edge_length=None,
        max_depth_jump=2.0,
    )
    patch_config["fov_deg"] = fov_deg

    for seq in bd.sequences:
        results_csv_path = (
            Path(boreas_vtr_wrapper_dir)
            / "localization_fast"
            / "results"
            / seq.ID
            / f"{experiment_name}.csv"
        )
        initialize_results_csv(
            results_csv_path,
            append=args.append,
            overwrite=args.overwrite,
        )
        run_sequence(
            seq=seq,
            lidar_results_dir=lidar_results_dir,
            radar_start_frame=radar_start_frame,
            radar_end_frame=radar_end_frame,
            patch_config=patch_config,
            model=model,
            device=device,
            results_csv_path=results_csv_path,
            geometry_backend=geometry_backend,
            geometry_batch_size=geometry_batch_size,
            mesh_root=args.mesh_root,
        )
        break


if __name__ == "__main__":
    main()
