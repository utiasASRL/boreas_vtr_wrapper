import argparse
import csv
import os
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np
import torch
from pyboreas import BoreasDataset
from pyboreas.utils.odometry import interpolate_poses
from pyboreas.utils.utils import get_inverse_tf
from pylgmath import Transformation
from scipy.spatial.transform import Rotation as R

from localization_dfo.io_utils import (
    build_T_lidar_robot,
    build_patch_config,
    cen_filter_2d,
    correct_offsets,
    get_path_vertices_with_submaps,
    load_submap_mesh_to_enu,
)
from localization_dfo.optix_backend import OptixDepthBackend, compute_optix_cost
from localization_dfo.optimizer import ObjectiveLogger, extract_best_from_result, run_imfil_direct
from localization_dfo.radar_translator_cnn import RadarTranslatorCNN
from localization_dfo.transforms import left_se3_retract, make_delta_T


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


T_180_YAW = np.array(
    [
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=float,
)


def vtr_pose_distance(T_query_submap, angle_weight=7.0):
    se3 = np.asarray(Transformation(T_ba=T_query_submap).vec()).reshape(-1)
    return float(np.linalg.norm(se3[:3]) + angle_weight * np.linalg.norm(se3[3:]))


def submap_distance(T_query_enu, T_enu_submap, angle_weight=7.0):
    T_query_submap = T_query_enu @ T_enu_submap
    return min(
        vtr_pose_distance(T_query_submap, angle_weight),
        vtr_pose_distance(T_query_submap @ T_180_YAW, angle_weight),
    )


def transform_matrix(transform):
    return np.asarray(transform.matrix(), dtype=float)


def build_T_radar_robot(seq, T_lidar_robot):
    return np.asarray(seq.calib.T_radar_lidar, dtype=float) @ transform_matrix(T_lidar_robot)


def build_path_candidates(map_seq, path_submap_pairs, T_lidar_robot):
    lidar_frames_by_stamp = {int(frame.frame): frame for frame in map_seq.lidar_frames}
    candidates = []
    for path_vertex, submap_vertex in path_submap_pairs:
        path_stamp_us = path_vertex.stamp // 1000
        path_lidar_frame = lidar_frames_by_stamp.get(path_stamp_us)
        if path_lidar_frame is None:
            raise ValueError(f"No lidar frame in {map_seq.ID} for path vertex stamp {path_stamp_us}.")

        submap_stamp_us = submap_vertex.stamp // 1000
        submap_lidar_frame = lidar_frames_by_stamp.get(submap_stamp_us)
        if submap_lidar_frame is None:
            raise ValueError(f"No lidar frame in {map_seq.ID} for submap stamp {submap_stamp_us}.")

        T_enu_robot = np.asarray(path_lidar_frame.pose, dtype=float) @ transform_matrix(T_lidar_robot)
        candidates.append((submap_vertex, submap_lidar_frame, T_enu_robot))
    if not candidates:
        raise ValueError(f"No path candidates found for {map_seq.ID}.")
    return candidates


def nearest_submap_idx(T_query_enu, candidates):
    return min(
        range(len(candidates)),
        key=lambda idx: submap_distance(T_query_enu, candidates[idx][2]),
    )


def advance_submap_idx(T_query_enu, candidates, submap_idx):
    while submap_idx + 1 < len(candidates):
        curr_dist = submap_distance(T_query_enu, candidates[submap_idx][2])
        next_dist = submap_distance(T_query_enu, candidates[submap_idx + 1][2])
        if next_dist >= curr_dist:
            break
        submap_idx += 1
    return submap_idx


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


def initialize_result_files(csv_path, pose_path, append=False, overwrite=False):
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    if overwrite:
        csv_path.unlink(missing_ok=True)
        pose_path.unlink(missing_ok=True)
    elif not append and (csv_path.exists() or pose_path.exists()):
        raise FileExistsError(
            f"Experiment results already exist in {csv_path.parent}. "
            "Use --append or --overwrite explicitly."
        )
    elif append and csv_path.exists() != pose_path.exists():
        raise FileNotFoundError(
            "Cannot append because only one of the CSV and pyboreas result files exists."
        )

    if not csv_path.exists():
        with csv_path.open("w", newline="") as f:
            csv.DictWriter(f, fieldnames=RESULT_FIELDNAMES).writeheader()
    pose_path.touch(exist_ok=True)


def append_localization_result(csv_path, result_row):
    with csv_path.open("a", newline="") as f:
        csv.DictWriter(f, fieldnames=RESULT_FIELDNAMES).writerow(result_row)


def build_pyboreas_result_row(radar_frame, reference_lidar_frame, T_radar_enu):
    T_lidar_radar = (
        np.linalg.inv(reference_lidar_frame.pose)
        @ np.linalg.inv(T_radar_enu)
    )
    return [
        radar_frame.timestamp_micro,
        reference_lidar_frame.timestamp_micro,
        *T_lidar_radar[:3, :].reshape(-1),
    ]


def load_pyboreas_results(pose_path):
    rows = {}
    with pose_path.open() as f:
        for line in f:
            values = line.split()
            if not values:
                continue
            if len(values) != 14:
                raise ValueError(
                    f"Expected 14 values per pyboreas row in {pose_path}, got {len(values)}."
                )
            timestamp = int(values[0])
            if timestamp in rows:
                raise ValueError(f"Duplicate radar timestamp in {pose_path}: {timestamp}")
            rows[timestamp] = values
    return rows


def write_pyboreas_results(pose_path, rows):
    with pose_path.open("w", newline="") as f:
        writer = csv.writer(f, delimiter=" ")
        writer.writerows(rows[timestamp] for timestamp in sorted(rows))


def append_pyboreas_result(pose_path, row):
    with pose_path.open("a", newline="") as f:
        csv.writer(f, delimiter=" ").writerow(row)


def save_cartesian_radar_image(radar_frame, polar, output_path):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    radar_frame.polar = polar
    cart = radar_frame.polar_to_cart(
        cart_resolution=0.2384,
        cart_pixel_width=1000,
        in_place=False,
    )
    image = (np.clip(cart, 0.0, 1.0) * 255.0).astype(np.uint8)
    if not cv2.imwrite(str(output_path), image):
        raise IOError(f"Could not save radar image: {output_path}")


def predict_radar_at_pose(T, model, optix_backend):
    depth = optix_backend.trace(T)
    with torch.no_grad():
        return torch.sigmoid(model(depth.unsqueeze(1))).cpu().numpy()


def print_dfo_timing_report(optimization_time_s, logger, cost_call_timings):
    cost_calls = len(logger.history_f)
    if len(cost_call_timings) != cost_calls:
        raise RuntimeError(
            f"Timing count {len(cost_call_timings)} does not match cost call count {cost_calls}."
        )

    print("DFO timing")
    print(f"  optimization wall time [s]: {optimization_time_s:.6f}")
    print(f"  cost function calls: {cost_calls}")
    if not cost_calls:
        return

    depth_time_s = sum(timing["depth_patch_generation_time_s"] for timing in cost_call_timings)
    model_time_s = sum(timing["model_inference_time_s"] for timing in cost_call_timings)
    print(f"  mean wall time / cost call [s]: {optimization_time_s / cost_calls:.6f}")
    print(
        f"  depth patch generation [s]: {depth_time_s:.6f} total, "
        f"{depth_time_s / cost_calls:.6f}/call"
    )
    print(
        f"  model inference [s]: {model_time_s:.6f} total, "
        f"{model_time_s / cost_calls:.6f}/call"
    )


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


def run_sequence(
    map_seq,
    loc_seq,
    lidar_results_dir,
    radar_start_frame,
    radar_end_frame,
    patch_config,
    model,
    device,
    results_csv_path,
    results_pose_path,
    mesh_root,
    random_seed,
    imfil_budget,
    save_predictions,
    save_labels,
):
    print(f"Map SequenceID: {map_seq.ID}")
    print(f"Localization SequenceID: {loc_seq.ID}")
    print(f"Number of Radar Frames: {len(loc_seq.radar_frames)}")

    end_frame = len(loc_seq.radar_frames) - 2 if radar_end_frame is None else min(radar_end_frame, len(loc_seq.radar_frames) - 2)
    radar_start_frame = max(radar_start_frame, 1)

    graph_dir = os.path.join(lidar_results_dir, map_seq.ID, map_seq.ID, "graph")
    _, path_submap_pairs = get_path_vertices_with_submaps(graph_dir=graph_dir)
    T_lidar_robot = build_T_lidar_robot(map_seq)
    submap_candidates = build_path_candidates(map_seq, path_submap_pairs, T_lidar_robot)
    T_robot_radar = np.linalg.inv(build_T_radar_robot(loc_seq, build_T_lidar_robot(loc_seq)))

    submap_idx = None
    radar_frame_idx = radar_start_frame
    loaded_mesh_submap_stamp_us = None
    mesh_vertices_gpu = None
    mesh_triangles_gpu = None
    rng = np.random.default_rng(random_seed)

    optix_backend = OptixDepthBackend(patch_config, device)
    pyboreas_rows = load_pyboreas_results(results_pose_path)

    first_radar_frame = loc_seq.radar_frames[0]
    first_timestamp = first_radar_frame.timestamp_micro
    if first_timestamp not in pyboreas_rows:
        T_first_gt = np.linalg.inv(first_radar_frame.pose)
        first_submap_idx = nearest_submap_idx(
            T_robot_radar @ T_first_gt,
            submap_candidates,
        )
        first_lidar_frame = submap_candidates[first_submap_idx][1]
        first_row = build_pyboreas_result_row(
            first_radar_frame,
            first_lidar_frame,
            T_first_gt,
        )
        pyboreas_rows[first_timestamp] = [str(value) for value in first_row]
        append_pyboreas_result(results_pose_path, first_row)

    while radar_frame_idx < end_frame + 1:
        radar_frame = loc_seq.get_radar(radar_frame_idx)
        if radar_frame.timestamp_micro in pyboreas_rows:
            raise ValueError(
                f"Radar timestamp already exists in {results_pose_path}: "
                f"{radar_frame.timestamp_micro}"
            )
        poses = [
            get_inverse_tf(rad_frame.pose)
            for rad_frame in loc_seq.radar_frames[radar_frame_idx - 1:radar_frame_idx + 2]
        ]
        times = [
            rad_frame.timestamp_micro
            for rad_frame in loc_seq.radar_frames[radar_frame_idx - 1:radar_frame_idx + 2]
        ]
        azimuth_poses = interpolate_poses(poses, times, radar_frame.timestamps.flatten().tolist())
        radar_azimuths = radar_frame.azimuths.flatten()

        T_enu_radar = radar_frame.pose
        odom_transforms = np.array([T_enu_radar @ T_i for T_i in azimuth_poses])

        T_offset = make_delta_T(
            translation=rng.uniform(-0.2, 0.2, size=3),
            rpy_deg=rng.uniform(-2.0, 2.0, size=3),
        )
        T_gt = np.linalg.inv(T_enu_radar)
        T_init = T_offset @ T_gt

        # submap selection with T_gt
        # T_robot_enu = T_robot_radar @ T_gt

        # submap selection with T_init
        T_robot_enu = T_robot_radar @ T_init

        submap_idx = nearest_submap_idx(T_robot_enu, submap_candidates)
        
        # if submap_idx is None:
        #     submap_idx = nearest_submap_idx(T_robot_enu, submap_candidates)
        # else:
        #     submap_idx = advance_submap_idx(T_robot_enu, submap_candidates, submap_idx)
        curr_submap, lidar_frame, _ = submap_candidates[submap_idx]

        submap_stamp_us = curr_submap.stamp // 1000
        if loaded_mesh_submap_stamp_us != submap_stamp_us:
            mesh_vertices_gpu, mesh_triangles_gpu, _ = load_submap_mesh_to_enu( # vertices in enu frame
                mesh_root=mesh_root,
                sequence_id=map_seq.ID,
                submap=curr_submap,
                T_lidar_robot=T_lidar_robot,
                lidar_pose=lidar_frame.pose,
                device=device,
            )
            optix_backend.set_mesh(mesh_vertices_gpu, mesh_triangles_gpu)
            loaded_mesh_submap_stamp_us = submap_stamp_us

        shifted_polar = correct_offsets(radar_frame, radar_frame_idx, loc_seq)
        filtered_polar = cen_filter_2d(shifted_polar, sigma_gauss=15.0, z_q=2.5, noise_scale=0.5)

        radar_polar_cropped = filtered_polar[:, :2736] / 0.5613
        if len(radar_polar_cropped) != len(odom_transforms):
            raise ValueError(
                f"Radar rows and odometry pose counts differ: "
                f"{len(radar_polar_cropped)} != {len(odom_transforms)}."
            )
        radar_polar_cropped_gpu = torch.as_tensor(
            radar_polar_cropped,
            device=device,
            dtype=torch.float32,
        ).contiguous()
        optix_backend.set_scan(odom_transforms, radar_azimuths)

        cost_call_timings = []

        def cost_fn(eps):
            T_current = left_se3_retract(T=T_init, delta=eps)
            cost, timing = compute_optix_cost(
                optix_backend,
                T_current,
                radar_polar_cropped_gpu,
                model,
            )
            cost_call_timings.append(timing)
            return cost

        logger = ObjectiveLogger(cost_fn)
        eps0 = np.zeros(6)
        bounds = np.array(
            [
                [-0.3, 0.3],
                [-0.3, 0.3],
                [-0.3, 0.3],
                [-3.0, 3.0],
                [-3.0, 3.0],
                [-3.0, 3.0],
            ],
            dtype=float,
        )
        bounds[3:] = np.deg2rad(bounds[3:])

        t_opt = perf_counter()
        result, _ = run_imfil_direct("imfil", logger, eps0, bounds, imfil_budget)
        optimization_time_s = perf_counter() - t_opt
        eps_best, _ = extract_best_from_result(result, logger)
        T_hat = left_se3_retract(T=T_init, delta=eps_best)

        image_root = results_csv_path.parent
        if save_labels:
            save_cartesian_radar_image(
                radar_frame,
                filtered_polar / 0.5613,
                image_root / "labels" / f"{radar_frame.frame}.png",
            )
        if save_predictions:
            predictions = predict_radar_at_pose(
                T=T_hat,
                model=model,
                optix_backend=optix_backend,
            )
            if predictions.shape != radar_polar_cropped.shape:
                raise RuntimeError(
                    f"Prediction shape {predictions.shape} does not match "
                    f"cropped radar shape {radar_polar_cropped.shape}."
                )
            padded_predictions = np.zeros_like(filtered_polar, dtype=np.float32)
            padded_predictions[:, :predictions.shape[1]] = predictions
            save_cartesian_radar_image(
                radar_frame,
                padded_predictions,
                image_root / "predictions" / f"{radar_frame.frame}.png",
            )

        print_dfo_timing_report(optimization_time_s, logger, cost_call_timings)
        print_localization_error_report(T_init, T_hat, T_gt)
        append_localization_result(
            results_csv_path,
            build_localization_result_row(loc_seq.ID, radar_frame.frame, T_init, T_hat, T_gt, logger.history_f),
        )
        pose_row = build_pyboreas_result_row(radar_frame, lidar_frame, T_hat)
        pyboreas_rows[radar_frame.timestamp_micro] = [str(value) for value in pose_row]
        append_pyboreas_result(results_pose_path, pose_row)
        print(f"Saved localization result: {results_csv_path}")

        radar_frame.unload_data()
        print("radar frame unloaded!")
        radar_frame_idx += 1

    penultimate_timestamp = loc_seq.radar_frames[-2].timestamp_micro
    last_radar_frame = loc_seq.radar_frames[-1]
    if (
        penultimate_timestamp in pyboreas_rows
        and last_radar_frame.timestamp_micro not in pyboreas_rows
    ):
        last_row = pyboreas_rows[penultimate_timestamp].copy()
        last_row[0] = str(last_radar_frame.timestamp_micro)
        pyboreas_rows[last_radar_frame.timestamp_micro] = last_row
        append_pyboreas_result(results_pose_path, last_row)
    write_pyboreas_results(results_pose_path, pyboreas_rows)
    print(f"Saved pyboreas results: {results_pose_path}")


def main():
    parser = argparse.ArgumentParser(description="Run DFO radar-lidar localization.")
    parser.add_argument("--experiment-name", required=True)
    parser.add_argument("--map-sequence", required=True)
    parser.add_argument("--loc-sequence", required=True)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--radar-start-frame", type=int, default=65)
    parser.add_argument("--radar-end-frame", type=int, default=None)
    parser.add_argument("--random-seed", type=int, default=0)
    parser.add_argument("--imfil-budget", type=int, default=120)
    parser.add_argument("--save-predictions", action="store_true")
    parser.add_argument("--save-labels", action="store_true")
    output_mode = parser.add_mutually_exclusive_group()
    output_mode.add_argument("--append", action="store_true")
    output_mode.add_argument("--overwrite", action="store_true")
    parser.add_argument(
        "--mesh-root",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "postprocessing" / "submap_meshes",
    )
    args = parser.parse_args()

    if args.imfil_budget < 1:
        raise ValueError("--imfil-budget must be positive.")

    experiment_name = Path(args.experiment_name).name
    if experiment_name != args.experiment_name or experiment_name in {"", ".", ".."}:
        raise ValueError("--experiment-name must be a simple filename-safe name.")

    boreas_vtr_wrapper_dir = os.getenv("VTRROOT")
    boreas_data = os.getenv("VTRRDATA")
    vtr_results = Path(
        os.getenv("VTRRESULT", Path(boreas_vtr_wrapper_dir or ".") / "results")
    )
    if boreas_vtr_wrapper_dir is None:
        raise RuntimeError("VTRROOT must be set.")
    if boreas_data is None:
        raise RuntimeError("VTRRDATA must be set.")

    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA device requested but CUDA is unavailable.")
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cuda.matmul.allow_tf32 = False
    lidar_results_dir = vtr_results / "lidar"
    weights_path = os.path.join(
        boreas_vtr_wrapper_dir,
        "model_dev/route_weights/1-suburb/best_total.pth",
    )
    model = load_radar_translator_model(weights_path, device)
    dataset = BoreasDataset(
        boreas_data,
        [[sequence_id] for sequence_id in dict.fromkeys((args.map_sequence, args.loc_sequence))],
    )

    patch_config = build_patch_config(
        fov_deg=6.0,
        res_deg=0.1,
        min_range=0.0,
        max_uv_edge_length=None,
        max_depth_jump=2.0,
    )
    patch_config["fov_deg"] = 6.0

    map_seq = dataset.get_seq_from_ID(args.map_sequence)
    loc_seq = dataset.get_seq_from_ID(args.loc_sequence)
    experiment_dir = (
        Path(boreas_vtr_wrapper_dir)
        / "localization_dfo"
        / "results"
        / loc_seq.ID
        / experiment_name
    )
    results_csv_path = experiment_dir / f"{experiment_name}.csv"
    results_pose_path = experiment_dir / f"{loc_seq.ID}.txt"
    initialize_result_files(
        results_csv_path,
        results_pose_path,
        append=args.append,
        overwrite=args.overwrite,
    )
    run_sequence(
        map_seq=map_seq,
        loc_seq=loc_seq,
        lidar_results_dir=lidar_results_dir,
        radar_start_frame=args.radar_start_frame,
        radar_end_frame=args.radar_end_frame,
        patch_config=patch_config,
        model=model,
        device=device,
        results_csv_path=results_csv_path,
        results_pose_path=results_pose_path,
        mesh_root=args.mesh_root,
        random_seed=args.random_seed,
        imfil_budget=args.imfil_budget,
        save_predictions=args.save_predictions,
        save_labels=args.save_labels,
    )


if __name__ == "__main__":
    main()
