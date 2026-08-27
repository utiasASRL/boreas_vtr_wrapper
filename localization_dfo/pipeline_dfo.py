import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
import math
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
from localization_dfo.transforms import left_se3_retract


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
    "optimization_mode",
    "near_depth_patch_count",
    "planar_frames_remaining",
    "optimizer_budget",
    "candidate_x_m",
    "candidate_y_m",
    "candidate_z_m",
    "candidate_roll_deg",
    "candidate_pitch_deg",
    "candidate_yaw_deg",
    "candidate_cost",
    "candidate_radar_cost",
    "candidate_prior_penalty",
    "optimizer_eps_x_m",
    "optimizer_eps_y_m",
    "optimizer_eps_z_m",
    "optimizer_eps_rx_deg",
    "optimizer_eps_ry_deg",
    "optimizer_eps_rz_deg",
    "observed_total_evidence",
    "initial_prediction_active_azimuth_count",
    "initial_prediction_active_fraction",
    "candidate_prediction_active_azimuth_count",
    "candidate_prediction_active_fraction",
    "candidate_prediction_active_ratio",
    "candidate_prediction_active_drop",
    "candidate_missing_observed_evidence",
    "candidate_missing_observed_evidence_fraction",
    "pose_gating_enabled",
    "localization_state",
    "optimizer_rejected",
    "rejection_reasons",
    "consecutive_rejections",
    "optimization_time_s",
    "model_inference_time_s",
    "depth_patch_generation_time_s",
    "peak_cuda_memory_mb",
]

TRANSLATION_JUMP_M = 0.20
ROTATION_JUMP_DEG = 0.50
MISSING_OBSERVED_EVIDENCE_FRACTION = 0.60
SUBMAP_SEARCH_RADIUS = 5
SUBMAP_SEARCH_MAX_RADIUS = 20
COARSE_TRANSLATION_TARGET_M = 1.0
COARSE_ROTATION_TARGET_DEG = 3.0
FINE_TRANSLATION_TARGET_M = 0.01
FINE_ROTATION_TARGET_DEG = 0.05
NEAR_DEPTH_M = 3.0
NEAR_DEPTH_PATCH_COUNT = 3
PLANAR_HOLD_FRAMES = 10
PLANAR_DOF_INDICES = (0, 1, 5)


def load_radar_translator_model(weights_path, device):
    model = RadarTranslatorCNN().to(device)
    checkpoint = torch.load(weights_path, map_location=device)
    if not isinstance(checkpoint, dict) or "radar_normalization_scale" not in checkpoint:
        raise ValueError(
            f"Checkpoint {weights_path} has no radar_normalization_scale metadata. "
            "Convert the legacy checkpoint before evaluation."
        )
    normalization_scale = float(checkpoint["radar_normalization_scale"])
    if not np.isfinite(normalization_scale) or normalization_scale <= 0:
        raise ValueError(f"Invalid radar_normalization_scale: {normalization_scale}")
    if isinstance(checkpoint, dict) and "model_state_dict" in checkpoint:
        state_dict = checkpoint["model_state_dict"]
    elif isinstance(checkpoint, dict) and "state_dict" in checkpoint:
        state_dict = checkpoint["state_dict"]
    else:
        state_dict = checkpoint
    if isinstance(state_dict, dict) and any(key.startswith("module.") for key in state_dict):
        state_dict = {key.removeprefix("module."): value for key, value in state_dict.items()}
    model.load_state_dict(state_dict)
    model.radar_normalization_scale = normalization_scale
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


def nearest_submap_idx(T_query_enu, candidates, center_idx=None, radius=SUBMAP_SEARCH_RADIUS):
    if center_idx is None:
        return min(
            range(len(candidates)),
            key=lambda idx: submap_distance(T_query_enu, candidates[idx][2]),
        )

    max_radius = min(SUBMAP_SEARCH_MAX_RADIUS, len(candidates) // 2)
    radius = min(radius, max_radius)
    while True:
        offsets = {
            (center_idx + offset) % len(candidates): offset
            for offset in range(-radius, radius + 1)
        }
        best_idx, best_offset = min(
            offsets.items(),
            key=lambda item: submap_distance(T_query_enu, candidates[item[0]][2]),
        )
        if abs(best_offset) < radius or radius >= max_radius:
            return best_idx
        radius = min(radius * 2, max_radius)


def optimizer_bounds(localization_state):
    # bounds = np.array(
    #     [[-0.3, 0.3]] * 3 + [[-3.0, 3.0]] * 3,
    #     dtype=float,
    # )
    # if localization_state == "recovery":
    #     bounds[:1] = [-1.0, 1.0]
    # bounds[3:] = np.deg2rad(bounds[3:])


    # wide
    bounds = np.array(
        [
            [-1.0, 1.0],
            [-1.0, 1.0],
            [-1.0, 1.0],
            [-3.0, 3.0],
            [-3.0, 3.0],
            [-3.0, 3.0],
        ],
        dtype=float,
    )

    # OG3 bounds, suburb bounds
    # bounds = np.array(
    #         [
    #             [-0.3, 0.3],
    #             [-0.3, 0.3],
    #             [-0.3, 0.3],
    #             [-3.0, 3.0],
    #             [-3.0, 3.0],
    #             [-3.0, 3.0],
    #         ],
    #         dtype=float,
    #     )
    bounds[3:] = np.deg2rad(bounds[3:])
    return bounds


def calculate_imfil_scales(bounds):
    spans = bounds[:, 1] - bounds[:, 0]
    coarse_ratio = max(
        spans[:3].max() / COARSE_TRANSLATION_TARGET_M,
        spans[3:].max() / np.deg2rad(COARSE_ROTATION_TARGET_DEG),
    )
    fine_ratio = max(
        spans[:3].max() / FINE_TRANSLATION_TARGET_M,
        spans[3:].max() / np.deg2rad(FINE_ROTATION_TARGET_DEG),
    )
    return max(0, math.ceil(math.log2(coarse_ratio))), max(
        0, math.ceil(math.log2(fine_ratio))
    )


def count_near_depth_patches(depth):
    valid = torch.isfinite(depth) & (depth > 0)
    patch_min = depth.masked_fill(~valid, torch.inf).amin(dim=(-2, -1))
    return int((patch_min < NEAR_DEPTH_M).sum().item())


def optimizer_dofs_and_budget(near_depth_patch_count, budget, frames_remaining=0):
    triggered = near_depth_patch_count >= NEAR_DEPTH_PATCH_COUNT
    if triggered or frames_remaining:
        next_remaining = PLANAR_HOLD_FRAMES if triggered else frames_remaining - 1
        return PLANAR_DOF_INDICES, max(1, budget // 2), next_remaining
    return range(6), budget, 0


def expand_optimizer_delta(active_eps, active_dofs):
    eps = np.zeros(6)
    eps[list(active_dofs)] = active_eps
    return eps


def pose_gate_diagnostics(observed, initial_prediction, candidate_prediction, eps_best):
    if observed.shape != initial_prediction.shape or observed.shape != candidate_prediction.shape:
        raise ValueError("Observed, initial, and candidate radar arrays must have the same shape.")

    initial_active = np.count_nonzero(initial_prediction > 0.05, axis=1) >= 3
    candidate_active = np.count_nonzero(candidate_prediction > 0.05, axis=1) >= 3
    azimuth_count = len(candidate_active)
    initial_count = int(initial_active.sum())
    candidate_count = int(candidate_active.sum())
    observed_evidence = observed.sum(axis=1)
    total_evidence = float(observed_evidence.sum())
    missing_evidence = float(observed_evidence[~candidate_active].sum())
    missing_evidence_fraction = missing_evidence / total_evidence if total_evidence else 0.0
    initial_fraction = initial_count / azimuth_count
    candidate_fraction = candidate_count / azimuth_count

    prediction_reasons = []
    if (
        candidate_fraction < 0.75 * initial_fraction
        and initial_fraction - candidate_fraction > 0.20
    ):
        prediction_reasons.append("candidate_prediction_coverage_collapse")
    if missing_evidence_fraction > MISSING_OBSERVED_EVIDENCE_FRACTION:
        prediction_reasons.append("candidate_observed_azimuth_mismatch")

    jump_reasons = []
    for axis, value in zip("xyz", np.abs(eps_best[:3])):
        if value > TRANSLATION_JUMP_M:
            jump_reasons.append(f"translation_jump_{axis}")
    for axis, value in zip(("rx", "ry", "rz"), np.abs(np.rad2deg(eps_best[3:]))):
        if value > ROTATION_JUMP_DEG:
            jump_reasons.append(f"rotation_jump_{axis}")

    return {
        "observed_total_evidence": total_evidence,
        "initial_prediction_active_azimuth_count": initial_count,
        "initial_prediction_active_fraction": initial_fraction,
        "candidate_prediction_active_azimuth_count": candidate_count,
        "candidate_prediction_active_fraction": candidate_fraction,
        "candidate_prediction_active_ratio": (
            candidate_fraction / initial_fraction if initial_fraction else float("nan")
        ),
        "candidate_prediction_active_drop": initial_fraction - candidate_fraction,
        "candidate_missing_observed_evidence": missing_evidence,
        "candidate_missing_observed_evidence_fraction": missing_evidence_fraction,
    }, jump_reasons, prediction_reasons


def apply_pose_gate(
    state,
    consecutive_rejections,
    healthy_recovery_accepts,
    jump_reasons,
    prediction_reasons,
):
    reasons = prediction_reasons + (jump_reasons if state == "tracking" else [])
    rejected = bool(reasons)
    next_state = state
    if rejected:
        consecutive_rejections += 1
        healthy_recovery_accepts = 0
        if state == "tracking" and consecutive_rejections >= 3:
            next_state = "recovery"
    else:
        consecutive_rejections = 0
        if state == "recovery":
            healthy_recovery_accepts = healthy_recovery_accepts + 1 if not jump_reasons else 0
            if healthy_recovery_accepts >= 2:
                next_state = "tracking"
                healthy_recovery_accepts = 0
    return rejected, reasons, next_state, consecutive_rejections, healthy_recovery_accepts


def build_localization_result_row(
    sequence_id,
    frame_id,
    T_init,
    T_hat,
    T_candidate,
    T_gt,
    history,
    eps_best,
    candidate_cost,
    gate_fields,
    benchmark_fields,
):
    init_xyz, init_rpy_deg = pose_error_xyz_rpy(T_init, T_gt)
    final_xyz, final_rpy_deg = pose_error_xyz_rpy(T_hat, T_gt)
    candidate_xyz, candidate_rpy_deg = pose_error_xyz_rpy(T_candidate, T_gt)
    has_history = len(history) > 0

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
        "final_cost": float(history[0]) if gate_fields["optimizer_rejected"] else candidate_cost,
        "iterations": len(history),
        "candidate_x_m": float(candidate_xyz[0]),
        "candidate_y_m": float(candidate_xyz[1]),
        "candidate_z_m": float(candidate_xyz[2]),
        "candidate_roll_deg": float(candidate_rpy_deg[0]),
        "candidate_pitch_deg": float(candidate_rpy_deg[1]),
        "candidate_yaw_deg": float(candidate_rpy_deg[2]),
        "candidate_cost": candidate_cost,
        "optimizer_eps_x_m": float(eps_best[0]),
        "optimizer_eps_y_m": float(eps_best[1]),
        "optimizer_eps_z_m": float(eps_best[2]),
        "optimizer_eps_rx_deg": float(np.rad2deg(eps_best[3])),
        "optimizer_eps_ry_deg": float(np.rad2deg(eps_best[4])),
        "optimizer_eps_rz_deg": float(np.rad2deg(eps_best[5])),
        **gate_fields,
        **benchmark_fields,
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

    if append and csv_path.exists():
        with csv_path.open(newline="") as f:
            header = next(csv.reader(f), None)
        if header != RESULT_FIELDNAMES:
            raise ValueError(
                f"Cannot append to {csv_path}: its columns do not match the current result schema."
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


def load_resume_state(csv_path, pose_rows, map_seq, loc_seq, radar_start_frame):
    last_result = None
    with csv_path.open(newline="") as f:
        for last_result in csv.DictReader(f):
            pass
    if last_result is None:
        return None, "tracking", 0, 0, 0

    previous_frame = loc_seq.radar_frames[radar_start_frame - 1]
    previous_timestamp = previous_frame.timestamp_micro
    if int(last_result["frame_id"]) != previous_timestamp or previous_timestamp not in pose_rows:
        raise ValueError(
            f"Resume frame {radar_start_frame} does not follow the last saved frame "
            f"{last_result['frame_id']}."
        )

    pose_row = pose_rows[previous_timestamp]
    lidar_timestamp = int(pose_row[1])
    lidar_frame = next(
        (frame for frame in map_seq.lidar_frames if frame.timestamp_micro == lidar_timestamp),
        None,
    )
    if lidar_frame is None:
        raise ValueError(f"Saved reference lidar frame not found: {lidar_timestamp}")
    T_lidar_radar = np.eye(4)
    T_lidar_radar[:3] = np.asarray(pose_row[2:], dtype=float).reshape(3, 4)
    previous_pose = np.linalg.inv(lidar_frame.pose @ T_lidar_radar)

    state = last_result["localization_state"]
    consecutive_rejections = int(last_result["consecutive_rejections"])
    rejected = last_result["optimizer_rejected"].lower() == "true"
    jump = any(
        abs(float(last_result[name])) > limit
        for name, limit in (
            ("optimizer_eps_x_m", TRANSLATION_JUMP_M),
            ("optimizer_eps_y_m", TRANSLATION_JUMP_M),
            ("optimizer_eps_z_m", TRANSLATION_JUMP_M),
            ("optimizer_eps_rx_deg", ROTATION_JUMP_DEG),
            ("optimizer_eps_ry_deg", ROTATION_JUMP_DEG),
            ("optimizer_eps_rz_deg", ROTATION_JUMP_DEG),
        )
    )
    if state == "tracking" and consecutive_rejections >= 3:
        state = "recovery"
    elif state == "recovery" and not rejected and not jump:
        state = "tracking"
    print(f"Resuming after radar frame {radar_start_frame - 1} in {state} state")
    return (
        previous_pose,
        state,
        consecutive_rejections,
        0,
        int(last_result["planar_frames_remaining"]),
    )


def load_dro_odometry(path, radar_frames):
    with np.load(path) as data:
        required = {
            "frame_timestamps_us",
            "reference_poses",
            "azimuth_timestamps_us",
            "frame_offsets",
            "odom_transforms",
            "frame_transforms",
            "frame_body_velocities",
            "velocity_start_us",
            "velocity_end_us",
            "odom_transform_convention",
        }
        if missing := required.difference(data.files):
            raise ValueError(f"Missing arrays in {path}: {sorted(missing)}")
        result = {name: data[name] for name in required}

    frame_times = result["frame_timestamps_us"]
    reference_poses = result["reference_poses"]
    azimuth_times = result["azimuth_timestamps_us"]
    offsets = result["frame_offsets"]
    transforms = result["odom_transforms"]
    frame_transforms = result["frame_transforms"]
    frame_body_velocities = result["frame_body_velocities"]
    velocity_start_us = result["velocity_start_us"]
    velocity_end_us = result["velocity_end_us"]
    if result["odom_transform_convention"].item() != "right":
        raise ValueError("DRO NPZ does not contain right-side pipeline odometry transforms.")
    expected_times = np.asarray(
        [frame.timestamp_micro for frame in radar_frames], dtype=np.int64
    )
    if not np.array_equal(frame_times, expected_times):
        raise ValueError("3DRO and Boreas radar frame timestamps differ.")
    if reference_poses.shape != (len(frame_times), 4, 4):
        raise ValueError(f"Invalid 3DRO reference pose shape: {reference_poses.shape}")
    if frame_transforms.shape != reference_poses.shape:
        raise ValueError(f"Invalid 3DRO frame transform shape: {frame_transforms.shape}")
    if frame_body_velocities.shape != (len(frame_times), 2):
        raise ValueError(f"Invalid 2DRO body velocity shape: {frame_body_velocities.shape}")
    if velocity_start_us.shape != frame_times.shape or velocity_end_us.shape != frame_times.shape:
        raise ValueError("Invalid 2DRO velocity validity timestamp shapes.")
    if offsets.shape != (len(frame_times) + 1,) or offsets[0] != 0:
        raise ValueError(f"Invalid 3DRO frame offsets: {offsets.shape}")
    if offsets[-1] != len(azimuth_times) or transforms.shape != (len(azimuth_times), 4, 4):
        raise ValueError("3DRO azimuth array lengths differ.")
    expected_start_us = np.asarray(
        [np.min(azimuth_times[start:end]) for start, end in zip(offsets[:-1], offsets[1:])]
    )
    expected_end_us = np.asarray(
        [np.max(azimuth_times[start:end]) for start, end in zip(offsets[:-1], offsets[1:])]
    )
    if not np.array_equal(velocity_start_us, expected_start_us) or not np.array_equal(
        velocity_end_us, expected_end_us
    ):
        raise ValueError("2DRO velocity and radar scan validity timestamps differ.")
    if not np.allclose(frame_transforms[0], np.eye(4)) or not np.allclose(
        frame_transforms[1:] @ reference_poses[:-1], reference_poses[1:]
    ):
        raise ValueError("3DRO inter-frame transforms do not compose with reference poses.")
    if (
        np.any(np.diff(offsets) <= 0)
        or not np.isfinite(reference_poses).all()
        or not np.isfinite(transforms).all()
        or not np.isfinite(frame_transforms).all()
        or not np.isfinite(frame_body_velocities).all()
    ):
        raise ValueError("3DRO odometry contains invalid offsets or poses.")
    return result


def get_gt_azimuth_odometry(loc_seq, frame_idx, radar_frame):
    poses = [
        get_inverse_tf(frame.pose)
        for frame in loc_seq.radar_frames[frame_idx - 1:frame_idx + 2]
    ]
    times = [
        frame.timestamp_micro
        for frame in loc_seq.radar_frames[frame_idx - 1:frame_idx + 2]
    ]
    azimuth_poses = interpolate_poses(
        poses, times, radar_frame.timestamps.flatten().tolist()
    )
    return np.asarray(azimuth_poses) @ radar_frame.pose


def print_dro_odometry_error(dro_transforms, reference_pose, gt_transforms):
    dro_left_transforms = reference_pose @ dro_transforms @ np.linalg.inv(reference_pose)
    errors = dro_left_transforms @ np.linalg.inv(gt_transforms)
    translation = np.linalg.norm(errors[:, :3, 3], axis=1)
    rotation_deg = np.rad2deg(R.from_matrix(errors[:, :3, :3]).magnitude())
    print(
        "3DRO azimuth error: "
        f"translation p50/p95/max [m] = {np.percentile(translation, [50, 95, 100])}, "
        f"rotation p50/p95/max [deg] = {np.percentile(rotation_deg, [50, 95, 100])}"
    )


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
    lambda_prior_tracking,
    lambda_prior_recovery,
    sigma_prior,
    results_csv_path,
    results_pose_path,
    mesh_root,
    imfil_budget,
    save_predictions,
    save_init,
    save_labels,
    dro_odometry_path,
    validate_dro_odometry,
    pose_gating,
    imfil_function_delta,
    imfil_stencil_delta,
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
    optix_backend = OptixDepthBackend(patch_config, device)
    pyboreas_rows = load_pyboreas_results(results_pose_path)
    dro_odometry = load_dro_odometry(dro_odometry_path, loc_seq.radar_frames)
    (
        previous_localized_pose,
        localization_state,
        consecutive_rejections,
        healthy_recovery_accepts,
        planar_frames_remaining,
    ) = load_resume_state(
        results_csv_path,
        pyboreas_rows,
        map_seq,
        loc_seq,
        radar_start_frame,
    )
    model_warmed_up = False
    steady_iteration_time_sum = 0.0

    imfil_options = {}
    for name, value in (
        ("function_delta", imfil_function_delta),
        ("stencil_delta", imfil_stencil_delta),
    ):
        if value is not None:
            imfil_options[name] = value
    total_frames = end_frame - radar_start_frame + 1

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

    def preprocess_radar_frame(frame_idx):
        start_time = perf_counter()
        frame = loc_seq.get_radar(frame_idx)
        azimuths = frame.azimuths.flatten()
        start, end = dro_odometry["frame_offsets"][frame_idx:frame_idx + 2]
        odom_times = dro_odometry["azimuth_timestamps_us"][start:end]
        if not np.array_equal(odom_times, frame.timestamps.flatten()):
            raise ValueError(f"3DRO azimuth timestamps differ for radar frame {frame.frame}.")
        transforms = dro_odometry["odom_transforms"][start:end]
        shifted = correct_offsets(
            frame,
            dro_odometry["frame_body_velocities"][frame_idx],
            loc_seq.calib.radar_offset,
        )
        filtered = cen_filter_2d(
            shifted,
            sigma_gauss=15.0,
            z_q=2.5,
            noise_scale=0.5,
            output_width=2736,
        )
        observed = filtered / model.radar_normalization_scale
        if len(observed) != len(transforms):
            raise ValueError(
                f"Radar rows and odometry pose counts differ: "
                f"{len(observed)} != {len(transforms)}."
            )
        return frame, azimuths, transforms, filtered, observed, perf_counter() - start_time

    preprocessing_executor = ThreadPoolExecutor(max_workers=1)
    preprocessing_future = preprocessing_executor.submit(
        preprocess_radar_frame, radar_start_frame
    )
    while radar_frame_idx < end_frame + 1:
        iteration_start = perf_counter()
        prefetch_wait_start = perf_counter()
        (
            radar_frame,
            radar_azimuths,
            odom_transforms,
            filtered_polar,
            radar_polar_cropped,
            radar_preprocessing_time_s,
        ) = preprocessing_future.result()
        prefetch_wait_time_s = perf_counter() - prefetch_wait_start
        if radar_frame_idx < end_frame:
            preprocessing_future = preprocessing_executor.submit(
                preprocess_radar_frame, radar_frame_idx + 1
            )

        frame_setup_start = perf_counter()
        if radar_frame.timestamp_micro in pyboreas_rows:
            raise ValueError(
                f"Radar timestamp already exists in {results_pose_path}: "
                f"{radar_frame.timestamp_micro}"
            )
        if validate_dro_odometry:
            print_dro_odometry_error(
                odom_transforms,
                dro_odometry["reference_poses"][radar_frame_idx],
                get_gt_azimuth_odometry(loc_seq, radar_frame_idx, radar_frame),
            )

        T_gt = np.linalg.inv(radar_frame.pose)
        if previous_localized_pose is None:
            T_init = T_gt.copy()
        else:
            frame_delta = dro_odometry["frame_transforms"][radar_frame_idx]
            T_init = frame_delta @ previous_localized_pose

        # submap selection with T_gt
        # T_robot_enu = T_robot_radar @ T_gt

        # submap selection with T_init
        T_robot_enu = T_robot_radar @ T_init

        submap_idx = nearest_submap_idx(T_robot_enu, submap_candidates, submap_idx)
        curr_submap, lidar_frame, _ = submap_candidates[submap_idx]
        frame_setup_time_s = perf_counter() - frame_setup_start

        mesh_update_start = perf_counter()
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
        mesh_update_time_s = perf_counter() - mesh_update_start

        radar_gpu_setup_start = perf_counter()
        radar_polar_cropped_gpu = torch.as_tensor(
            radar_polar_cropped,
            device=device,
            dtype=torch.float32,
        ).contiguous()
        optix_backend.set_scan(odom_transforms, radar_azimuths)
        radar_preprocessing_time_s += perf_counter() - radar_gpu_setup_start
        initial_prediction = None
        with torch.inference_mode():
            initial_depth = optix_backend.trace(T_init)
            near_depth_patch_count = count_near_depth_patches(initial_depth)
            if save_init or pose_gating:
                initial_prediction = torch.sigmoid(
                    model(initial_depth.unsqueeze(1))
                ).cpu().numpy()
            elif not model_warmed_up:
                model(initial_depth.unsqueeze(1))
        del initial_depth
        if not model_warmed_up:
            torch.cuda.synchronize(device)
            model_warmed_up = True
        active_dofs, optimizer_budget, planar_frames_remaining = optimizer_dofs_and_budget(
            near_depth_patch_count, imfil_budget, planar_frames_remaining
        )
        use_2d = active_dofs == PLANAR_DOF_INDICES
        cost_call_timings = []
        lambda_prior = (
            lambda_prior_recovery
            if localization_state == "recovery"
            else lambda_prior_tracking
        )

        def cost_fn(active_eps):
            eps = expand_optimizer_delta(active_eps, active_dofs)
            cost, timing = compute_optix_cost(
                optix_backend,
                left_se3_retract(T=T_init, delta=eps),
                radar_polar_cropped_gpu,
                model,
            )

            prior_cost = np.sum((eps / sigma_prior)**2)
            total_cost = cost + lambda_prior * prior_cost
            cost_call_timings.append(timing)
            return total_cost

        logger = ObjectiveLogger(cost_fn)
        eps0 = np.zeros(len(active_dofs))

        torch.cuda.reset_peak_memory_stats(device)
        t_opt = perf_counter()
        bounds = optimizer_bounds(localization_state)
        scale_start, scale_depth = calculate_imfil_scales(bounds)
        imfil_options.update(
            scale_start=scale_start, scale_depth=scale_depth
        )
        result, _ = run_imfil_direct(
            "imfil", logger, eps0, bounds[list(active_dofs)], optimizer_budget,
            imfil_options or None,
        )
        torch.cuda.synchronize(device)
        optimization_time_s = perf_counter() - t_opt
        peak_cuda_memory_mb = torch.cuda.max_memory_allocated(device) / 1024**2
        active_eps_best, candidate_cost = extract_best_from_result(result, logger)
        eps_best = expand_optimizer_delta(active_eps_best, active_dofs)
        candidate_prior_penalty = float(
            lambda_prior * np.sum((eps_best / sigma_prior) ** 2)
        )
        T_candidate = left_se3_retract(T=T_init, delta=eps_best)
        T_hat = T_candidate
        candidate_prediction = None
        accepted_prediction = None
        decision_state = localization_state
        gate_fields = {
            "observed_total_evidence": "",
            "initial_prediction_active_azimuth_count": "",
            "initial_prediction_active_fraction": "",
            "candidate_prediction_active_azimuth_count": "",
            "candidate_prediction_active_fraction": "",
            "candidate_prediction_active_ratio": "",
            "candidate_prediction_active_drop": "",
            "candidate_missing_observed_evidence": "",
            "candidate_missing_observed_evidence_fraction": "",
            "pose_gating_enabled": pose_gating,
            "localization_state": decision_state,
            "optimizer_rejected": False,
            "rejection_reasons": "",
            "consecutive_rejections": 0,
        }
        if pose_gating:
            candidate_prediction = predict_radar_at_pose(T_candidate, model, optix_backend)
            diagnostics, jump_reasons, prediction_reasons = pose_gate_diagnostics(
                radar_polar_cropped,
                initial_prediction,
                candidate_prediction,
                eps_best,
            )
            (
                rejected,
                rejection_reasons,
                localization_state,
                consecutive_rejections,
                healthy_recovery_accepts,
            ) = apply_pose_gate(
                decision_state,
                consecutive_rejections,
                healthy_recovery_accepts,
                jump_reasons,
                prediction_reasons,
            )
            if rejected:
                T_hat = T_init
                accepted_prediction = initial_prediction
            else:
                accepted_prediction = candidate_prediction
            gate_fields.update(
                diagnostics,
                optimizer_rejected=rejected,
                rejection_reasons=";".join(rejection_reasons),
                consecutive_rejections=consecutive_rejections,
            )
        previous_localized_pose = T_hat

        image_save_start = perf_counter()
        image_root = results_csv_path.parent
        if save_init:
            if initial_prediction.shape != radar_polar_cropped.shape:
                raise RuntimeError(
                    f"Initial prediction shape {initial_prediction.shape} does not match "
                    f"cropped radar shape {radar_polar_cropped.shape}."
                )
            padded_init = np.zeros_like(filtered_polar, dtype=np.float32)
            padded_init[:, :initial_prediction.shape[1]] = initial_prediction
            save_cartesian_radar_image(
                radar_frame,
                padded_init,
                image_root / "init_predictions" / f"{radar_frame.frame}.png",
            )
        if save_labels:
            save_cartesian_radar_image(
                radar_frame,
                filtered_polar / model.radar_normalization_scale,
                image_root / "labels" / f"{radar_frame.frame}.png",
                # image_root / "labels" / f"{radar_frame_idx}.png",
            )
        if save_predictions:
            predictions = accepted_prediction
            if predictions is None:
                predictions = predict_radar_at_pose(T_hat, model, optix_backend)
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
                # image_root / "predictions" / f"{radar_frame_idx}.png",
            )
        image_save_time_s = perf_counter() - image_save_start

        print("Frame timing")
        print(f"  frame setup [s]:        {frame_setup_time_s:.6f}")
        print(f"  mesh update [s]:        {mesh_update_time_s:.6f}")
        print(f"  radar preprocessing [s]: {radar_preprocessing_time_s:.6f}")
        print(f"  preprocessing wait [s]: {prefetch_wait_time_s:.6f}")
        print(f"  image save [s]:         {image_save_time_s:.6f}")
        print_dfo_timing_report(optimization_time_s, logger, cost_call_timings)
        print_localization_error_report(T_init, T_hat, T_gt)
        append_localization_result(
            results_csv_path,
            build_localization_result_row(
                loc_seq.ID,
                radar_frame.frame,
                T_init,
                T_hat,
                T_candidate,
                T_gt,
                logger.history_f,
                eps_best,
                candidate_cost,
                gate_fields,
                {
                    "optimization_mode": "2d" if use_2d else "3d",
                    "near_depth_patch_count": near_depth_patch_count,
                    "planar_frames_remaining": planar_frames_remaining,
                    "optimizer_budget": optimizer_budget,
                    "optimization_time_s": optimization_time_s,
                    "model_inference_time_s": sum(
                        timing["model_inference_time_s"] for timing in cost_call_timings
                    ),
                    "depth_patch_generation_time_s": sum(
                        timing["depth_patch_generation_time_s"] for timing in cost_call_timings
                    ),
                    "peak_cuda_memory_mb": peak_cuda_memory_mb,
                    "candidate_radar_cost": candidate_cost - candidate_prior_penalty,
                    "candidate_prior_penalty": candidate_prior_penalty,
                },
            ),
        )
        pose_row = build_pyboreas_result_row(radar_frame, lidar_frame, T_hat)
        pyboreas_rows[radar_frame.timestamp_micro] = [str(value) for value in pose_row]
        append_pyboreas_result(results_pose_path, pose_row)
        print(f"Saved localization result: {results_csv_path}")

        radar_frame.unload_data()
        print("radar frame unloaded!")
        if device.type == "cuda":
            torch.cuda.synchronize(device)
        iteration_time = perf_counter() - iteration_start
        completed_frames = radar_frame_idx - radar_start_frame + 1
        remaining_frames = total_frames - completed_frames
        if completed_frames == 1:
            print(
                f"Frame {completed_frames} / {total_frames} - "
                f"iteration: {iteration_time:.3f}s, time left: estimating"
            )
        else:
            steady_iteration_time_sum += iteration_time
            average_iteration_time = steady_iteration_time_sum / (completed_frames - 1)
            time_left_minutes = remaining_frames * average_iteration_time / 60.0
            print(
                f"Frame {completed_frames} / {total_frames} - "
                f"iteration: {iteration_time:.3f}s, avg: {average_iteration_time:.3f}s, "
                f"time left: {time_left_minutes:.3f}min"
            )
        radar_frame_idx += 1

    preprocessing_executor.shutdown()

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


def resolve_dro_odometry_paths(wrapper_dir, sequence_ids, override=None):
    sequence_ids = list(dict.fromkeys(sequence_ids))
    if override and len(sequence_ids) != 1:
        raise ValueError("--dro-odometry can only be used with one --loc-sequence.")
    # output_root = Path(wrapper_dir) / "external" / "wheel_odometry" / "output"
    output_root = Path(wrapper_dir) / "external" / "dro" / "output"
    return sequence_ids, {
        sequence_id: override
        or output_root / sequence_id / "odometry_result" / "azimuth_odometry.npz"
        for sequence_id in sequence_ids
    }


def main():
    parser = argparse.ArgumentParser(description="Run DFO radar-lidar localization.")
    parser.add_argument("--experiment-name", required=True)
    parser.add_argument("--map-sequence", required=True)
    parser.add_argument("--loc-sequence", nargs="+", required=True)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--radar-start-frame", type=int, default=0)
    parser.add_argument("--radar-end-frame", type=int, default=None)
    parser.add_argument("--imfil-budget", type=int, default=60)
    parser.add_argument("--imfil-function-delta", type=float)
    parser.add_argument("--imfil-stencil-delta", type=float)
    parser.add_argument("--save-predictions", action="store_true")
    parser.add_argument("--save-init", action="store_true")
    parser.add_argument("--save-labels", action="store_true")
    parser.add_argument("--dro-odometry", type=Path)
    parser.add_argument("--validate-dro-odometry", action="store_true")
    parser.add_argument("--pose-gating", action="store_true")
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
    for name in ("imfil_function_delta", "imfil_stencil_delta"):
        value = getattr(args, name)
        if value is not None and value <= 0:
            raise ValueError(f"--{name.replace('_', '-')} must be positive.")

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

    loc_sequence_ids, dro_odometry_paths = resolve_dro_odometry_paths(
        boreas_vtr_wrapper_dir, args.loc_sequence, args.dro_odometry
    )
    for sequence_id, path in dro_odometry_paths.items():
        if not path.is_file():
            raise FileNotFoundError(
                f"Odometry not found for {sequence_id}: {path}. Run wheel_odometry.py "
                "for this localization sequence or pass --dro-odometry."
            )

    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA device requested but CUDA is unavailable.")
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cuda.matmul.allow_tf32 = False
    lidar_results_dir = vtr_results / "lidar"
    weights_path = os.path.join(
        boreas_vtr_wrapper_dir,
        # "model_dev/route_weights/1-suburb/best_total.pth",
        # "model_dev/route_weights/1-farm/best_total.pth",
        "model_dev/route_weights/1-suburb-industrial-farm/best_total.pth",
    )
    model = load_radar_translator_model(weights_path, device)
    model = torch.compile(model)
    dataset = BoreasDataset(
        boreas_data,
        [[sequence_id] for sequence_id in dict.fromkeys((args.map_sequence, *loc_sequence_ids))],
    )

    patch_config = build_patch_config(
        fov_deg=6.0,
        res_deg=0.1,
        min_range=0.0,
        max_uv_edge_length=None,
        max_depth_jump=2.0,
    )
    patch_config["fov_deg"] = 6.0

    lambda_prior_tracking = 2.5
    lambda_prior_recovery = 1.0
    # lambda_prior_tracking = 0
    # lambda_prior_recovery = 0
    # sigma_x, sigma_y, sigma_z, sigma_roll, sigma_pitch, sigma_yaw = 0.075, 0.050, 0.05, 0.15, 0.15, 0.05 # OG3 best values
    sigma_x, sigma_y, sigma_z, sigma_roll, sigma_pitch, sigma_yaw = 0.2, 0.2, 0.2, 3.0, 3.0, 0.1
    # sigma_x, sigma_y, sigma_z, sigma_roll, sigma_pitch, sigma_yaw = 1000.0, 1000.0, 0.3, 3.0, 3.0, 0.1
    
    sigma_prior = np.array([
        sigma_x,
        sigma_y,
        sigma_z,
        np.deg2rad(sigma_roll),
        np.deg2rad(sigma_pitch),
        np.deg2rad(sigma_yaw)
    ])

    map_seq = dataset.get_seq_from_ID(args.map_sequence)
    for sequence_id in loc_sequence_ids:
        loc_seq = dataset.get_seq_from_ID(sequence_id)
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
            lambda_prior_tracking=lambda_prior_tracking,
            lambda_prior_recovery=lambda_prior_recovery,
            sigma_prior=sigma_prior,
            results_csv_path=results_csv_path,
            results_pose_path=results_pose_path,
            mesh_root=args.mesh_root,
            imfil_budget=args.imfil_budget,
            save_predictions=args.save_predictions,
            save_init=args.save_init,
            save_labels=args.save_labels,
            dro_odometry_path=dro_odometry_paths[sequence_id],
            validate_dro_odometry=args.validate_dro_odometry,
            pose_gating=args.pose_gating,
            imfil_function_delta=args.imfil_function_delta,
            imfil_stencil_delta=args.imfil_stencil_delta,
        )


if __name__ == "__main__":
    main()
