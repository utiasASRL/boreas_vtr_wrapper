import argparse
import csv
import gc
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
from scipy.interpolate import interp1d
from scipy.ndimage import shift

from localization_dfo.io_utils import (
    build_T_lidar_robot,
    build_patch_config,
    cen_filter_2d,
    get_path_vertices_with_submaps,
    load_submap_mesh_to_enu,
)
from localization_dfo.optix_backend import OptixDepthBackend
from localization_dfo.pipeline_dfo import (
    build_path_candidates,
    build_T_radar_robot,
    nearest_submap_idx,
)
from perturbation_cost_tests.perturbation_utils import (
    generate_delta_transforms,
    write_to_csv,
)
from perturbation_cost_tests.radar_translator_cnn import RadarTranslatorCNN


def correct_offsets_gt(radar_frame, frame_index, sequence):
    neighbors = sequence.radar_frames[frame_index - 1:frame_index + 2]
    times = [frame.timestamp_micro for frame in neighbors]
    body_rates = [frame.body_rate.flatten() for frame in neighbors]
    velocities = interp1d(times, body_rates, axis=0, kind="quadratic")(
        radar_frame.timestamps.flatten()
    )[:, :2]

    shifted = shift(
        radar_frame.polar,
        shift=(0, sequence.calib.radar_offset / radar_frame.resolution),
        order=3,
        mode="nearest",
    )
    azimuths = radar_frame.azimuths.flatten()
    radial = -velocities[:, 0] * np.cos(azimuths) - velocities[:, 1] * np.sin(azimuths)
    chirp_type = radar_frame.chirp_type.flatten()
    chirp_sign = np.where(chirp_type == 0, -1, chirp_type)
    bin_shift = chirp_sign * 0.05024 * radial / radar_frame.resolution
    corrected = np.empty_like(shifted)
    for row in range(len(shifted)):
        corrected[row] = shift(shifted[row], -bin_shift[row], order=3, mode="nearest")
    return corrected


def make_perturbations():
    # x_offsets = np.unique(
    #     np.concatenate([
    #         np.array([0.0]),
    #         np.linspace(-0.2, 0.2, 101),
    #     ])
    # )

    translation_offsets = {
        # "x": np.linspace(-0.3, 0.3, 601),
        # "y": np.linspace(-0.3, 0.3, 601),
        # "z": np.linspace(-0.3, 0.3, 601),
        "x": np.linspace(-2.0, 2.0, 41),
        "y": np.linspace(-2.0, 2.0, 41),
        "z": np.linspace(-1.0, 1.0, 21),
        # "x": [0.0],
        # "y": [0.0],
        # "z": [0.0],
    }

    rotation_offsets_deg = {
        # "roll": [0.0],
        # "pitch": [0.0],
        # "yaw": [0.0],
        # "roll": np.linspace(-2.0, 2.0, 401),
        # "pitch": np.linspace(-2.0, 2.0, 401),
        # "yaw": np.linspace(-2.0, 2.0, 401),
        "roll": np.linspace(-3.0, 3.0, 61),
        "pitch": np.linspace(-3.0, 3.0, 61),
        "yaw": np.linspace(-3.0, 3.0, 61),
    }

    return generate_delta_transforms(
        translation_offsets=translation_offsets,
        rotation_offsets_deg=rotation_offsets_deg,
        mode="axis",
        include_identity=True,
    )


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


def compute_cost(preds_np, obs_cropped):
    pred_cropped = preds_np[:, :obs_cropped.shape[1]]
    residual = pred_cropped - obs_cropped
    return float(np.mean(0.5 * residual ** 2))


def print_sorted_costs(csv_path):
    with open(csv_path, newline="") as f:
        rows = sorted(csv.DictReader(f), key=lambda row: float(row["cost"]))
    for row in rows:
        print(row)


def predict_polar_optix(
    tracer,
    T_radar_enu,
    model,
    batch_size=32,
    target_bins=6848,
):
    with torch.no_grad():
        depth_patches = tracer.trace(T_radar_enu)

    num_azimuths = depth_patches.shape[0]
    preds_np = np.zeros((num_azimuths, target_bins), dtype=np.float32)
    with torch.no_grad():
        for start in range(0, num_azimuths, batch_size):
            stop = min(start + batch_size, num_azimuths)
            preds = torch.sigmoid(model(depth_patches[start:stop].unsqueeze(1)))
            preds_np[start:stop, :preds.shape[1]] = preds.cpu().numpy()

    depth_np = depth_patches.cpu().numpy()
    hit_counts = np.count_nonzero(depth_np > 0.0, axis=(1, 2))
    stats = {
        "avg_hit_pixels": float(np.mean(hit_counts)),
        "min_hit_pixels": int(np.min(hit_counts)),
        "max_hit_pixels": int(np.max(hit_counts)),
    }
    return preds_np, stats


def save_debug_images_if_better(
    perturb,
    cost,
    gt_cost,
    perturbation_dir,
    radar_frame,
    filtered_polar,
    normalization_scale,
    preds_np,
    diff,
):
    if perturb["name"] == "identity":
        gt_cost = cost

    if cost <= gt_cost:
        print(perturb["name"])

        output_dir = f"{perturbation_dir}/{radar_frame.frame}"
        os.makedirs(output_dir, exist_ok=True)

        original_polar = radar_frame.polar
        cart_diff = diff_img = cart_gt = gt_img = cart_pred = pred_img = stacked_img = None
        try:
            radar_frame.polar = diff
            cart_diff = radar_frame.polar_to_cart(
                cart_resolution=0.2384,
                cart_pixel_width=1000,
                in_place=False,
            )
            diff_img = (cart_diff * 255.0).astype(np.uint8)

            radar_frame.polar = filtered_polar / normalization_scale
            cart_gt = radar_frame.polar_to_cart(
                cart_resolution=0.2384,
                cart_pixel_width=1000,
                in_place=False,
            )
            gt_img = (cart_gt * 255.0).astype(np.uint8)

            radar_frame.polar = preds_np
            cart_pred = radar_frame.polar_to_cart(
                cart_resolution=0.2384,
                cart_pixel_width=1000,
                in_place=False,
            )
            pred_img = (cart_pred * 255.0).astype(np.uint8)

            stacked_img = cv2.hconcat([gt_img, pred_img, diff_img])
            stacked_filename = os.path.join(output_dir, f"{perturb['name']}.png")
            cv2.imwrite(stacked_filename, stacked_img)
        finally:
            radar_frame.polar = original_polar
            del cart_diff, diff_img, cart_gt, gt_img, cart_pred, pred_img, stacked_img
            gc.collect()

    return gt_cost


def run_sequence(
    map_seq,
    loc_seq,
    lidar_results_dir,
    radar_start_frame,
    radar_end_frame,
    patch_config,
    model,
    device,
    mesh_root,
    output_dir,
    batch_size=32,
):
    print(f"Map SequenceID: {map_seq.ID}")
    print(f"Localization SequenceID: {loc_seq.ID}")
    print(f"Number Radar Frames: {len(loc_seq.radar_frames)}")

    end_frame = radar_end_frame
    if end_frame is None:
        end_frame = len(loc_seq.radar_frames) - 2
    else:
        end_frame = min(end_frame, len(loc_seq.radar_frames) - 2)

    radar_start_frame = max(radar_start_frame, 1)

    graph_dir = os.path.join(lidar_results_dir, map_seq.ID, map_seq.ID, "graph")
    _, path_submap_pairs = get_path_vertices_with_submaps(graph_dir=graph_dir)
    T_lidar_robot = build_T_lidar_robot(map_seq)
    submap_candidates = build_path_candidates(map_seq, path_submap_pairs, T_lidar_robot)
    T_robot_radar = np.linalg.inv(build_T_radar_robot(loc_seq, build_T_lidar_robot(loc_seq)))
    perturbations = make_perturbations()

    radar_frame_idx = radar_start_frame
    loaded_mesh_submap_stamp_us = None
    mesh_vertices_gpu = None
    mesh_triangles_gpu = None
    tracer = OptixDepthBackend(patch_config, device)

    while radar_frame_idx < end_frame + 1:
        t0 = perf_counter()
        radar_frame = loc_seq.get_radar(radar_frame_idx)

        poses = [
            get_inverse_tf(rad_frame.pose)
            for rad_frame in loc_seq.radar_frames[radar_frame_idx - 1:radar_frame_idx + 2]
        ]
        times = [
            rad_frame.timestamp_micro
            for rad_frame in loc_seq.radar_frames[radar_frame_idx - 1:radar_frame_idx + 2]
        ]
        query_times = radar_frame.timestamps.flatten().tolist()
        azimuth_poses = interpolate_poses(poses, times, query_times)
        radar_azimuths = radar_frame.azimuths.flatten()

        T_enu_radar = radar_frame.pose
        T_gt = np.linalg.inv(T_enu_radar)
        odom_transforms = np.array([T_enu_radar @ T_i for T_i in azimuth_poses])
        T_robot_enu = T_robot_radar @ T_gt
        curr_submap, lidar_frame, _ = submap_candidates[
            nearest_submap_idx(T_robot_enu, submap_candidates)
        ]

        submap_stamp_us = curr_submap.stamp // 1000
        if loaded_mesh_submap_stamp_us != submap_stamp_us:
            del mesh_vertices_gpu, mesh_triangles_gpu
            mesh_vertices_gpu, mesh_triangles_gpu, _ = load_submap_mesh_to_enu(
                mesh_root=mesh_root,
                sequence_id=map_seq.ID,
                submap=curr_submap,
                T_lidar_robot=T_lidar_robot,
                lidar_pose=lidar_frame.pose,
                device=device,
            )
            tracer.set_mesh(mesh_vertices_gpu, mesh_triangles_gpu)
            loaded_mesh_submap_stamp_us = submap_stamp_us

        tracer.set_scan(odom_transforms, radar_azimuths)

        shifted_polar = correct_offsets_gt(radar_frame, radar_frame_idx, loc_seq)
        filtered_polar = cen_filter_2d(
            shifted_polar,
            sigma_gauss=15.0,
            z_q=2.5,
            noise_scale=0.5,
        )
        filtered_polar[:, : math.ceil(patch_config["min_range"] / radar_frame.resolution)] = 0.0

        target_bins = filtered_polar.shape[1]
        output_bins = 2736

        obs_padded = filtered_polar / model.radar_normalization_scale
        obs_cropped = obs_padded[:, :output_bins]

        perturbation_dir = Path(output_dir)
        perturbation_dir.mkdir(parents=True, exist_ok=True)
        csv_path = perturbation_dir / f"{radar_frame.frame}.csv"

        gt_cost = float("inf")
        print(f"Processing radar frame {radar_frame.frame} {len(perturbations)} perturbations")
        for perturb in perturbations:
            t_pert = perf_counter()
            # generate_delta_transforms stores the transform applied to points in the
            # radar frame. Convert it back to the equivalent left perturbation of
            # the radar pose.
            T_perturbed = np.linalg.inv(perturb["delta_T"]) @ T_gt

            preds_np, patch_stats = predict_polar_optix(
                tracer=tracer,
                T_radar_enu=T_perturbed,
                model=model,
                batch_size=batch_size,
                target_bins=target_bins,
            )

            cost = compute_cost(preds_np, obs_cropped)

            diff = np.abs(preds_np - obs_padded)
            diff[:, output_bins:] = 0.0

            gt_cost = save_debug_images_if_better(
                perturb=perturb,
                cost=cost,
                gt_cost=gt_cost,
                perturbation_dir=str(perturbation_dir),
                radar_frame=radar_frame,
                filtered_polar=filtered_polar,
                normalization_scale=model.radar_normalization_scale,
                preds_np=preds_np,
                diff=diff,
            )

            write_to_csv(str(csv_path), perturb, cost)
            print(
                f"{perturb['name']}: cost={cost:.6e} | "
                f"avg hits={patch_stats['avg_hit_pixels']:.1f} | "
                f"time={perf_counter() - t_pert:.3f}s | saved csv"
            )

            del preds_np, diff
            gc.collect()

        print_sorted_costs(csv_path)

        radar_frame.unload_data()
        print(f"radar frame unloaded! elapsed={perf_counter() - t0:.3f}s")
        radar_frame_idx += 1


def main():
    parser = argparse.ArgumentParser(
        description="Plot perturbation costs using cached NKSR meshes and OptiX ray tracing."
    )
    parser.add_argument("--radar-start-frame", type=int, default=1)
    parser.add_argument("--radar-end-frame", type=int, default=None)
    parser.add_argument("--map-sequence", required=True)
    parser.add_argument("--loc-sequence", required=True)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--device", default="cuda:3" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--min-range-m", type=float, default=7.0)
    parser.add_argument("--mesh-root", type=Path, default=None)
    parser.add_argument("--model-weights", type=Path, default=None)
    args = parser.parse_args()

    if args.batch_size < 1:
        raise ValueError("--batch-size must be at least 1.")
    if not np.isfinite(args.min_range_m) or args.min_range_m < 0:
        raise ValueError("--min-range-m must be finite and non-negative.")

    boreas_vtr_wrapper_dir = os.getenv("VTRROOT")
    boreas_data = os.getenv("VTRRDATA")
    if boreas_vtr_wrapper_dir is None:
        raise RuntimeError("VTRROOT must be set.")
    if boreas_data is None:
        raise RuntimeError("VTRRDATA must be set.")

    device = torch.device(args.device)
    if device.type != "cuda" or not torch.cuda.is_available():
        raise RuntimeError("The OptiX perturbation test requires a CUDA device.")

    lidar_results_dir = os.path.join(boreas_vtr_wrapper_dir, "results/lidar")
    weights_path = args.model_weights or (
        Path(boreas_vtr_wrapper_dir)
        / "model_dev"
        / "route_weights"
        / "1-suburb-industrial-farm"
        / "best_total.pth"
    )
    mesh_root = args.mesh_root or (
        Path(boreas_vtr_wrapper_dir) / "postprocessing" / "submap_meshes"
    )
    bd = BoreasDataset(
        boreas_data,
        [[sequence_id] for sequence_id in dict.fromkeys((args.map_sequence, args.loc_sequence))],
    )
    model = load_radar_translator_model(weights_path, device)

    fov_deg = 6.0
    res_deg = 0.1
    patch_config = build_patch_config(
        fov_deg=fov_deg,
        res_deg=res_deg,
        min_range=args.min_range_m,
        max_uv_edge_length=None,
        max_depth_jump=2.0,
    )
    patch_config["fov_deg"] = fov_deg

    map_seq = bd.get_seq_from_ID(args.map_sequence)
    loc_seq = bd.get_seq_from_ID(args.loc_sequence)
    output_dir = Path("perturbation_cost_tests") / loc_seq.ID
    run_sequence(
        map_seq=map_seq,
        loc_seq=loc_seq,
        lidar_results_dir=lidar_results_dir,
        radar_start_frame=args.radar_start_frame,
        radar_end_frame=args.radar_end_frame,
        patch_config=patch_config,
        model=model,
        device=device,
        mesh_root=mesh_root,
        output_dir=output_dir,
        batch_size=args.batch_size,
    )


if __name__ == "__main__":
    main()
