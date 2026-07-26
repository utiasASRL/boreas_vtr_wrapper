import argparse
import csv
import gc
import os
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np
import torch
from pyboreas import BoreasDataset
from pyboreas.utils.odometry import interpolate_poses
from pyboreas.utils.utils import get_inverse_tf
from scipy.ndimage import gaussian_filter1d, maximum_filter1d

from localization_dfo.io_utils import (
    build_T_lidar_robot,
    build_patch_config,
    cen_filter_2d,
    correct_offsets,
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
        "x": np.linspace(-0.2, 0.2, 41),
        "y": np.linspace(-0.2, 0.2, 41),
        "z": np.linspace(-0.2, 0.2, 41),
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
        "roll": np.linspace(-2.0, 2.0, 41),
        "pitch": np.linspace(-2.0, 2.0, 41),
        "yaw": np.linspace(-2.0, 2.0, 41),
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


def cauchy_loss(residual, c=0.1):
    """
    Cauchy robust loss.

    residual: numpy array
    c: robust scale parameter, in the same units as residual intensity

    rho(r) = 0.5 * c^2 * log(1 + (r / c)^2)
    """
    r_scaled = residual / c
    return 0.5 * (c ** 2) * np.log1p(r_scaled ** 2)


def robust_cost(residual, loss="mse", cauchy_c=0.1):
    if loss == "mse":
        return 0.5 * residual ** 2
    if loss == "cauchy":
        return cauchy_loss(residual, c=cauchy_c)
    raise ValueError(f"Unknown loss: {loss}")


def gaussian_kernel_1d(window_size=25, sigma=3.0):
    x = np.arange(window_size, dtype=np.float32) - window_size // 2
    kernel = np.exp(-(x ** 2) / (2.0 * sigma ** 2))
    return kernel / kernel.max()


def depth_patch_to_waveform(
    depth_patch,
    resolution=0.04381,
    bins=6848,
    window_size=25,
    sigma=3.0,
):
    waveform = np.zeros(bins, dtype=np.float32)
    nonzero_depths = depth_patch[np.isfinite(depth_patch) & (depth_patch != 0.0)]
    if nonzero_depths.size == 0:
        return waveform

    indices = np.round(nonzero_depths / resolution).astype(np.int64)
    valid_indices = indices[(indices >= 0) & (indices < bins)]
    if valid_indices.size == 0:
        return waveform

    waveform[valid_indices] = 1.0
    kernel = gaussian_kernel_1d(window_size=window_size, sigma=sigma)
    smoothed = np.convolve(waveform, kernel, mode="same")
    return np.clip(smoothed, 0.0, 1.0).astype(np.float32, copy=False)


def build_covisibility_mask(
    lidar_waveforms,
    radar_waveforms,
    window_size=9,
    lidar_threshold=1e-2,
    radar_threshold=1e-2,
):
    lidar_mask = lidar_waveforms > lidar_threshold
    radar_mask = radar_waveforms > radar_threshold
    positive_centers = lidar_mask & radar_mask

    roi_mask = maximum_filter1d(
        positive_centers.astype(np.uint8),
        size=window_size,
        axis=1,
        mode="constant",
        cval=0,
    ).astype(bool)

    return roi_mask, positive_centers


def compute_covisibility_cost(
    preds_np,
    obs_cropped,
    lidar_waveforms,
    window_size=9,
    lidar_threshold=1e-2,
    radar_threshold=1e-2,
    loss="mse",
    cauchy_c=0.1
):
    pred_cropped = preds_np[:, :obs_cropped.shape[1]]
    lidar_cropped = lidar_waveforms[:, :obs_cropped.shape[1]]
    roi_mask, positive_centers = build_covisibility_mask(
        lidar_waveforms=lidar_cropped,
        radar_waveforms=obs_cropped,
        window_size=window_size,
        lidar_threshold=lidar_threshold,
        radar_threshold=radar_threshold,
    )

    active_bins = int(np.count_nonzero(roi_mask))
    positive_bins = int(np.count_nonzero(positive_centers))
    if active_bins == 0:
        return float("inf"), roi_mask, {
            "active_roi_bins": active_bins,
            "positive_center_bins": positive_bins,
        }

    residual = pred_cropped - obs_cropped
    cost = float(np.mean(robust_cost(residual[roi_mask], loss=loss, cauchy_c=cauchy_c)))
    stats = {
        "active_roi_bins": active_bins,
        "positive_center_bins": positive_bins,
    }
    return cost, roi_mask, stats


def compute_plain_cost(preds_np, obs_cropped, loss, cauchy_c):
    pred_cropped = preds_np[:, :obs_cropped.shape[1]]
    residual = pred_cropped - obs_cropped
    cost = float(np.mean(robust_cost(residual, loss=loss, cauchy_c=cauchy_c)))
    return cost


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
    lidar_waveforms = np.zeros((num_azimuths, target_bins), dtype=np.float32)
    with torch.no_grad():
        for start in range(0, num_azimuths, batch_size):
            stop = min(start + batch_size, num_azimuths)
            preds = torch.sigmoid(model(depth_patches[start:stop].unsqueeze(1)))
            preds_np[start:stop, :preds.shape[1]] = preds.cpu().numpy()

    depth_np = depth_patches.cpu().numpy()
    hit_counts = np.count_nonzero(depth_np > 0.0, axis=(1, 2))
    for row, depth_patch in enumerate(depth_np):
        lidar_waveforms[row] = depth_patch_to_waveform(depth_patch, bins=target_bins)

    stats = {
        "avg_hit_pixels": float(np.mean(hit_counts)),
        "min_hit_pixels": int(np.min(hit_counts)),
        "max_hit_pixels": int(np.max(hit_counts)),
    }
    return preds_np, lidar_waveforms, stats


def save_debug_images_if_better(
    perturb,
    cost,
    gt_cost,
    perturbation_dir,
    radar_frame,
    filtered_polar,
    norm_factor,
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

            radar_frame.polar = filtered_polar / norm_factor
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
    use_covisibility=False,
    use_gaussian_blur=False,
    loss="mse",
    cauchy_c=0.1
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

        shifted_polar = correct_offsets(radar_frame, radar_frame_idx, loc_seq)
        filtered_polar = cen_filter_2d(
            shifted_polar,
            sigma_gauss=15.0,
            z_q=2.5,
            noise_scale=0.5,
        )
        if use_gaussian_blur:
            filtered_polar = gaussian_filter1d(
                filtered_polar,
                sigma=30.0,
                axis=1,
                mode="reflect",
            )

        norm_factor = 0.5613
        target_bins = filtered_polar.shape[1]
        output_bins = 2736
        covis_window_size = 15
        lidar_threshold = 5e-2
        radar_threshold = 5e-2

        obs_padded = filtered_polar / norm_factor
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

            preds_np, lidar_waveforms, patch_stats = predict_polar_optix(
                tracer=tracer,
                T_radar_enu=T_perturbed,
                model=model,
                batch_size=batch_size,
                target_bins=target_bins,
            )

            if use_covisibility:
                cost, roi_mask, cost_stats = compute_covisibility_cost(
                    preds_np=preds_np,
                    obs_cropped=obs_cropped,
                    lidar_waveforms=lidar_waveforms,
                    window_size=covis_window_size,
                    lidar_threshold=lidar_threshold,
                    radar_threshold=radar_threshold,
                    loss=loss,
                    cauchy_c=cauchy_c
                )
            else:
                cost = compute_plain_cost(preds_np, obs_cropped, loss, cauchy_c)
                roi_mask = np.ones_like(obs_cropped, dtype=bool)
                cost_stats = {
                    "active_roi_bins": int(np.count_nonzero(roi_mask)),
                    "positive_center_bins": 0,
                }

            diff = np.abs(preds_np - obs_padded)
            diff[:, :output_bins] *= roi_mask
            diff[:, output_bins:] = 0.0

            gt_cost = save_debug_images_if_better(
                perturb=perturb,
                cost=cost,
                gt_cost=gt_cost,
                perturbation_dir=str(perturbation_dir),
                radar_frame=radar_frame,
                filtered_polar=filtered_polar,
                norm_factor=norm_factor,
                preds_np=preds_np,
                diff=diff,
            )

            write_to_csv(str(csv_path), perturb, cost)
            print(
                f"{perturb['name']}: cost={cost:.6e} | "
                f"roi bins={cost_stats['active_roi_bins']} | "
                f"positive bins={cost_stats['positive_center_bins']} | "
                f"avg hits={patch_stats['avg_hit_pixels']:.1f} | "
                f"time={perf_counter() - t_pert:.3f}s | saved csv"
            )

            del preds_np, lidar_waveforms, roi_mask, diff
            gc.collect()

        print_sorted_costs(csv_path)

        radar_frame.unload_data()
        print(f"radar frame unloaded! elapsed={perf_counter() - t0:.3f}s")
        radar_frame_idx += 100


def main():
    parser = argparse.ArgumentParser(
        description="Plot perturbation costs using cached NKSR meshes and OptiX ray tracing."
    )
    parser.add_argument("--radar-start-frame", type=int, default=65)
    parser.add_argument("--radar-end-frame", type=int, default=None)
    parser.add_argument("--map-sequence", required=True)
    parser.add_argument("--loc-sequence", required=True)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--covisibility", action="store_true")
    parser.add_argument("--gaussian-blur", action="store_true")
    parser.add_argument("--mesh-root", type=Path, default=None)
    parser.add_argument("--model-weights", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--loss", choices=["mse", "cauchy"], default="mse")
    parser.add_argument("--cauchy-c", type=float, default=0.1)
    args = parser.parse_args()

    if args.batch_size < 1:
        raise ValueError("--batch-size must be at least 1.")

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
        / "model_weights"
        / "6_deg_attentional_skip_bigger"
        / "best.pth"
    )
    mesh_root = args.mesh_root or (
        Path(boreas_vtr_wrapper_dir) / "postprocessing" / "submap_meshes"
    )
    if args.output_dir is None:
        output_dir = (
            Path("perturbation_cost_tests")
            / f"optix_loss_{args.loss}_cauchy_{args.cauchy_c}_covis_{int(args.covisibility)}_blur_{int(args.gaussian_blur)}"
        )
    else:
        output_dir = Path("perturbation_cost_tests") / f"{args.output_dir}"

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
        min_range=0.0,
        max_uv_edge_length=None,
        max_depth_jump=2.0,
    )
    patch_config["fov_deg"] = fov_deg

    map_seq = bd.get_seq_from_ID(args.map_sequence)
    loc_seq = bd.get_seq_from_ID(args.loc_sequence)
    output_dir = output_dir / loc_seq.ID
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
        use_covisibility=args.covisibility,
        use_gaussian_blur=args.gaussian_blur,
        loss=args.loss,
        cauchy_c=args.cauchy_c
    )


if __name__ == "__main__":
    main()
