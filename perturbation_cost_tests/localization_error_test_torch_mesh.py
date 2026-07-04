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

from localization.pipeline import (
    build_lidar_to_robot_transform,
    build_patch_config,
    cen_filter_2d,
    correct_offsets,
    get_submap_vertices,
)
from localization_fast.gauss_newton_localization_fast import (
    GeometryParams,
    torch_mesh_depth_geometry_batch,
)
from perturbation_cost_tests.perturbation_utils import (
    generate_delta_transforms,
    write_to_csv,
)
from perturbation_cost_tests.radar_translator_cnn import RadarTranslatorCNN
from postprocessing.mesh_to_depth_image import load_submap_mesh_to_enu


def make_perturbations():
    # x_offsets = np.unique(
    #     np.concatenate([
    #         np.array([0.0]),
    #         np.linspace(-0.2, 0.2, 101),
    #     ])
    # )

    translation_offsets = {
        "x": np.linspace(-0.3, 0.3, 601),
        "y": np.linspace(-0.3, 0.3, 601),
        "z": np.linspace(-0.3, 0.3, 601),
        # "x": np.linspace(-0.2, 0.2, 41),
        # "y": np.linspace(-0.2, 0.2, 41),
        # "z": np.linspace(-0.2, 0.2, 41),
        # "x": [0.0],
        # "y": [0.0],
        # "z": [0.0],
    }

    rotation_offsets_deg = {
        # "roll": [0.0],
        # "pitch": [0.0],
        # "yaw": [0.0],
        "roll": np.linspace(-2.0, 2.0, 401),
        "pitch": np.linspace(-2.0, 2.0, 401),
        "yaw": np.linspace(-2.0, 2.0, 401),
        # "roll": np.linspace(-2.0, 2.0, 41),
        # "pitch": np.linspace(-2.0, 2.0, 41),
        # "yaw": np.linspace(-2.0, 2.0, 41),
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


def build_geometry_params(patch_config):
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


def predict_polar_mesh(
    mesh_vertices,
    mesh_triangles,
    T_radar_enu,
    odom_transforms,
    radar_azimuths,
    geom,
    model,
    device,
    batch_size=32,
    geometry_batch_size=16,
    target_bins=6848,
):
    num_azimuths = len(odom_transforms)
    preds_np = np.zeros((num_azimuths, target_bins), dtype=np.float32)
    lidar_waveforms = np.zeros((num_azimuths, target_bins), dtype=np.float32)

    patches = []
    patch_rows = []
    covered_pixels = []
    candidate_faces = []

    def flush_batch():
        if not patches:
            return
        patches_np = np.stack(patches).astype(np.float32, copy=False)
        with torch.no_grad():
            x = torch.from_numpy(patches_np).unsqueeze(1).to(device)
            preds = torch.sigmoid(model(x)).detach().cpu().numpy()
        for row, pred in zip(patch_rows, preds):
            preds_np[row, :pred.shape[0]] = pred
        patches.clear()
        patch_rows.clear()

    for start in range(0, num_azimuths, geometry_batch_size):
        stop = min(start + geometry_batch_size, num_azimuths)
        batch_indices = np.arange(start, stop)
        mesh_outputs, _ = torch_mesh_depth_geometry_batch(
            P_v=mesh_vertices,
            triangles=mesh_triangles,
            T=T_radar_enu,
            odom_batch=odom_transforms[batch_indices],
            radar_azimuth_batch=radar_azimuths[batch_indices],
            geom=geom,
            device=device,
            with_jacobian=False,
        )
        for row, (depth_patch, _, mesh_stats) in zip(batch_indices, mesh_outputs):
            lidar_waveforms[row] = depth_patch_to_waveform(depth_patch, bins=target_bins)
            patches.append(depth_patch)
            patch_rows.append(row)
            covered_pixels.append(mesh_stats["covered_pixels"])
            candidate_faces.append(mesh_stats["candidate_faces"])
            if len(patches) >= batch_size:
                flush_batch()

    flush_batch()
    stats = {
        "avg_covered_pixels": float(np.mean(covered_pixels)) if covered_pixels else 0.0,
        "min_covered_pixels": int(np.min(covered_pixels)) if covered_pixels else 0,
        "max_covered_pixels": int(np.max(covered_pixels)) if covered_pixels else 0,
        "avg_candidate_faces": float(np.mean(candidate_faces)) if candidate_faces else 0.0,
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
    seq,
    lidar_results_dir,
    radar_start_frame,
    radar_end_frame,
    patch_config,
    model,
    device,
    mesh_root,
    output_dir,
    batch_size=32,
    geometry_batch_size=16,
    use_covisibility=False,
    use_gaussian_blur=False,
    loss="mse",
    cauchy_c=0.1
):
    print(f"SequenceID: {seq.ID}")
    print(f"Number Radar Frames: {len(seq.radar_frames)}")

    end_frame = radar_end_frame
    if end_frame is None:
        end_frame = len(seq.radar_frames) - 2
    else:
        end_frame = min(end_frame, len(seq.radar_frames) - 2)

    radar_start_frame = max(radar_start_frame, 1)

    graph_dir = os.path.join(lidar_results_dir, seq.ID, seq.ID, "graph")
    _, submap_vertices = get_submap_vertices(graph_dir=graph_dir)
    T_lidar_robot = build_lidar_to_robot_transform(seq)
    perturbations = make_perturbations()

    lidar_frame_idx = 0
    submap_vertices_idx = 0
    radar_frame_idx = radar_start_frame
    loaded_mesh_submap_stamp_us = None
    mesh_vertices_gpu = None
    mesh_triangles_gpu = None
    geom = build_geometry_params(patch_config)

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

        t0 = perf_counter()
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
        T_gt = np.linalg.inv(T_enu_radar)
        odom_transforms = np.array([T_enu_radar @ T_i for T_i in azimuth_poses])

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

        shifted_polar = correct_offsets(radar_frame, radar_frame_idx, seq)
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

            preds_np, lidar_waveforms, patch_stats = predict_polar_mesh(
                mesh_vertices=mesh_vertices_gpu,
                mesh_triangles=mesh_triangles_gpu,
                T_radar_enu=T_perturbed,
                odom_transforms=odom_transforms,
                radar_azimuths=radar_azimuths,
                geom=geom,
                model=model,
                device=device,
                batch_size=batch_size,
                geometry_batch_size=geometry_batch_size,
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
                f"avg covered={patch_stats['avg_covered_pixels']:.1f} | "
                f"avg faces={patch_stats['avg_candidate_faces']:.1f} | "
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
        description="Plot perturbation costs using cached NKSR meshes and the fast mesh forward chain."
    )
    parser.add_argument("--radar-start-frame", type=int, default=65)
    parser.add_argument("--radar-end-frame", type=int, default=665)
    parser.add_argument("--sequence-id", default="boreas-2024-12-03-12-54")
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--geometry-batch-size", type=int, default=400)
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
    if args.geometry_batch_size < 1:
        raise ValueError("--geometry-batch-size must be at least 1.")

    boreas_vtr_wrapper_dir = os.getenv("VTRROOT")
    boreas_data = os.getenv("VTRRDATA")
    if boreas_vtr_wrapper_dir is None:
        raise RuntimeError("VTRROOT must be set.")
    if boreas_data is None:
        raise RuntimeError("VTRRDATA must be set.")

    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA requested but not available.")

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
            / f"torch_mesh_loss_{args.loss}_cauchy_{args.cauchy_c}_covis_{int(args.covisibility)}_blur_{int(args.gaussian_blur)}"
        )
    else:
        output_dir = Path("perturbation_cost_tests") / f"{args.output_dir}"

    bd = BoreasDataset(boreas_data)
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

    for seq in bd.sequences:
        if args.sequence_id is not None and seq.ID != args.sequence_id:
            print(f"Skipping {seq.ID}")
            continue
        run_sequence(
            seq=seq,
            lidar_results_dir=lidar_results_dir,
            radar_start_frame=args.radar_start_frame,
            radar_end_frame=args.radar_end_frame,
            patch_config=patch_config,
            model=model,
            device=device,
            mesh_root=mesh_root,
            output_dir=output_dir,
            batch_size=args.batch_size,
            geometry_batch_size=args.geometry_batch_size,
            use_covisibility=args.covisibility,
            use_gaussian_blur=args.gaussian_blur,
            loss=args.loss,
            cauchy_c=args.cauchy_c
        )


if __name__ == "__main__":
    main()
