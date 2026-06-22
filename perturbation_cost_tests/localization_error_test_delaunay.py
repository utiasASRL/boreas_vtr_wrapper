import gc
import os
from time import perf_counter

import cv2
import numpy as np
import pandas as pd
import torch
from pyboreas import BoreasDataset
from pyboreas.utils.odometry import interpolate_poses
from pyboreas.utils.utils import get_inverse_tf
from pylgmath import Transformation
from vtr_utils.plot_utils import convert_points_to_frame, extract_points_from_vertex

from localization.pipeline import (
    build_lidar_to_robot_transform,
    build_patch_config,
    cen_filter_2d,
    correct_offsets,
    get_submap_vertices,
    load_radar_translator_model,
)
from perturbation_cost_tests.perturbation_utils import (
    generate_delta_transforms,
    write_to_csv,
)
from postprocessing.submap_to_depth_image_multiprocess_refactored import (
    barycentric_interpolate_depth_image,
    project_points_to_patch_samples,
    transform_points_fast,
)


def make_perturbations():
    # x_offsets = np.unique(
    #     np.concatenate([
    #         np.array([0.0]),
    #         np.linspace(-0.2, 0.2, 101),
    #     ])
    # )

    translation_offsets = {
        "x": [0.0],
        # "y": np.linspace(-0.2, 0.2, 41),
        # "z": np.linspace(-0.2, 0.2, 41),
        "y": [0.0],
        "z": [0.0],
    }

    rotation_offsets_deg = {
        "roll": [0.0],
        "pitch": [0.0],
        # "yaw": [0.0],
        # "roll": np.linspace(-2.0, 2.0, 41),
        # "pitch": np.linspace(-2.0, 2.0, 41),
        "yaw": np.linspace(-2.0, 2.0, 81),
    }

    return generate_delta_transforms(
        translation_offsets=translation_offsets,
        rotation_offsets_deg=rotation_offsets_deg,
        mode="axis",
        include_identity=True,
    )


def predict_polar_barycentric(
    map_pts_enu,
    T_radar_enu,
    odom_transforms,
    radar_azimuths,
    patch_config,
    model,
    device,
    batch_size=32,
    target_bins=6848,
):
    """
    Memory-conscious forward chain:

        map points in ENU
        -> per-azimuth radar frame
        -> local spherical patch coords using theta - radar_azi
        -> Delaunay/barycentric depth patch
        -> sigmoid(model(depth_patch))

    Returns a full-width polar prediction for visualization compatibility.
    """
    num_azimuths = len(odom_transforms)
    output_bins = 2736
    preds_np = np.zeros((num_azimuths, target_bins), dtype=np.float32)

    patches = []
    patch_rows = []
    selected_counts = []

    def flush_batch():
        if not patches:
            return

        patches_np = np.stack(patches).astype(np.float32, copy=False)
        with torch.no_grad():
            x = torch.from_numpy(patches_np).unsqueeze(1).to(device)
            logits = model(x)
            preds = torch.sigmoid(logits).detach().cpu().numpy()

        for row, pred in zip(patch_rows, preds):
            preds_np[row, :pred.shape[0]] = pred

        patches.clear()
        patch_rows.clear()

    for row, (odom_i, radar_azi) in enumerate(zip(odom_transforms, radar_azimuths)):
        T_i = T_radar_enu @ odom_i
        P_r = transform_points_fast(T_i, map_pts_enu)
        I = project_points_to_patch_samples(P_r, radar_azi, patch_config)
        selected_counts.append(I.shape[0])
        D = barycentric_interpolate_depth_image(I, patch_config)

        patches.append(D)
        patch_rows.append(row)

        if len(patches) >= batch_size:
            flush_batch()

    flush_batch()

    stats = {
        "avg_selected_points": float(np.mean(selected_counts)) if selected_counts else 0.0,
        "min_selected_points": int(np.min(selected_counts)) if selected_counts else 0,
        "max_selected_points": int(np.max(selected_counts)) if selected_counts else 0,
        "output_bins": output_bins,
    }
    return preds_np, stats


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
    batch_size=32,
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
    perturbations = make_perturbations()

    lidar_frame_idx = 0
    submap_vertices_idx = 0
    radar_frame_idx = radar_start_frame

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
        odom_transforms = np.array([
            T_enu_radar @ T_i
            for T_i in azimuth_poses
        ])

        map_pts_robot = extract_points_from_vertex(curr_submap, msg="pointmap")
        map_pts_lidar = convert_points_to_frame(map_pts_robot, T_lidar_robot)

        T_enu_lidar = Transformation(T_ba=lidar_frame.pose)
        map_pts_enu = convert_points_to_frame(map_pts_lidar, T_enu_lidar)

        shifted_polar = correct_offsets(radar_frame, radar_frame_idx, seq)
        filtered_polar = cen_filter_2d(
            shifted_polar,
            sigma_gauss=15.0,
            z_q=2.5,
            noise_scale=0.5,
        )

        norm_factor = 0.5613
        target_bins = filtered_polar.shape[1]
        obs_padded = filtered_polar / norm_factor
        obs_cropped = obs_padded[:, :2736]

        perturbation_dir = f"delaunay_yaw"
        os.makedirs(perturbation_dir, exist_ok=True)
        csv_path = f"{perturbation_dir}/{radar_frame.frame}.csv"

        gt_cost = float("inf")
        print(f"Processing radar frame {radar_frame.frame} with {len(perturbations)} perturbations")

        for perturb in perturbations:
            t_pert = perf_counter()
            # generate_delta_transforms stores the transform applied to points
            # when the radar frame is perturbed. Convert it back to the
            # equivalent left perturbation of the radar pose.
            T_perturbed = np.linalg.inv(perturb["delta_T"]) @ T_gt

            preds_np, patch_stats = predict_polar_barycentric(
                map_pts_enu=map_pts_enu,
                T_radar_enu=T_perturbed,
                odom_transforms=odom_transforms,
                radar_azimuths=radar_azimuths,
                patch_config=patch_config,
                model=model,
                device=device,
                batch_size=batch_size,
                target_bins=target_bins,
            )

            residual = preds_np[:, :2736] - obs_cropped
            cost = 0.5 * float(np.sum(residual ** 2))

            diff = np.abs(preds_np - obs_padded)
            diff[:, 2736:] = 0.0

            gt_cost = save_debug_images_if_better(
                perturb=perturb,
                cost=cost,
                gt_cost=gt_cost,
                perturbation_dir=perturbation_dir,
                radar_frame=radar_frame,
                filtered_polar=filtered_polar,
                norm_factor=norm_factor,
                preds_np=preds_np,
                diff=diff,
            )

            write_to_csv(csv_path, perturb, cost)
            print(
                f"{perturb['name']}: cost={cost:.6e} | "
                f"avg selected={patch_stats['avg_selected_points']:.1f} | "
                f"time={perf_counter() - t_pert:.3f}s | saved to csv"
            )

            del preds_np, residual, diff
            gc.collect()

        df = pd.read_csv(csv_path)
        df_sorted = df.sort_values("cost")
        print(df_sorted)

        radar_frame.unload_data()
        print(f"radar frame unloaded! elapsed={perf_counter() - t0:.3f}s")
        radar_frame_idx += 1


def main():
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
        "model_dev/model_weights/6_deg_attentional_MSE_delauney/best.pth",
    )
    model = load_radar_translator_model(weights_path, device)

    radar_start_frame = 65
    radar_end_frame = 200
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
        # if seq.ID != "boreas-2025-01-08-10-59":
        #     print(f"Skipping {seq.ID}")
        #     continue

        run_sequence(
            seq=seq,
            lidar_results_dir=lidar_results_dir,
            radar_start_frame=radar_start_frame,
            radar_end_frame=radar_end_frame,
            patch_config=patch_config,
            model=model,
            device=device,
            batch_size=32,
        )
        break


if __name__ == "__main__":
    main()
