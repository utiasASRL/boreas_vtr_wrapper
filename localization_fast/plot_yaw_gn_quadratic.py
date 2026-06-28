#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


DOF_COLUMNS = {
    "x": "dx",
    "y": "dy",
    "z": "dz",
    "roll": "roll_deg",
    "pitch": "pitch_deg",
    "yaw": "yaw_deg",
}
DOF_DIMS = {name: idx for idx, name in enumerate(DOF_COLUMNS)}
ROTATION_DOFS = {"roll", "pitch", "yaw"}
YAW_ONLY_COLUMNS = ["dx", "dy", "dz", "roll_deg", "pitch_deg"]


def dof_unit(dof: str) -> str:
    return "deg" if dof in ROTATION_DOFS else "m"


def dof_to_solver(value: np.ndarray | float, dof: str) -> np.ndarray | float:
    return np.deg2rad(value) if dof in ROTATION_DOFS else value


def make_delta_for_dof(dof: str, value: float):
    from perturbation_cost_tests.perturbation_utils import make_delta_T

    translation = np.zeros(3)
    rpy_deg = np.zeros(3)
    if dof in ROTATION_DOFS:
        rpy_deg[DOF_DIMS[dof] - 3] = value
    else:
        translation[DOF_DIMS[dof]] = value
    return make_delta_T(translation=translation, rpy_deg=rpy_deg)


def load_yaw_curve(csv_path: Path) -> pd.DataFrame:
    required = ["cost", "dx", "dy", "dz", "roll_deg", "pitch_deg", "yaw_deg"]
    df = pd.read_csv(csv_path)
    missing = [col for col in required if col not in df.columns]
    if missing:
        raise ValueError(f"{csv_path} is missing required columns: {missing}")

    mask = np.ones(len(df), dtype=bool)
    for col in YAW_ONLY_COLUMNS:
        mask &= np.isclose(df[col].to_numpy(dtype=float), 0.0)

    yaw_df = df[mask].copy()
    if yaw_df.empty:
        raise ValueError(f"No yaw-only rows found in {csv_path}.")

    yaw_df = yaw_df.sort_values("yaw_deg")
    yaw_df = yaw_df.drop_duplicates(subset=["yaw_deg"], keep="last")
    return yaw_df


def load_dof_curve(csv_path: Path, dof: str) -> pd.DataFrame:
    dof_col = DOF_COLUMNS[dof]
    required = ["cost", *DOF_COLUMNS.values()]
    df = pd.read_csv(csv_path)
    missing = [col for col in required if col not in df.columns]
    if missing:
        raise ValueError(f"{csv_path} missing required columns: {missing}")

    mask = np.ones(len(df), dtype=bool)
    for col in DOF_COLUMNS.values():
        if col != dof_col:
            mask &= np.isclose(df[col].to_numpy(dtype=float), 0.0)

    dof_df = df[mask].copy()
    if dof_df.empty:
        raise ValueError(f"No {dof}-only rows found in {csv_path}.")

    dof_df = dof_df.sort_values(dof_col)
    dof_df = dof_df.drop_duplicates(subset=[dof_col], keep="last")
    return dof_df


def interpolate_cost(yaw_deg: np.ndarray, cost: np.ndarray, center_yaw_deg: float) -> float:
    if center_yaw_deg < yaw_deg[0] or center_yaw_deg > yaw_deg[-1]:
        raise ValueError(
            f"center_yaw_deg={center_yaw_deg} is outside sampled range "
            f"[{yaw_deg[0]}, {yaw_deg[-1]}]."
        )
    return float(np.interp(center_yaw_deg, yaw_deg, cost))


def finite_difference_slope_cost_per_rad(
    yaw_deg: np.ndarray,
    cost: np.ndarray,
    center_yaw_deg: float,
) -> tuple[float, float, float]:
    left_indices = np.flatnonzero(yaw_deg < center_yaw_deg)
    right_indices = np.flatnonzero(yaw_deg > center_yaw_deg)

    if len(left_indices) == 0 or len(right_indices) == 0:
        raise ValueError("Need sampled yaw points on both sides of center_yaw_deg for a slope check.")

    left_idx = int(left_indices[-1])
    right_idx = int(right_indices[0])

    dyaw_deg = yaw_deg[right_idx] - yaw_deg[left_idx]
    if np.isclose(dyaw_deg, 0.0):
        raise ValueError("Nearest yaw samples are degenerate.")

    slope_per_deg = (cost[right_idx] - cost[left_idx]) / dyaw_deg
    slope_per_rad = slope_per_deg * (180.0 / np.pi)
    return float(slope_per_rad), float(yaw_deg[left_idx]), float(yaw_deg[right_idx])


def build_quadratic(
    yaw_deg: np.ndarray,
    center_yaw_deg: float,
    c0: float,
    g_yaw: float,
    h_yaw: float,
) -> np.ndarray:
    delta_rad = np.deg2rad(yaw_deg - center_yaw_deg)
    return c0 + g_yaw * delta_rad + 0.5 * h_yaw * delta_rad**2


def format_yaw_for_filename(yaw_deg: float) -> str:
    return f"{yaw_deg:+.6f}".rstrip("0").rstrip(".").replace("+", "pos_").replace("-", "neg_").replace(".", "p")


def make_yaw_samples(
    center_yaw_deg: float,
    global_min_deg: float,
    global_max_deg: float,
    global_step_deg: float,
    local_window_deg: float,
    local_step_deg: float,
) -> np.ndarray:
    global_samples = np.arange(
        global_min_deg,
        global_max_deg + 0.5 * global_step_deg,
        global_step_deg,
    )
    local_samples = np.arange(
        center_yaw_deg - local_window_deg,
        center_yaw_deg + local_window_deg + 0.5 * local_step_deg,
        local_step_deg,
    )
    samples = np.concatenate([global_samples, local_samples, [center_yaw_deg]])
    samples = np.unique(np.round(samples, decimals=10))
    return samples[(samples >= global_min_deg) & (samples <= global_max_deg)]


def save_generated_yaw_curve(
    csv_path: Path,
    yaw_deg: np.ndarray,
    costs: np.ndarray,
    center_yaw_deg: float,
    c0: float,
    g_yaw: float,
    h_yaw: float,
) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    df = pd.DataFrame({
        "name": [f"yaw_{yaw:+.6f}deg" for yaw in yaw_deg],
        "cost": costs,
        "dx": 0.0,
        "dy": 0.0,
        "dz": 0.0,
        "roll_deg": 0.0,
        "pitch_deg": 0.0,
        "yaw_deg": yaw_deg,
        "linearization_yaw_deg": center_yaw_deg,
        "gn_C0": c0,
        "gn_g_yaw_per_rad": g_yaw,
        "gn_H_yaw": h_yaw,
    })
    df.to_csv(csv_path, index=False)


def plot_yaw_curve_and_quadratic(
    csv_path: Path,
    center_yaw_deg: float,
    c0: float,
    g_yaw: float,
    h_yaw: float,
    output_path: Path,
    window_deg: Optional[float] = None,
    quadratic_window_deg: float = 0.5,
    num_quad: int = 1000,
    align_value_to_csv: bool = False,
    show: bool = False,
) -> None:
    yaw_df = load_yaw_curve(csv_path)
    yaw_deg_all = yaw_df["yaw_deg"].to_numpy(dtype=float)
    cost_all = yaw_df["cost"].to_numpy(dtype=float)

    center_cost_csv = interpolate_cost(yaw_deg_all, cost_all, center_yaw_deg)
    fd_slope_rad, fd_left_deg, fd_right_deg = finite_difference_slope_cost_per_rad(
        yaw_deg_all,
        cost_all,
        center_yaw_deg,
    )

    if window_deg is None:
        plot_mask = np.ones_like(yaw_deg_all, dtype=bool)
        yaw_min = float(yaw_deg_all[0])
        yaw_max = float(yaw_deg_all[-1])
    else:
        yaw_min = center_yaw_deg - window_deg
        yaw_max = center_yaw_deg + window_deg
        plot_mask = (yaw_deg_all >= yaw_min) & (yaw_deg_all <= yaw_max)
        if not np.any(plot_mask):
            raise ValueError(f"No sampled points found in requested window [{yaw_min}, {yaw_max}] deg.")

    yaw_plot = yaw_deg_all[plot_mask]
    cost_plot = cost_all[plot_mask]
    quad_min = max(yaw_min, center_yaw_deg - quadratic_window_deg)
    quad_max = min(yaw_max, center_yaw_deg + quadratic_window_deg)
    yaw_quad = np.linspace(quad_min, quad_max, max(int(num_quad), 2))
    cost_quad = build_quadratic(yaw_quad, center_yaw_deg, c0, g_yaw, h_yaw)
    quadratic_cost_ceiling = max(float(cost_all[0]), float(cost_all[-1]))
    cost_quad_visible = np.where(cost_quad <= quadratic_cost_ceiling, cost_quad, np.nan)

    if np.isclose(h_yaw, 0.0):
        gn_step_deg = np.nan
        gn_yaw_star_deg = np.nan
        gn_cost_star = np.nan
    else:
        gn_step_rad = -g_yaw / h_yaw
        gn_step_deg = float(np.rad2deg(gn_step_rad))
        gn_yaw_star_deg = float(center_yaw_deg + gn_step_deg)
        gn_cost_star = float(c0 + g_yaw * gn_step_rad + 0.5 * h_yaw * gn_step_rad**2)

    sample_min_idx = int(np.argmin(cost_all))
    sample_min_yaw = float(yaw_deg_all[sample_min_idx])
    sample_min_cost = float(cost_all[sample_min_idx])

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(yaw_plot, cost_plot, "o-", linewidth=1.5, markersize=4, label="sampled nonlinear cost")
    ax.plot(yaw_quad, cost_quad_visible, "-", linewidth=2.0, label="GN quadratic")

    if align_value_to_csv:
        shifted_quad = build_quadratic(
            yaw_quad,
            center_yaw_deg,
            center_cost_csv,
            g_yaw,
            h_yaw,
        )
        shifted_quad_visible = np.where(
            shifted_quad <= quadratic_cost_ceiling,
            shifted_quad,
            np.nan,
        )
        ax.plot(
            yaw_quad,
            shifted_quad_visible,
            "--",
            linewidth=1.8,
            label="GN quadratic shifted to CSV C0",
        )

    ax.axvline(center_yaw_deg, color="black", linestyle=":", linewidth=1.5, label="GN center")
    ax.plot([center_yaw_deg], [c0], "s", color="tab:orange", label="GN C0")
    ax.plot([center_yaw_deg], [center_cost_csv], "x", color="black", markersize=8, label="CSV cost at center")
    ax.plot([sample_min_yaw], [sample_min_cost], "*", color="tab:green", markersize=12, label="sampled minimum")
    if np.isfinite(gn_yaw_star_deg):
        ax.plot([gn_yaw_star_deg], [gn_cost_star], "D", color="tab:red", markersize=7, label="GN quadratic minimum")

    ax.set_xlabel("yaw perturbation [deg]")
    ax.set_ylabel("cost = 0.5 * ||residual||^2")
    ax.set_title(f"Yaw Cost vs GN Quadratic: {csv_path.stem}")
    if window_deg is not None:
        ax.set_xlim(yaw_min, yaw_max)
    ax.grid(True, alpha=0.35)
    ax.legend(fontsize=8)
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=200)

    print(f"CSV:                         {csv_path}")
    print(f"Output:                      {output_path}")
    print(f"sampled yaw range [deg]:      {yaw_deg_all[0]:.6f} to {yaw_deg_all[-1]:.6f}")
    print(f"center yaw [deg]:             {center_yaw_deg:.6f}")
    print(f"GN C0:                        {c0:.9e}")
    print(f"CSV C(center):                {center_cost_csv:.9e}")
    print(f"C0 difference GN-CSV:         {c0 - center_cost_csv:.9e}")
    print(f"GN g=dC/dyaw_rad:             {g_yaw:.9e}")
    print(f"CSV finite-diff slope/rad:    {fd_slope_rad:.9e}")
    print(f"slope bracket [deg]:          {fd_left_deg:.6f}, {fd_right_deg:.6f}")
    print(f"slope difference GN-CSV:      {g_yaw - fd_slope_rad:.9e}")
    print(f"GN H=J^T J yaw:               {h_yaw:.9e}")
    print(f"GN predicted step [deg]:      {gn_step_deg:.9e}")
    print(f"GN predicted yaw [deg]:       {gn_yaw_star_deg:.9e}")
    print(f"GN quadratic min cost:        {gn_cost_star:.9e}")
    print(f"sampled min yaw [deg]:        {sample_min_yaw:.9e}")
    print(f"sampled min cost:             {sample_min_cost:.9e}")

    if show:
        plt.show()
    plt.close(fig)


def format_value_for_filename(value: float) -> str:
    return (
        f"{value:+.6f}"
        .rstrip("0")
        .rstrip(".")
        .replace("+", "pos_")
        .replace("-", "neg_")
        .replace(".", "p")
    )


def make_dof_samples(
    center_value: float,
    global_min: float,
    global_max: float,
    global_step: float,
    local_window: float,
    local_step: float,
) -> np.ndarray:
    global_samples = np.arange(
        global_min,
        global_max + 0.5 * global_step,
        global_step,
    )
    local_samples = np.arange(
        center_value - local_window,
        center_value + local_window + 0.5 * local_step,
        local_step,
    )
    samples = np.concatenate([global_samples, local_samples, [center_value]])
    samples = np.unique(np.round(samples, decimals=10))
    return samples[(samples >= global_min) & (samples <= global_max)]


def save_generated_dof_curve(
    csv_path: Path,
    dof: str,
    values: np.ndarray,
    costs: np.ndarray,
    center_value: float,
    c0: float,
    g: float,
    h: float,
) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "name": [f"{dof}_{value:+.6f}{dof_unit(dof)}" for value in values],
        "cost": costs,
        "linearization_dof": dof,
        "linearization_value": center_value,
        "gn_C0": c0,
        "gn_g": g,
        "gn_H": h,
    }
    for name, col in DOF_COLUMNS.items():
        data[col] = values if name == dof else 0.0
    pd.DataFrame(data).to_csv(csv_path, index=False)


def interpolate_dof_cost(values: np.ndarray, cost: np.ndarray, center_value: float) -> float:
    if center_value < values[0] or center_value > values[-1]:
        raise ValueError(
            f"center={center_value} outside sampled range [{values[0]}, {values[-1]}]."
        )
    return float(np.interp(center_value, values, cost))


def finite_difference_slope_solver_units(
    values: np.ndarray,
    cost: np.ndarray,
    center_value: float,
    dof: str,
) -> tuple[float, float, float]:
    left_indices = np.flatnonzero(values < center_value)
    right_indices = np.flatnonzero(values > center_value)
    if len(left_indices) == 0 or len(right_indices) == 0:
        raise ValueError(f"Need sampled {dof} points on both sides of center.")
    left_idx = int(left_indices[-1])
    right_idx = int(right_indices[0])
    delta = dof_to_solver(values[right_idx], dof) - dof_to_solver(values[left_idx], dof)
    if np.isclose(delta, 0.0):
        raise ValueError(f"Nearest {dof} samples degenerate.")
    slope = (cost[right_idx] - cost[left_idx]) / delta
    return float(slope), float(values[left_idx]), float(values[right_idx])


def build_dof_quadratic(
    values: np.ndarray,
    center_value: float,
    c0: float,
    g: float,
    h: float,
    dof: str,
) -> np.ndarray:
    delta = dof_to_solver(values, dof) - dof_to_solver(center_value, dof)
    return c0 + g * delta + 0.5 * h * delta**2


def plot_dof_curve_and_quadratic(
    csv_path: Path,
    dof: str,
    center_value: float,
    c0: float,
    g: float,
    h: float,
    output_path: Path,
    window: Optional[float] = None,
    quadratic_window: float = 0.5,
    num_quad: int = 1000,
    align_value_to_csv: bool = False,
    show: bool = False,
) -> None:
    dof_col = DOF_COLUMNS[dof]
    unit = dof_unit(dof)
    curve_df = load_dof_curve(csv_path, dof)
    values_all = curve_df[dof_col].to_numpy(dtype=float)
    cost_all = curve_df["cost"].to_numpy(dtype=float)
    center_cost_csv = interpolate_dof_cost(values_all, cost_all, center_value)
    fd_slope, fd_left, fd_right = finite_difference_slope_solver_units(
        values_all,
        cost_all,
        center_value,
        dof,
    )

    if window is None:
        plot_mask = np.ones_like(values_all, dtype=bool)
        value_min = float(values_all[0])
        value_max = float(values_all[-1])
    else:
        value_min = center_value - window
        value_max = center_value + window
        plot_mask = (values_all >= value_min) & (values_all <= value_max)
    if not np.any(plot_mask):
        raise ValueError(f"No sampled points found in requested window [{value_min}, {value_max}].")

    values_plot = values_all[plot_mask]
    cost_plot = cost_all[plot_mask]
    quad_min = max(value_min, center_value - quadratic_window)
    quad_max = min(value_max, center_value + quadratic_window)
    values_quad = np.linspace(quad_min, quad_max, max(int(num_quad), 2))
    cost_quad = build_dof_quadratic(values_quad, center_value, c0, g, h, dof)
    quadratic_cost_ceiling = max(float(cost_all[0]), float(cost_all[-1]))
    cost_quad_visible = np.where(cost_quad <= quadratic_cost_ceiling, cost_quad, np.nan)

    if np.isclose(h, 0.0):
        gn_step = np.nan
        gn_star = np.nan
        gn_cost_star = np.nan
    else:
        gn_step_solver = -g / h
        gn_step = float(np.rad2deg(gn_step_solver) if dof in ROTATION_DOFS else gn_step_solver)
        gn_star = float(center_value + gn_step)
        gn_cost_star = float(c0 + g * gn_step_solver + 0.5 * h * gn_step_solver**2)

    sample_min_idx = int(np.argmin(cost_all))
    sample_min_value = float(values_all[sample_min_idx])
    sample_min_cost = float(cost_all[sample_min_idx])

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(values_plot, cost_plot, "o-", linewidth=1.5, markersize=4, label="sampled nonlinear cost")
    ax.plot(values_quad, cost_quad_visible, "-", linewidth=2.0, label="GN quadratic")
    if align_value_to_csv:
        shifted_quad = build_dof_quadratic(values_quad, center_value, center_cost_csv, g, h, dof)
        shifted_quad_visible = np.where(shifted_quad <= quadratic_cost_ceiling, shifted_quad, np.nan)
        ax.plot(
            values_quad,
            shifted_quad_visible,
            "--",
            linewidth=1.5,
            label="GN quadratic shifted to CSV center",
        )
    ax.axvline(center_value, color="black", linestyle=":", linewidth=1.5, label="GN center")
    ax.plot([center_value], [c0], "s", color="tab:orange", label="GN C0")
    ax.plot([center_value], [center_cost_csv], "x", color="black", markersize=8, label="CSV cost at center")
    ax.plot([sample_min_value], [sample_min_cost], "*", color="tab:green", markersize=12, label="sampled minimum")
    if np.isfinite(gn_star):
        ax.plot([gn_star], [gn_cost_star], "D", color="tab:red", label="GN quadratic minimum")

    ax.set_xlabel(f"{dof} perturbation [{unit}]")
    ax.set_ylabel("cost")
    ax.set_title(f"{csv_path.stem}: {dof} cost curve vs GN quadratic")
    if window is not None:
        ax.set_xlim(value_min, value_max)
    ax.grid(True, alpha=0.35)
    ax.legend(fontsize=8)
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=200)

    print(f"CSV: {csv_path}")
    print(f"Output: {output_path}")
    print(f"sampled {dof} range [{unit}]: {values_all[0]:.6f} {values_all[-1]:.6f}")
    print(f"center {dof} [{unit}]: {center_value:.6f}")
    print(f"GN C0: {c0:.9e}")
    print(f"CSV C(center): {center_cost_csv:.9e}")
    print(f"C0 difference GN-CSV: {c0 - center_cost_csv:.9e}")
    print(f"GN g=dC/d{dof}_solver: {g:.9e}")
    print(f"CSV finite-diff slope/solver: {fd_slope:.9e}")
    print(f"slope bracket [{unit}]: {fd_left:.6f}, {fd_right:.6f}")
    print(f"slope difference GN-CSV: {g - fd_slope:.9e}")
    print(f"GN H=J^T J {dof}: {h:.9e}")
    print(f"GN predicted step [{unit}]: {gn_step:.9e}")
    print(f"GN predicted {dof} [{unit}]: {gn_star:.9e}")
    print(f"GN quadratic min cost: {gn_cost_star:.9e}")
    print(f"sampled min {dof} [{unit}]: {sample_min_value:.9e}")
    print(f"sampled min cost: {sample_min_cost:.9e}")

    if show:
        plt.show()
    plt.close(fig)


def prepare_sequence_context(
    sequence_id,
    device_name,
    weights_path,
    fov_deg,
    res_deg,
    geometry_backend,
    mesh_root,
):
    import torch
    from pyboreas import BoreasDataset

    from localization.pipeline import (
        build_lidar_to_robot_transform,
        build_patch_config,
        get_submap_vertices,
    )
    from localization_fast.pipeline_fast import load_radar_translator_model
    from localization_fast.gauss_newton_localization_fast import GeometryParams

    root = os.getenv("VTRROOT")
    data = os.getenv("VTRRDATA")
    if root is None:
        raise RuntimeError("VTRROOT must be set.")
    if data is None:
        raise RuntimeError("VTRRDATA must be set.")

    device = torch.device(device_name if device_name is not None else ("cuda" if torch.cuda.is_available() else "cpu"))
    weights = weights_path or Path(root) / "model_dev/model_weights/6_deg_attentional_skip_bigger/best.pth"
    model = load_radar_translator_model(str(weights), device)

    dataset = BoreasDataset(data)
    seq = next((candidate for candidate in dataset.sequences if candidate.ID == sequence_id), None)
    if seq is None:
        raise ValueError(f"Sequence not found: {sequence_id}")

    graph_dir = Path(root) / "results/lidar" / seq.ID / seq.ID / "graph"
    _, submap_vertices = get_submap_vertices(graph_dir=str(graph_dir))
    patch_config = build_patch_config(
        fov_deg=fov_deg,
        res_deg=res_deg,
        min_range=0.0,
        max_uv_edge_length=None,
        max_depth_jump=2.0,
    )
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
    return {
        "seq": seq,
        "model": model,
        "device": device,
        "submap_vertices": submap_vertices,
        "T_lidar_robot": build_lidar_to_robot_transform(seq),
        "geom": geom,
        "geometry_backend": geometry_backend,
        "mesh_root": mesh_root,
        "loaded_mesh_submap_stamp_us": None,
        "mesh_vertices_gpu": None,
        "mesh_triangles_gpu": None,
        "radar_index": {str(frame.frame): idx for idx, frame in enumerate(seq.radar_frames)},
        "lidar_index": {int(frame.frame): idx for idx, frame in enumerate(seq.lidar_frames)},
    }


def prepare_frame_inputs(context, frame_id):
    import torch
    from pyboreas.utils.odometry import interpolate_poses
    from pyboreas.utils.utils import get_inverse_tf
    from pylgmath import Transformation
    from vtr_utils.plot_utils import convert_points_to_frame, extract_points_from_vertex

    from localization.pipeline import cen_filter_2d, correct_offsets
    from postprocessing.mesh_to_depth_image import load_submap_mesh_to_enu

    seq = context["seq"]
    frame_id = str(frame_id)
    if frame_id not in context["radar_index"]:
        raise ValueError(f"Radar frame not found in {seq.ID}: {frame_id}")
    radar_idx = context["radar_index"][frame_id]
    if radar_idx <= 0 or radar_idx >= len(seq.radar_frames) - 1:
        raise ValueError(f"Radar frame {frame_id} is too close to the sequence boundary.")

    radar_stamp_ms = int(frame_id)
    curr_submap = next(
        (
            context["submap_vertices"][idx]
            for idx in range(len(context["submap_vertices"]) - 1)
            if context["submap_vertices"][idx].stamp // 1000
            <= radar_stamp_ms
            <= context["submap_vertices"][idx + 1].stamp // 1000
        ),
        None,
    )
    if curr_submap is None:
        raise ValueError(f"No submap contains radar frame {frame_id}.")

    lidar_stamp_ms = curr_submap.stamp // 1000
    if lidar_stamp_ms not in context["lidar_index"]:
        raise ValueError(f"No lidar frame found for submap stamp {lidar_stamp_ms}.")
    lidar_frame = seq.lidar_frames[context["lidar_index"][lidar_stamp_ms]]
    radar_frame = seq.get_radar(radar_idx)

    poses = [
        get_inverse_tf(frame.pose)
        for frame in seq.radar_frames[radar_idx - 1:radar_idx + 2]
    ]
    times = [
        frame.timestamp_micro
        for frame in seq.radar_frames[radar_idx - 1:radar_idx + 2]
    ]
    azimuth_poses = interpolate_poses(poses, times, radar_frame.timestamps.flatten().tolist())
    radar_azimuths = radar_frame.azimuths.flatten()
    T_enu_radar = radar_frame.pose
    T_gt = np.linalg.inv(T_enu_radar)
    odom_transforms = np.asarray([T_enu_radar @ T_i for T_i in azimuth_poses])

    mesh_triangles = None
    if context["geometry_backend"] == "torch_mesh":
        submap_stamp_us = curr_submap.stamp // 1000
        if context["loaded_mesh_submap_stamp_us"] != submap_stamp_us:
            context["mesh_vertices_gpu"], context["mesh_triangles_gpu"], _ = load_submap_mesh_to_enu(
                mesh_root=context["mesh_root"],
                sequence_id=seq.ID,
                submap=curr_submap,
                T_lidar_robot=context["T_lidar_robot"],
                lidar_pose=lidar_frame.pose,
                device=context["device"],
            )
            context["loaded_mesh_submap_stamp_us"] = submap_stamp_us
        P_v = context["mesh_vertices_gpu"]
        mesh_triangles = context["mesh_triangles_gpu"]
    else:
        map_pts_robot = extract_points_from_vertex(curr_submap, msg="pointmap")
        map_pts_lidar = convert_points_to_frame(map_pts_robot, context["T_lidar_robot"])
        map_pts_enu = convert_points_to_frame(
            map_pts_lidar,
            Transformation(T_ba=lidar_frame.pose),
        )
        P_v = torch.as_tensor(map_pts_enu.T, device=context["device"], dtype=torch.float32)

    shifted_polar = correct_offsets(radar_frame, radar_idx, seq)
    filtered_polar = cen_filter_2d(shifted_polar, sigma_gauss=15.0, z_q=2.5, noise_scale=0.5)
    observation = filtered_polar[:, :2736] / 0.5613

    return {
        "P_v": P_v,
        "mesh_triangles": mesh_triangles,
        "T_gt": T_gt,
        "odom_transforms": odom_transforms,
        "radar_azimuths": radar_azimuths,
        "observation": observation,
        "radar_frame": radar_frame,
    }


def compute_frame_quadratic_and_curve(
    context,
    frame_inputs,
    dof,
    center_value,
    samples,
    linearization_batch_size,
    geometry_batch_size,
):
    from localization_fast.gauss_newton_localization_fast import (
        ResidualBuildOptions,
        compute_radar_cost_only_fast,
        compute_radar_residual_and_jacobian_fast,
    )
    options = ResidualBuildOptions(
        device=str(context["device"]),
        model_output_activation="sigmoid",
        active_dims=[DOF_DIMS[dof]],
        linearization_batch_size=linearization_batch_size,
        candidate_batch_size=linearization_batch_size,
        geometry_backend=context["geometry_backend"],
        geometry_batch_size=geometry_batch_size,
        keep_diagnostics=False,
    )
    T_center = make_delta_for_dof(dof, center_value) @ frame_inputs["T_gt"]
    lin = compute_radar_residual_and_jacobian_fast(
        P_v=frame_inputs["P_v"],
        mesh_triangles=frame_inputs["mesh_triangles"],
        T=T_center,
        odom_transforms=frame_inputs["odom_transforms"],
        m_obs_all=frame_inputs["observation"],
        model=context["model"],
        geom=context["geom"],
        options=options,
        radar_azimuths=frame_inputs["radar_azimuths"],
    )

    costs = []
    for sample_idx, value in enumerate(samples, start=1):
        print(f"    cost sample {sample_idx}/{len(samples)}: {dof} {value:+.3f} {dof_unit(dof)}")
        T_sample = make_delta_for_dof(dof, value) @ frame_inputs["T_gt"]
        result = compute_radar_cost_only_fast(
            P_v=frame_inputs["P_v"],
            mesh_triangles=frame_inputs["mesh_triangles"],
            T=T_sample,
            odom_transforms=frame_inputs["odom_transforms"],
            m_obs_all=frame_inputs["observation"],
            model=context["model"],
            geom=context["geom"],
            options=options,
            radar_azimuths=frame_inputs["radar_azimuths"],
        )
        costs.append(result.cost)

    dim = DOF_DIMS[dof]
    return float(lin.cost), float(lin.g[dim]), float(lin.H[dim, dim]), np.asarray(costs)


def process_failed_frames(args) -> None:
    results_path = args.results_csv.expanduser().resolve()
    results_df = pd.read_csv(results_path)
    dof = args.dof
    value_col = f"final_{dof}_{dof_unit(dof)}"
    required = ["sequence_id", "frame_id", value_col]
    missing = [column for column in required if column not in results_df.columns]
    if missing:
        raise ValueError(f"{results_path} is missing required columns: {missing}")

    failed = results_df[np.abs(results_df[value_col].astype(float)) >= args.failure_threshold].copy()
    if failed.empty:
        print(f"No frames exceed |{value_col}| >= {args.failure_threshold}.")
        return

    output_dir = results_path.parent / "GN_quadratic_curves" / results_path.stem
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"Processing {len(failed)} failed frames. Output: {output_dir}")
    sampling_local_window_deg = (
        args.local_window_deg
        if args.local_window_deg is not None
        else 0.5
    )
    plot_window_deg = (
        args.local_window_deg
        if args.local_window_deg is not None
        else args.window_deg
    )

    for sequence_id, sequence_rows in failed.groupby("sequence_id", sort=False):
        context = None
        for row_idx, row in sequence_rows.iterrows():
            frame_id = str(int(row["frame_id"]))
            final_value = float(row[value_col])
            center_value = float(args.center) if args.center is not None else final_value
            center_source = "override center" if args.center is not None else f"final {dof} error"
            print(
                f"\nFrame {frame_id}: {center_source} "
                f"{center_value:+.6f} {dof_unit(dof)}"
            )
            value_label = format_value_for_filename(center_value)
            sample_csv = output_dir / f"{frame_id}_center_{dof}_{value_label}_{dof_unit(dof)}_samples.csv"
            output_png = output_dir / f"{frame_id}_center_{dof}_{value_label}_{dof_unit(dof)}.png"

            if sample_csv.exists() and not args.recompute_costs:
                cached_df = pd.read_csv(sample_csv)
                metadata_columns = ["linearization_dof", "linearization_value", "gn_C0", "gn_g", "gn_H"]
                missing = [column for column in metadata_columns if column not in cached_df.columns]
                if missing:
                    print(f"[WARN] Cache lacks GN metadata {missing}; recomputing costs.")
                else:
                    cached_center = float(cached_df["linearization_value"].iloc[0])
                    print(f"  Using cached cost samples: {sample_csv}")
                    plot_dof_curve_and_quadratic(
                        csv_path=sample_csv,
                        dof=dof,
                        center_value=cached_center,
                        c0=float(cached_df["gn_C0"].iloc[0]),
                        g=float(cached_df["gn_g"].iloc[0]),
                        h=float(cached_df["gn_H"].iloc[0]),
                        output_path=output_png,
                        window=plot_window_deg,
                        quadratic_window=args.quadratic_window,
                        num_quad=args.num_quad,
                        align_value_to_csv=args.align_value_to_csv,
                        show=False,
                    )
                    continue

            if context is None:
                context = prepare_sequence_context(
                    sequence_id=str(sequence_id),
                    device_name=args.device,
                weights_path=None if args.weights_path is None else args.weights_path.expanduser().resolve(),
                fov_deg=args.fov_deg,
                res_deg=args.res_deg,
                geometry_backend=args.geometry_backend,
                mesh_root=args.mesh_root,
            )

            frame_inputs = None
            try:
                frame_inputs = prepare_frame_inputs(context, frame_id)
                samples = make_dof_samples(
                    center_value=center_value,
                    global_min=args.global_min,
                    global_max=args.global_max,
                    global_step=args.global_step,
                    local_window=sampling_local_window_deg,
                    local_step=args.local_step,
                )
                c0, g, h, costs = compute_frame_quadratic_and_curve(
                    context=context,
                    frame_inputs=frame_inputs,
                    dof=dof,
                    center_value=center_value,
                    samples=samples,
                    linearization_batch_size=args.linearization_batch_size,
                    geometry_batch_size=args.geometry_batch_size,
                )
                save_generated_dof_curve(
                    sample_csv,
                    dof,
                    samples,
                    costs,
                    center_value,
                    c0,
                    g,
                    h,
                )
                plot_dof_curve_and_quadratic(
                    csv_path=sample_csv,
                    dof=dof,
                    center_value=center_value,
                    c0=c0,
                    g=g,
                    h=h,
                    output_path=output_png,
                    window=plot_window_deg,
                    quadratic_window=args.quadratic_window,
                    num_quad=args.num_quad,
                    align_value_to_csv=args.align_value_to_csv,
                    show=False,
                )
            except Exception as err:
                print(f"[ERROR] Failed frame {frame_id}: {err}")
            finally:
                if frame_inputs is not None:
                    frame_inputs["radar_frame"].unload_data()


def compute_gn_quadratic_terms_from_frame(
    frame_id: str,
    dof: str,
    center_value: float,
    sequence_id: Optional[str],
    weights_path: Optional[Path],
    fov_deg: float,
    res_deg: float,
    linearization_batch_size: int,
    device_name: Optional[str],
    geometry_backend: str,
    mesh_root: Path,
) -> tuple[float, float, float]:
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
    )
    from localization_fast.pipeline_fast import load_radar_translator_model
    from localization_fast.gauss_newton_localization_fast import (
        GeometryParams,
        ResidualBuildOptions,
        compute_radar_residual_and_jacobian_fast,
    )
    from postprocessing.mesh_to_depth_image import load_submap_mesh_to_enu
    boreas_vtr_wrapper_dir = os.getenv("VTRROOT")
    boreas_data = os.getenv("VTRRDATA")
    if boreas_vtr_wrapper_dir is None:
        raise RuntimeError("VTRROOT must be set.")
    if boreas_data is None:
        raise RuntimeError("VTRRDATA must be set.")

    device = torch.device(device_name if device_name is not None else ("cuda" if torch.cuda.is_available() else "cpu"))
    if weights_path is None:
            weights_path = Path(boreas_vtr_wrapper_dir) / "model_dev/model_weights/6_deg_attentional_skip_bigger/best.pth"
    model = load_radar_translator_model(str(weights_path), device)

    patch_config = build_patch_config(
        fov_deg=fov_deg,
        res_deg=res_deg,
        min_range=0.0,
        max_uv_edge_length=None,
        max_depth_jump=2.0,
    )

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

    lidar_results_dir = Path(boreas_vtr_wrapper_dir) / "results/lidar"
    bd = BoreasDataset(boreas_data)
    frame_id_str = str(frame_id)

    for seq in bd.sequences:
        if sequence_id is not None and seq.ID != sequence_id:
            continue

        print(f"Searching sequence {seq.ID} for radar frame {frame_id_str}...")
        radar_idx = None
        for idx, radar_frame in enumerate(seq.radar_frames):
            if str(radar_frame.frame) == frame_id_str:
                radar_idx = idx
                break
        if radar_idx is None:
            continue
        if radar_idx <= 0 or radar_idx >= len(seq.radar_frames) - 1:
            raise ValueError(f"Radar frame {frame_id_str} is too close to the sequence boundary.")

        graph_dir = lidar_results_dir / seq.ID / seq.ID / "graph"
        _, submap_vertices = get_submap_vertices(graph_dir=str(graph_dir))
        T_lidar_robot = build_lidar_to_robot_transform(seq)

        radar_frame_meta = seq.radar_frames[radar_idx]
        radar_stamp_ms = int(radar_frame_meta.frame)

        submap_vertices_idx = None
        for idx in range(len(submap_vertices) - 1):
            curr_submap = submap_vertices[idx]
            next_submap = submap_vertices[idx + 1]
            if curr_submap.stamp // 1000 <= radar_stamp_ms <= next_submap.stamp // 1000:
                submap_vertices_idx = idx
                break
        if submap_vertices_idx is None:
            raise ValueError(f"Could not find a submap containing radar frame {frame_id_str}.")

        curr_submap = submap_vertices[submap_vertices_idx]
        lidar_stamp_ms = curr_submap.stamp // 1000
        lidar_idx = None
        for idx, lidar_frame_meta in enumerate(seq.lidar_frames):
            if int(lidar_frame_meta.frame) == lidar_stamp_ms:
                lidar_idx = idx
                break
        if lidar_idx is None:
            raise ValueError(f"Could not find lidar frame {lidar_stamp_ms} for submap vertex.")

        print(f"Loading sequence {seq.ID}, radar index {radar_idx}, lidar index {lidar_idx}...")
        radar_frame = seq.get_radar(radar_idx)
        lidar_frame = seq.lidar_frames[lidar_idx]

        poses = [
            get_inverse_tf(rad_frame.pose)
            for rad_frame in seq.radar_frames[radar_idx - 1:radar_idx + 2]
        ]
        times = [
            rad_frame.timestamp_micro
            for rad_frame in seq.radar_frames[radar_idx - 1:radar_idx + 2]
        ]
        query_times = radar_frame.timestamps.flatten().tolist()
        azimuth_poses = interpolate_poses(poses, times, query_times)
        radar_azimuths = radar_frame.azimuths.flatten()

        T_enu_radar = radar_frame.pose
        T_gt = np.linalg.inv(T_enu_radar)
        odom_transforms = np.array([T_enu_radar @ T_i for T_i in azimuth_poses])

        mesh_triangles = None
        if geometry_backend == "torch_mesh":
            P_v, mesh_triangles, _ = load_submap_mesh_to_enu(
                mesh_root=mesh_root,
                sequence_id=seq.ID,
                submap=curr_submap,
                T_lidar_robot=T_lidar_robot,
                lidar_pose=lidar_frame.pose,
                device=device,
            )
        else:
            map_pts_robot = extract_points_from_vertex(curr_submap, msg="pointmap")
            map_pts_lidar = convert_points_to_frame(map_pts_robot, T_lidar_robot)
            T_enu_lidar = Transformation(T_ba=lidar_frame.pose)
            map_pts_enu = convert_points_to_frame(map_pts_lidar, T_enu_lidar)
            P_v = torch.as_tensor(map_pts_enu.T, device=device, dtype=torch.float32)

        shifted_polar = correct_offsets(radar_frame, radar_idx, seq)
        filtered_polar = cen_filter_2d(shifted_polar, sigma_gauss=15.0, z_q=2.5, noise_scale=0.5)
        radar_polar_cropped = filtered_polar[:, :2736] / 0.5613

        T_offset = make_delta_for_dof(dof, center_value)
        T_center = T_offset @ T_gt

        options = ResidualBuildOptions(
            device=str(device),
            model_output_activation="sigmoid",
            active_dims=[DOF_DIMS[dof]],
            linearization_batch_size=linearization_batch_size,
            candidate_batch_size=linearization_batch_size,
            geometry_backend=geometry_backend,
        )

        print(f"Computing {dof}-only GN linearization...")
        lin = compute_radar_residual_and_jacobian_fast(
                P_v=P_v,
                mesh_triangles=mesh_triangles,
                T=T_center,
            odom_transforms=odom_transforms,
            m_obs_all=radar_polar_cropped,
            model=model,
            geom=geom,
            options=options,
            radar_azimuths=radar_azimuths,
        )

        radar_frame.unload_data()
        dim = DOF_DIMS[dof]
        return float(lin.cost), float(lin.g[dim]), float(lin.H[dim, dim])

    if sequence_id is None:
        raise ValueError(f"Could not find radar frame {frame_id_str} in any sequence.")
    raise ValueError(f"Could not find radar frame {frame_id_str} in sequence {sequence_id}.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Compare a densely sampled yaw cost curve against the local "
            "Gauss-Newton quadratic C(delta)=C0+g*delta+0.5*H*delta^2."
        )
    )
    parser.add_argument("--csv", type=Path, default=None, help="Yaw perturbation cost CSV.")
    parser.add_argument(
        "--results-csv",
        type=Path,
        default=None,
        help="Localization result CSV. Automatically process frames above the yaw-error threshold.",
    )
    parser.add_argument(
        "--csv-folder",
        type=Path,
        default=Path("perturbation_cost_tests/delaunay_nonlinear_test2"),
        help="Folder containing <frame>.csv when --csv is omitted.",
    )
    parser.add_argument(
        "--frame",
        type=str,
        default=None,
        help="Radar frame id. Used to locate <frame>.csv and compute GN terms automatically.",
    )
    parser.add_argument(
        "--sequence-id",
        type=str,
        default=None,
        help="Optional Boreas sequence id to search when computing GN terms.",
    )
    parser.add_argument(
        "--dof",
        choices=tuple(DOF_COLUMNS),
        default="yaw",
        help="Single perturbation DOF to analyze.",
    )
    parser.add_argument(
        "--center",
        "--center-yaw-deg",
        dest="center",
        type=float,
        default=None,
        help="Yaw perturbation, in degrees, where the GN linearization was computed.",
    )
    parser.add_argument("--C0", type=float, default=None, help="GN cost at the linearization point.")
    parser.add_argument("--g", type=float, default=None, help="GN yaw gradient dC/dyaw_rad.")
    parser.add_argument("--H", type=float, default=None, help="GN yaw Hessian approximation J_yaw^T J_yaw.")
    parser.add_argument(
        "--weights-path",
        type=Path,
        default=None,
        help="Model weights path. Defaults to the same weights as pipeline_fast.py.",
    )
    parser.add_argument("--device", type=str, default=None, help="Torch device for automatic linearization.")
    parser.add_argument("--fov-deg", type=float, default=6.0, help="Patch FOV in degrees.")
    parser.add_argument("--res-deg", type=float, default=0.1, help="Patch angular resolution in degrees.")
    parser.add_argument(
        "--geometry-backend",
        choices=["torch_mesh", "torch_frustum"],
        default="torch_mesh",
        help="Depth patch geometry backend.",
    )
    parser.add_argument(
        "--mesh-root",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "postprocessing" / "submap_meshes",
        help="Root directory containing cached per-submap meshes.",
    )
    parser.add_argument(
        "--linearization-batch-size",
        type=int,
        default=100,
        help="Batch size for automatic GN model JVP linearization.",
    )
    parser.add_argument(
        "--geometry-batch-size",
        type=int,
        default=400,
        help="Azimuth batch size for GPU frustum geometry.",
    )
    parser.add_argument(
        "--failure-threshold",
        "--failure-threshold-deg",
        dest="failure_threshold_deg",
        type=float,
        default=0.1,
        help="Process result rows with abs(final_<dof>) at or above this threshold.",
    )
    parser.add_argument(
        "--recompute-costs",
        action="store_true",
        help="Ignore existing sampled-cost CSVs and recompute nonlinear costs and GN terms.",
    )
    parser.add_argument("--global-min", "--global-min-deg", dest="global_min_deg", type=float, default=-3.0)
    parser.add_argument("--global-max", "--global-max-deg", dest="global_max_deg", type=float, default=3.0)
    parser.add_argument("--global-step", "--global-step-deg", dest="global_step_deg", type=float, default=0.5)
    parser.add_argument(
        "--local-window",
        "--local-window-deg",
        dest="local_window_deg",
        type=float,
        default=None,
        help=(
            "Densely sample and plot only this +/- yaw window around the "
            "linearization point. If omitted, dense sampling still uses "
            "+/-0.5 deg but the full global curve is plotted."
        ),
    )
    parser.add_argument("--local-step", "--local-step-deg", dest="local_step_deg", type=float, default=0.001)
    parser.add_argument(
        "--window",
        "--window-deg",
        dest="window_deg",
        type=float,
        default=None,
        help="Optional yaw window around center_yaw_deg to plot.",
    )
    parser.add_argument(
        "--num-quad",
        type=int,
        default=1000,
        help="Number of points used to draw the GN quadratic curve.",
    )
    parser.add_argument(
        "--quadratic-window",
        "--quadratic-window-deg",
        dest="quadratic_window_deg",
        type=float,
        default=0.5,
        help="Draw the GN quadratic only within this yaw distance of its linearization point.",
    )
    parser.add_argument(
        "--align-value-to-csv",
        action="store_true",
        help="Plot an additional quadratic shifted to intersect the sampled CSV curve at center_yaw_deg.",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Output PNG path. Defaults next to the CSV.",
    )
    parser.add_argument("--show", action="store_true", help="Show the plot interactively.")
    args = parser.parse_args()
    args.failure_threshold = args.failure_threshold_deg
    args.global_min = args.global_min_deg
    args.global_max = args.global_max_deg
    args.global_step = args.global_step_deg
    args.local_step = args.local_step_deg
    args.quadratic_window = args.quadratic_window_deg

    if args.geometry_batch_size < 1:
        raise ValueError("--geometry-batch-size must be at least 1.")
    if args.global_step_deg <= 0.0 or args.local_step_deg <= 0.0:
        raise ValueError("Sampling step sizes must be positive.")
    if args.global_min_deg >= args.global_max_deg:
        raise ValueError("--global-min-deg must be smaller than --global-max-deg.")
    if args.local_window_deg is not None and args.local_window_deg <= 0.0:
        raise ValueError("--local-window-deg must be positive.")
    if args.quadratic_window_deg <= 0.0:
        raise ValueError("--quadratic-window-deg must be positive.")

    if args.results_csv is not None:
        process_failed_frames(args)
        return

    if args.center is None:
        raise ValueError("--center is required unless --results-csv is used.")

    if args.csv is None:
        if args.frame is None:
            raise ValueError("Provide either --csv or --frame.")
        csv_path = (args.csv_folder / f"{args.frame}.csv").expanduser().resolve()
    else:
        csv_path = args.csv.expanduser().resolve()

    if args.C0 is None or args.g is None or args.H is None:
        if args.frame is None:
            raise ValueError("Automatic GN term computation requires --frame.")
        args.C0, args.g, args.H = compute_gn_quadratic_terms_from_frame(
            frame_id=args.frame,
            dof=args.dof,
            center_value=args.center,
            sequence_id=args.sequence_id,
            weights_path=None if args.weights_path is None else args.weights_path.expanduser().resolve(),
            fov_deg=args.fov_deg,
            res_deg=args.res_deg,
            linearization_batch_size=args.linearization_batch_size,
            device_name=args.device,
            geometry_backend=args.geometry_backend,
            mesh_root=args.mesh_root,
        )

    if args.output is None:
        output_dir = csv_path.parent / "GN_quadratic_curves"
        value_label = format_value_for_filename(args.center)
        output_path = output_dir / f"{csv_path.stem}_center_{args.dof}_{value_label}_{dof_unit(args.dof)}.png"
    else:
        output_path = args.output.expanduser().resolve()
    plot_dof_curve_and_quadratic(
        csv_path=csv_path,
        dof=args.dof,
        center_value=args.center,
        c0=args.C0,
        g=args.g,
        h=args.H,
        output_path=output_path,
        window=args.window_deg,
        quadratic_window=args.quadratic_window,
        num_quad=args.num_quad,
        align_value_to_csv=args.align_value_to_csv,
        show=args.show,
    )


if __name__ == "__main__":
    main()
