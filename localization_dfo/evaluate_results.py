import argparse
import csv
import math
import os
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pyboreas.utils.odometry import read_traj_file_gt
from scipy.spatial.transform import Rotation as R


DOFS = [
    ("x_m", "x [m]"),
    ("y_m", "y [m]"),
    ("z_m", "z [m]"),
    ("roll_deg", "roll [deg]"),
    ("pitch_deg", "pitch [deg]"),
    ("yaw_deg", "yaw [deg]"),
]
TRANSLATION_DOFS = ["x_m", "y_m", "z_m"]
ROTATION_DOFS = ["roll_deg", "pitch_deg", "yaw_deg"]
TRANSLATION_THRESHOLDS_M = [0.05, 0.10, 0.15, 0.20]
ROTATION_THRESHOLDS_DEG = [0.25, 0.5, 1.0]
STATE_COLORS = {"tracking": "tab:blue", "recovery": "tab:orange"}


def load_errors(csv_path):
    rows = []
    with csv_path.open(newline="") as f:
        for row in csv.DictReader(f):
            try:
                rows.append(
                    {
                        "frame_id": row.get("frame_id", ""),
                        "sequence_id": row.get("sequence_id", ""),
                        "localization_state": row.get("localization_state", "tracking")
                        or "tracking",
                        **{f"final_{name}": float(row[f"final_{name}"]) for name, _ in DOFS},
                    }
                )
            except (KeyError, TypeError, ValueError):
                continue
    if not rows:
        raise ValueError(f"No valid error rows found in {csv_path}.")
    return rows


def stats(values):
    n = len(values)
    return {
        "rmse": math.sqrt(sum(v * v for v in values) / n),
        "mean_abs": sum(abs(v) for v in values) / n,
        "max_abs": max(abs(v) for v in values),
        "mean": sum(values) / n,
    }


def write_stats(rows, output_csv):
    stat_rows = []
    for name, _ in DOFS:
        values = [row[f"final_{name}"] for row in rows]
        stat_rows.append({"dof": name, **stats(values)})

    with output_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["dof", "rmse", "mean_abs", "max_abs", "mean"])
        writer.writeheader()
        writer.writerows(stat_rows)

    print(f"{'dof':<10} {'rmse':>12} {'mean_abs':>12} {'max_abs':>12} {'mean':>12}")
    for row in stat_rows:
        print(
            f"{row['dof']:<10} {row['rmse']:12.6g} {row['mean_abs']:12.6g} "
            f"{row['max_abs']:12.6g} {row['mean']:12.6g}"
        )


def write_threshold_table(rows, dofs, thresholds, unit, output_csv):
    fieldnames = ["dof", *[f"pct_abs_error_lt_{threshold:g}_{unit}" for threshold in thresholds]]
    table_rows = []
    for name in dofs:
        values = [abs(row[f"final_{name}"]) for row in rows]
        table_rows.append(
            {
                "dof": name,
                **{
                    fieldnames[idx + 1]: 100.0 * sum(value < threshold for value in values) / len(values)
                    for idx, threshold in enumerate(thresholds)
                },
            }
        )

    with output_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(table_rows)

    print()
    print("final absolute error percent below threshold")
    print(" ".join([f"{'dof':<10}", *[f"<{threshold:g} {unit:>3}" for threshold in thresholds]]))
    for row in table_rows:
        print(" ".join([f"{row['dof']:<10}", *[f"{row[field]:8.2f}" for field in fieldnames[1:]]]))


def single_frame_odometry_errors(frame_transforms, gt_poses, frame_indices):
    errors = {name: [] for name, _ in DOFS}
    for frame_idx in frame_indices:
        if frame_idx == 0:
            xyz = rpy = np.full(3, np.nan)
        else:
            gt_delta = gt_poses[frame_idx] @ np.linalg.inv(gt_poses[frame_idx - 1])
            error = frame_transforms[frame_idx] @ np.linalg.inv(gt_delta)
            xyz = error[:3, 3]
            rpy = R.from_matrix(error[:3, :3]).as_euler("xyz", degrees=True)
        for (name, _), value in zip(DOFS, (*xyz, *rpy)):
            errors[name].append(float(value))
    return errors


def load_odometry_errors(rows, odometry_path, gt_path):
    gt_poses, gt_times = read_traj_file_gt(str(gt_path), np.eye(4), dim=3)
    gt_poses = np.asarray(gt_poses)
    gt_times = np.asarray(gt_times, dtype=np.int64)
    with np.load(odometry_path) as data:
        frame_times = data["frame_timestamps_us"]
        frame_transforms = data["frame_transforms"]
        convention = data["odom_transform_convention"].item()

    if convention != "right":
        raise ValueError("Odometry must use the right transform convention.")
    if not np.array_equal(frame_times, gt_times):
        raise ValueError("Odometry and GT radar timestamps differ.")
    if frame_transforms.shape != (len(frame_times), 4, 4):
        raise ValueError(f"Invalid frame transform shape: {frame_transforms.shape}")

    time_to_index = {int(timestamp): idx for idx, timestamp in enumerate(frame_times)}
    try:
        frame_indices = np.asarray(
            [time_to_index[int(row["frame_id"])] for row in rows], dtype=int
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("Result frame IDs do not match odometry timestamps.") from error
    if np.any(np.diff(frame_indices) <= 0):
        raise ValueError("Result frame IDs must be strictly increasing.")
    return frame_indices, single_frame_odometry_errors(
        frame_transforms, gt_poses, frame_indices
    )


def write_plots(rows, frame_indices, odometry_errors, output_dir):
    states = np.asarray([row["localization_state"].strip().lower() for row in rows])
    unknown_states = set(states).difference(STATE_COLORS)
    if unknown_states:
        raise ValueError(f"Unknown localization states: {sorted(unknown_states)}")

    for name, label in DOFS:
        final = np.asarray([row[f"final_{name}"] for row in rows])
        fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        for state, color in STATE_COLORS.items():
            axes[0].plot(
                frame_indices,
                np.ma.masked_where(states != state, final),
                color=color,
                label=state,
                linewidth=1.5,
                marker=".",
                markersize=3,
            )
        axes[0].axhline(0.0, color="black", linewidth=0.8)
        axes[0].set_ylabel(f"final error {label}")
        axes[0].set_title(f"{name} localization and single-frame odometry error")
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(
            frame_indices,
            odometry_errors[name],
            color="tab:purple",
            linewidth=1.2,
        )
        axes[1].axhline(0.0, color="black", linewidth=0.8)
        axes[1].set_xlabel("radar frame index")
        axes[1].set_ylabel(f"odometry error {label}")
        axes[1].grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(output_dir / f"{name}_error.png", dpi=150)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Evaluate DFO localization result errors.")
    parser.add_argument("--csv", required=True, type=Path, help="Path to a DFO result CSV.")
    parser.add_argument("--odometry", type=Path, help="Override azimuth_odometry.npz path.")
    parser.add_argument("--data-root", type=Path, help="Override VTRRDATA Boreas root.")
    args = parser.parse_args()

    csv_path = args.csv
    if not csv_path.is_file():
        raise FileNotFoundError(f"Result CSV does not exist: {csv_path}")
    output_dir = csv_path.parent / "evaluate_results"
    output_dir.mkdir(parents=True, exist_ok=True)

    rows = load_errors(csv_path)
    sequence_ids = {row["sequence_id"] for row in rows}
    if len(sequence_ids) != 1 or not next(iter(sequence_ids)):
        raise ValueError("Result CSV must contain exactly one sequence_id.")
    sequence_id = next(iter(sequence_ids))
    wrapper_dir = Path(os.getenv("VTRROOT", Path(__file__).resolve().parents[1]))
    data_root = args.data_root or os.getenv("VTRRDATA")
    if data_root is None:
        raise RuntimeError("VTRRDATA must be set or --data-root must be provided.")
    data_root = Path(data_root)
    odometry_path = args.odometry or (
        wrapper_dir
        / "external"
        / "wheel_odometry"
        / "output"
        / sequence_id
        / "odometry_result"
        / "azimuth_odometry.npz"
    )
    gt_path = data_root / sequence_id / "applanix" / "radar_poses.csv"
    for label, path in (("Odometry", odometry_path), ("GT", gt_path)):
        if not path.is_file():
            raise FileNotFoundError(f"{label} file does not exist: {path}")
    frame_indices, odometry_errors = load_odometry_errors(
        rows, odometry_path, gt_path
    )
    write_stats(rows, output_dir / "error_stats.csv")
    write_threshold_table(
        rows,
        TRANSLATION_DOFS,
        TRANSLATION_THRESHOLDS_M,
        "m",
        output_dir / "translation_threshold_stats.csv",
    )
    write_threshold_table(
        rows,
        ROTATION_DOFS,
        ROTATION_THRESHOLDS_DEG,
        "deg",
        output_dir / "rotation_threshold_stats.csv",
    )
    write_plots(rows, frame_indices, odometry_errors, output_dir)
    print(f"\nSaved results to {output_dir}")


if __name__ == "__main__":
    main()
