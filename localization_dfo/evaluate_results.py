import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


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


def load_errors(csv_path):
    rows = []
    with csv_path.open(newline="") as f:
        for row in csv.DictReader(f):
            try:
                rows.append(
                    {
                        "frame_id": row.get("frame_id", ""),
                        **{f"initial_{name}": float(row[f"initial_{name}"]) for name, _ in DOFS},
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


def write_plots(rows, output_dir):
    x = range(len(rows))
    for name, label in DOFS:
        initial = [row[f"initial_{name}"] for row in rows]
        final = [row[f"final_{name}"] for row in rows]

        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(x, initial, label="initial", linewidth=1.5)
        ax.plot(x, final, label="final", linewidth=1.5)
        ax.axhline(0.0, color="black", linewidth=0.8)
        ax.set_xlabel("row")
        ax.set_ylabel(f"error {label}")
        ax.set_title(f"{name} error")
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(output_dir / f"{name}_error.png", dpi=150)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Evaluate DFO localization result errors.")
    parser.add_argument("--loc-sequence", required=True)
    parser.add_argument("--csv", required=True, help="CSV filename under results/<loc-sequence>/")
    args = parser.parse_args()

    results_dir = Path(__file__).resolve().parent / "results" / args.loc_sequence
    csv_path = results_dir / args.csv
    output_dir = results_dir / Path(args.csv).stem
    output_dir.mkdir(parents=True, exist_ok=True)

    rows = load_errors(csv_path)
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
    write_plots(rows, output_dir)
    print(f"\nSaved results to {output_dir}")


if __name__ == "__main__":
    main()
