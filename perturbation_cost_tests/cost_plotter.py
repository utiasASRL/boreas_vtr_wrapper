#!/usr/bin/env python3

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


DOFS = [
    ("dx", "x translation offset [m]"),
    ("dy", "y translation offset [m]"),
    ("dz", "z translation offset [m]"),
    ("roll_deg", "roll offset [deg]"),
    ("pitch_deg", "pitch offset [deg]"),
    ("yaw_deg", "yaw offset [deg]"),
]


def is_identity_row(df):
    """Return boolean mask for identity perturbation rows."""
    return (
        np.isclose(df["dx"], 0.0)
        & np.isclose(df["dy"], 0.0)
        & np.isclose(df["dz"], 0.0)
        & np.isclose(df["roll_deg"], 0.0)
        & np.isclose(df["pitch_deg"], 0.0)
        & np.isclose(df["yaw_deg"], 0.0)
    )


def is_single_axis_row(df, dof):
    """
    For mode='axis', select rows where only the requested DoF is nonzero,
    plus the identity row.
    """
    mask = is_identity_row(df)

    other_dofs = [col for col, _ in DOFS if col != dof]

    selected_axis_mask = ~np.isclose(df[dof], 0.0)

    for other in other_dofs:
        selected_axis_mask &= np.isclose(df[other], 0.0)

    return mask | selected_axis_mask


def load_csv(csv_path):
    required_cols = [
        "cost",
        "dx",
        "dy",
        "dz",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
    ]

    df = pd.read_csv(csv_path)

    missing = [col for col in required_cols if col not in df.columns]
    if missing:
        raise ValueError(f"{csv_path} is missing required columns: {missing}")

    return df


def plot_cost_vs_dof(csv_paths, dof, xlabel, output_dir, normalize=False, normalized_ymax=None, ignore_identity=False, show=False):
    plt.figure(figsize=(9, 6))

    plotted_any = False

    for csv_path in csv_paths:
        df = load_csv(csv_path)
        if ignore_identity:
            df = df[~is_identity_row(df)].copy()

        axis_df = df[is_single_axis_row(df, dof)].copy()

        if axis_df.empty:
            print(f"[WARN] No valid {dof} rows found in {csv_path.name}")
            continue

        axis_df = axis_df.sort_values(dof)

        x = axis_df[dof].to_numpy()
        y = axis_df["cost"].to_numpy()

        if normalize:
            min_cost = np.min(y)
            if not np.isclose(min_cost, 0.0):
                y = y / min_cost

        label = csv_path.stem

        plt.plot(x, y, linewidth=1.5, label=label)
        plotted_any = True

    plt.xlabel(xlabel)

    if normalize:
        plt.ylabel("cost / minimum cost")
        if normalized_ymax is not None:
            plt.ylim(top=normalized_ymax)
    else:
        plt.ylabel("sum of squared error cost")

    plt.title(f"Cost vs {dof}")
    plt.grid(True)

    # if plotted_any:
    #     plt.legend(fontsize=8)

    plt.tight_layout()

    output_path = output_dir / f"cost_vs_{dof}.png"
    plt.savefig(output_path, dpi=200)
    print(f"Saved: {output_path}")

    if show:
        plt.show()

    plt.close()


def main():
    parser = argparse.ArgumentParser(
        description="Plot cost vs perturbation offset for axis-mode perturbation CSVs."
    )

    parser.add_argument(
        "csv_folder",
        type=str,
        help="Folder containing perturbation cost CSV files.",
    )

    parser.add_argument(
        "-o",
        "--output-dir",
        type=str,
        default=None,
        help="Folder to save plots. Defaults to <csv_folder>/cost_plots.",
    )

    parser.add_argument(
        "--normalize",
        action="store_true",
        help="Normalize each curve by its own minimum cost.",
    )
    parser.add_argument(
        "--normalized-ymax",
        type=float,
        default=None,
        help="Maximum y limit for normalized cost plots.",
    )

    parser.add_argument(
        "--ignore-identity",
        action="store_true",
        help="Exclude the identity row from each plot.",
    )

    parser.add_argument(
        "--show",
        action="store_true",
        help="Show plots interactively.",
    )

    args = parser.parse_args()

    csv_folder = Path(args.csv_folder).expanduser().resolve()

    if not csv_folder.exists():
        raise FileNotFoundError(f"CSV folder does not exist: {csv_folder}")

    if args.output_dir is None:
        output_dir = csv_folder / "cost_plots"
    else:
        output_dir = Path(args.output_dir).expanduser().resolve()

    output_dir.mkdir(parents=True, exist_ok=True)

    csv_paths = sorted(csv_folder.glob("*.csv"))

    if len(csv_paths) == 0:
        raise FileNotFoundError(f"No CSV files found in: {csv_folder}")

    print(f"Found {len(csv_paths)} CSV files.")

    for dof, xlabel in DOFS:
        plot_cost_vs_dof(
            csv_paths=csv_paths,
            dof=dof,
            xlabel=xlabel,
            output_dir=output_dir,
            normalize=args.normalize,
            normalized_ymax=args.normalized_ymax,
            ignore_identity=args.ignore_identity,
            show=args.show,
        )


if __name__ == "__main__":
    main()
