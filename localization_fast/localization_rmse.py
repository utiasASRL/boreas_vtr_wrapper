#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np


ERROR_COLUMNS = [
    ("final_x_m", "x [m]"),
    ("final_y_m", "y [m]"),
    ("final_z_m", "z [m]"),
    ("final_roll_deg", "roll [deg]"),
    ("final_pitch_deg", "pitch [deg]"),
    ("final_yaw_deg", "yaw [deg]"),
]


def discover_csv_paths(inputs):
    csv_paths = []
    for input_path in inputs:
        path = input_path.expanduser().resolve()
        if path.is_dir():
            csv_paths.extend(sorted(path.rglob("*.csv")))
        elif path.is_file():
            csv_paths.append(path)
        else:
            raise FileNotFoundError(f"Input does not exist: {path}")

    unique_paths = []
    seen = set()
    for path in csv_paths:
        if path not in seen:
            unique_paths.append(path)
            seen.add(path)
    if not unique_paths:
        raise FileNotFoundError("No result CSV files were found.")
    return unique_paths


def load_result_rows(csv_path):
    with csv_path.open(newline="") as f:
        reader = csv.DictReader(f)
        missing = [column for column, _ in ERROR_COLUMNS if column not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f"{csv_path} is missing columns: {missing}")

        rows = []
        for row in reader:
            try:
                values = {column: float(row[column]) for column, _ in ERROR_COLUMNS}
            except (TypeError, ValueError):
                continue
            if all(np.isfinite(value) for value in values.values()):
                rows.append(values)
    return rows


def compute_rmse(rows):
    if not rows:
        raise ValueError("No finite localization result rows were available.")
    return {
        column: float(np.sqrt(np.mean([row[column] ** 2 for row in rows])))
        for column, _ in ERROR_COLUMNS
    }


def print_summary(label, rows):
    rmse = compute_rmse(rows)
    print(f"\n{label}")
    print(f"frames: {len(rows)}")
    for column, display_name in ERROR_COLUMNS:
        print(f"  {display_name:12s} RMSE: {rmse[column]:.9e}")


def main():
    parser = argparse.ArgumentParser(
        description="Compute component-wise localization RMSE from one or more result CSVs."
    )
    parser.add_argument(
        "inputs",
        nargs="+",
        type=Path,
        help="Result CSV files or directories containing result CSVs.",
    )
    parser.add_argument(
        "--combined-only",
        action="store_true",
        help="Only print the combined summary when multiple CSVs are supplied.",
    )
    args = parser.parse_args()

    csv_paths = discover_csv_paths(args.inputs)
    all_rows = []

    for csv_path in csv_paths:
        rows = load_result_rows(csv_path)
        if not rows:
            print(f"[WARN] No finite rows in {csv_path}")
            continue
        all_rows.extend(rows)
        if not args.combined_only:
            print_summary(str(csv_path), rows)

    if not all_rows:
        raise ValueError("No finite localization result rows were found.")

    if len(csv_paths) > 1 or args.combined_only:
        print_summary("Combined results", all_rows)


if __name__ == "__main__":
    main()
