import itertools
import numpy as np
from scipy.spatial.transform import Rotation as R

import csv
import os
import torch

def write_to_csv(csv_path, perturb, cost):
    fieldnames = [
        "name",
        "cost",
        "dx",
        "dy",
        "dz",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
    ]

    file_exists = os.path.exists(csv_path)

    translation = perturb["translation"]
    rpy_deg = perturb["rotation_rpy_deg"]

    with open(csv_path, mode="a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)

        if not file_exists:
            writer.writeheader()

        writer.writerow({
            "name": perturb["name"],
            "cost": cost,
            "dx": translation[0],
            "dy": translation[1],
            "dz": translation[2],
            "roll_deg": rpy_deg[0],
            "pitch_deg": rpy_deg[1],
            "yaw_deg": rpy_deg[2],
        })

        f.flush()


def depth_to_waveform(nonzero_depths, resolution=0.04381, bins=6848):
    waveform = np.zeros(bins)
    
    if nonzero_depths.size == 0:
        return waveform

    # Divide by resolution, round to the nearest whole number, and subtract 1
    indices = np.round(nonzero_depths / resolution).astype(np.int64)
    
    # Filter out out-of-bounds indices 
    # (including depths < 0.5 * resolution which would become index -1)
    valid_mask = (indices >= 0) & (indices < bins)
    valid_indices = indices[valid_mask]
    # print(valid_indices)
    
    # Assign 0.5 to the valid bins
    waveform[valid_indices] = 1.0
    
    return waveform

def get_gaussian_kernel(window_size=25, sigma=3.0, dtype=np.float32):
    """
    Creates a 1D Gaussian kernel matching the PyTorch version.
    Kernel peak is normalized to 1.0, not sum-normalized.
    """
    x = np.arange(window_size, dtype=dtype)
    x = x - (window_size // 2)

    kernel = np.exp(-(x ** 2) / (2 * sigma ** 2))
    kernel = kernel / kernel.max()

    return kernel.astype(dtype)


# def depth_to_waveform(
#     nonzero_depths,
#     resolution=0.04381,
#     bins=6848,
#     window_size=25,
#     sigma=3.0,
# ):
#     """
#     NumPy version of the PyTorch depth_to_waveform function.

#     Converts depths to waveform bins, sets occupied bins to 1.0,
#     then applies Gaussian smoothing with peak-normalized kernel.
#     """
#     nonzero_depths = np.asarray(nonzero_depths)

#     waveform = np.zeros(bins, dtype=np.float32)

#     if nonzero_depths.size == 0:
#         return waveform

#     indices = np.round(nonzero_depths / resolution).astype(np.int64)

#     valid_mask = (indices >= 0) & (indices < bins)
#     valid_indices = indices[valid_mask]

#     waveform[valid_indices] = 1.0

#     kernel = get_gaussian_kernel(
#         window_size=window_size,
#         sigma=sigma,
#         dtype=np.float32,
#     )

#     pad = window_size // 2

#     # Match PyTorch F.conv1d(..., padding=pad)
#     padded_waveform = np.pad(
#         waveform,
#         pad_width=pad,
#         mode="constant",
#         constant_values=0.0,
#     )

#     smoothed_waveform = np.convolve(
#         padded_waveform,
#         kernel,
#         mode="valid",
#     )

#     smoothed_waveform = np.clip(smoothed_waveform, None, 1.0)

#     return smoothed_waveform.astype(np.float32)


def make_delta_T(translation=None, rpy_deg=None):
    """
    Build a 4x4 SE(3) perturbation matrix.

    translation: [dx, dy, dz]
    rpy_deg: [roll, pitch, yaw] in degrees
    """
    if translation is None:
        translation = np.zeros(3)

    if rpy_deg is None:
        rpy_deg = np.zeros(3)

    translation = np.asarray(translation, dtype=float)
    rpy_deg = np.asarray(rpy_deg, dtype=float)

    delta_T = np.eye(4)
    delta_T[:3, :3] = R.from_euler("xyz", rpy_deg, degrees=True).as_matrix()
    delta_T[:3, 3] = translation

    return delta_T


def generate_delta_transforms(
    translation_offsets=None,
    rotation_offsets_deg=None,
    mode="axis",
    include_identity=True,
):
    """
    Generate delta_T perturbations for SE(3) pose testing.

    Intended usage for T_radar_enu:

        T_radar_enu_pert = delta_T @ T_radar_enu_gt

    assuming delta_T is expressed in the radar frame.

    Parameters
    ----------
    translation_offsets:
        Dict with keys "x", "y", "z", values are lists of offsets in meters.

        Example:
            {
                "x": [-2, -1, 0, 1, 2],
                "y": [-2, -1, 0, 1, 2],
                "z": [0],
            }

    rotation_offsets_deg:
        Dict with keys "roll", "pitch", "yaw", values are lists of offsets in degrees.

        Example:
            {
                "roll": [0],
                "pitch": [0],
                "yaw": [-10, -5, 0, 5, 10],
            }

    mode:
        "axis" means perturb one dimension at a time.
        "grid" means use all combinations.

    include_identity:
        Whether to include the zero perturbation.

    Returns
    -------
    perturbations:
        List of dictionaries. Each dictionary contains:
            - name
            - delta_T
            - translation
            - rotation_rpy_deg
    """

    if translation_offsets is None:
        translation_offsets = {
            "x": [0.0],
            "y": [0.0],
            "z": [0.0],
        }

    if rotation_offsets_deg is None:
        rotation_offsets_deg = {
            "roll": [0.0],
            "pitch": [0.0],
            "yaw": [0.0],
        }

    txs = translation_offsets.get("x", [0.0])
    tys = translation_offsets.get("y", [0.0])
    tzs = translation_offsets.get("z", [0.0])

    rolls = rotation_offsets_deg.get("roll", [0.0])
    pitches = rotation_offsets_deg.get("pitch", [0.0])
    yaws = rotation_offsets_deg.get("yaw", [0.0])

    perturbations = []

    def add_perturbation(name, translation, rpy_deg):
        translation = np.asarray(translation, dtype=float)
        rpy_deg = np.asarray(rpy_deg, dtype=float)

        is_identity = (
            np.allclose(translation, 0.0)
            and np.allclose(rpy_deg, 0.0)
        )

        if is_identity and not include_identity:
            return

        # Pose of the offset frame with respect to the radar frame.
        # This means: coordinates in offset frame -> coordinates in radar frame.
        T_radar_offset = make_delta_T(translation, rpy_deg)

        # This is the transform you actually apply to radar-frame points
        # to express them in the offset frame.
        T_offset_radar = np.linalg.inv(T_radar_offset)

        perturbations.append({
            "name": name,
            "delta_T": T_offset_radar,
            "translation": translation, 
            "rotation_rpy_deg": rpy_deg,
        })

    if mode == "axis":
        if include_identity:
            add_perturbation(
                name="identity",
                translation=[0.0, 0.0, 0.0],
                rpy_deg=[0.0, 0.0, 0.0],
            )

        for dx in txs:
            if np.isclose(dx, 0.0):
                continue
            add_perturbation(
                name=f"dx_{dx:.3f}m",
                translation=[dx, 0.0, 0.0],
                rpy_deg=[0.0, 0.0, 0.0],
            )

        for dy in tys:
            if np.isclose(dy, 0.0):
                continue
            add_perturbation(
                name=f"dy_{dy:.3f}m",
                translation=[0.0, dy, 0.0],
                rpy_deg=[0.0, 0.0, 0.0],
            )

        for dz in tzs:
            if np.isclose(dz, 0.0):
                continue
            add_perturbation(
                name=f"dz_{dz:.3f}m",
                translation=[0.0, 0.0, dz],
                rpy_deg=[0.0, 0.0, 0.0],
            )

        for roll in rolls:
            if np.isclose(roll, 0.0):
                continue
            add_perturbation(
                name=f"roll_{roll:.3f}deg",
                translation=[0.0, 0.0, 0.0],
                rpy_deg=[roll, 0.0, 0.0],
            )

        for pitch in pitches:
            if np.isclose(pitch, 0.0):
                continue
            add_perturbation(
                name=f"pitch_{pitch:.3f}deg",
                translation=[0.0, 0.0, 0.0],
                rpy_deg=[0.0, pitch, 0.0],
            )

        for yaw in yaws:
            if np.isclose(yaw, 0.0):
                continue
            add_perturbation(
                name=f"yaw_{yaw:.3f}deg",
                translation=[0.0, 0.0, 0.0],
                rpy_deg=[0.0, 0.0, yaw],
            )

    elif mode == "grid":
        if include_identity:
            add_perturbation(
                name="identity",
                translation=[0.0, 0.0, 0.0],
                rpy_deg=[0.0, 0.0, 0.0],
            )
        for dx, dy, dz, roll, pitch, yaw in itertools.product(
            txs, tys, tzs, rolls, pitches, yaws
        ):
            translation = [dx, dy, dz]
            rpy_deg = [roll, pitch, yaw]

            name = (
                f"dx_{dx:.3f}_dy_{dy:.3f}_dz_{dz:.3f}_"
                f"roll_{roll:.3f}_pitch_{pitch:.3f}_yaw_{yaw:.3f}"
            )

            add_perturbation(name, translation, rpy_deg)

    else:
        raise ValueError(f"Unknown mode: {mode}. Use 'axis' or 'grid'.")

    return perturbations


if __name__ == "__main__":
    translation_offsets = {
        # "x": np.linspace(-2.0, 2.0, 9),
        # "y": np.linspace(-2.0, 2.0, 9),
        "x": [5.0],
        "y": [5.0],
        "z": [0.0],
    }

    rotation_offsets_deg = {
        "roll": [0.0],
        "pitch": [0.0],
        # "yaw": [30.0],
        "yaw": np.linspace(-10.0, 10.0, 3),
    }

    perturbations = generate_delta_transforms(
        translation_offsets=translation_offsets,
        rotation_offsets_deg=rotation_offsets_deg,
        mode="grid",
        include_identity=True,
    )

    for perturb in perturbations:
        delta_T = perturb["delta_T"]

        # T_radar_enu_perturbed = delta_T @ T_radar_enu_gt

        # residual = compute_localization_residual(T_radar_enu_perturbed)

        # print(perturb["name"], residual)

        print(perturb["name"])
        print(delta_T)