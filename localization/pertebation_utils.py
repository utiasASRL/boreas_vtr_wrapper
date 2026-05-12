import itertools
import numpy as np
from scipy.spatial.transform import Rotation as R


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

        perturbations.append({
            "name": name,
            "delta_T": make_delta_T(translation, rpy_deg),
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
        "x": np.linspace(-2.0, 2.0, 9),
        "y": np.linspace(-2.0, 2.0, 9),
        "z": [0.0],
    }

    rotation_offsets_deg = {
        "roll": [0.0],
        "pitch": [0.0],
        "yaw": np.linspace(-10.0, 10.0, 9),
    }

    perturbations = generate_delta_transforms(
        translation_offsets=translation_offsets,
        rotation_offsets_deg=rotation_offsets_deg,
        mode="axis",
        include_identity=True,
    )

    for perturb in perturbations:
        delta_T = perturb["delta_T"]

        # T_radar_enu_perturbed = delta_T @ T_radar_enu_gt

        # residual = compute_localization_residual(T_radar_enu_perturbed)

        # print(perturb["name"], residual)

        print(perturb["name"])
        print(delta_T)