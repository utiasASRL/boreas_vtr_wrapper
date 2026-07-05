import numpy as np
from scipy.spatial.transform import Rotation as R


def make_delta_T(translation=None, rpy_deg=None):
    translation = np.zeros(3) if translation is None else np.asarray(translation, dtype=float)
    rpy_deg = np.zeros(3) if rpy_deg is None else np.asarray(rpy_deg, dtype=float)
    delta_T = np.eye(4)
    delta_T[:3, :3] = R.from_euler("xyz", rpy_deg, degrees=True).as_matrix()
    delta_T[:3, 3] = translation
    return delta_T
