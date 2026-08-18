import unittest
from unittest.mock import patch

import numpy as np

from localization_dfo.pipeline_dfo import calculate_imfil_scales, nearest_submap_idx


class PipelineHelpersTest(unittest.TestCase):
    def test_calculate_imfil_scales_for_optimizer_bounds(self):
        bounds = np.array(
            [[-0.3, 0.3]] * 3 + [[-np.deg2rad(3), np.deg2rad(3)]] * 3
        )
        self.assertEqual(calculate_imfil_scales(bounds), (4, 7))

    def test_nearest_submap_expands_window_at_boundary(self):
        candidates = [(None, None, idx) for idx in range(30)]
        with patch(
            "localization_dfo.pipeline_dfo.submap_distance",
            side_effect=lambda _query, candidate: abs(candidate - 17),
        ):
            self.assertEqual(nearest_submap_idx(None, candidates), 17)
            self.assertEqual(
                nearest_submap_idx(None, candidates, center_idx=10, radius=5), 17
            )


if __name__ == "__main__":
    unittest.main()
