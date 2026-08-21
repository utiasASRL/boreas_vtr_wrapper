import unittest
from unittest.mock import patch

import numpy as np

from localization_dfo.io_utils import cen_filter_2d
from localization_dfo.pipeline_dfo import calculate_imfil_scales, nearest_submap_idx


class PipelineHelpersTest(unittest.TestCase):
    def test_cropped_cen_matches_full_output(self):
        polar = np.random.default_rng(0).normal(size=(4, 128))
        full = cen_filter_2d(polar, sigma_gauss=3.0)
        cropped = cen_filter_2d(polar, sigma_gauss=3.0, output_width=47)

        np.testing.assert_array_equal(cropped, full[:, :47])

    def test_calculate_imfil_scales_for_optimizer_bounds(self):
        bounds = np.array(
            [[-0.3, 0.3]] * 3 + [[-np.deg2rad(3), np.deg2rad(3)]] * 3
        )
        self.assertEqual(calculate_imfil_scales(bounds), (4, 7))

    def test_nearest_submap_circular_window_and_expansion(self):
        candidates = [(None, None, idx) for idx in range(100)]

        def nearest(center_idx, target):
            with patch(
                "localization_dfo.pipeline_dfo.submap_distance",
                side_effect=lambda _query, candidate: abs(candidate - target),
            ):
                return nearest_submap_idx(None, candidates, center_idx=center_idx)

        with patch(
            "localization_dfo.pipeline_dfo.submap_distance",
            side_effect=lambda _query, candidate: abs(candidate - 90),
        ):
            self.assertEqual(nearest_submap_idx(None, candidates), 90)

        self.assertEqual(nearest(99, 2), 2)
        self.assertEqual(nearest(25, 42), 42)
        self.assertEqual(nearest(50, 90), 70)


if __name__ == "__main__":
    unittest.main()
