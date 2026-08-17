import csv
import tempfile
import unittest
from pathlib import Path

import numpy as np

from localization_dfo.evaluate_results import DOFS, load_errors
from localization_dfo.pipeline_dfo import apply_pose_gate, pose_gate_diagnostics


class PoseGatingTest(unittest.TestCase):
    def test_exact_azimuth_mismatch_and_state_recovery(self):
        observed = np.zeros((4, 4))
        observed[0, :] = 2
        observed[1:, :] = 0.5 / 3
        initial = np.ones_like(observed)
        candidate = np.zeros_like(observed)
        candidate[1:, :3] = 1

        diagnostics, jumps, prediction_failures = pose_gate_diagnostics(
            observed, initial, candidate, np.zeros(6)
        )
        self.assertEqual(diagnostics["candidate_prediction_active_fraction"], 0.75)
        self.assertAlmostEqual(
            diagnostics["candidate_missing_observed_evidence_fraction"], 0.8
        )
        scaled, _, _ = pose_gate_diagnostics(
            observed * 7, initial, candidate, np.zeros(6)
        )
        self.assertAlmostEqual(
            scaled["candidate_missing_observed_evidence_fraction"], 0.8
        )
        self.assertEqual(prediction_failures, ["candidate_observed_azimuth_mismatch"])
        self.assertEqual(jumps, [])

        state, rejected_count, healthy_count = "tracking", 0, 0
        for _ in range(3):
            rejected, _, state, rejected_count, healthy_count = apply_pose_gate(
                state, rejected_count, healthy_count, ["translation_jump_x"], []
            )
            self.assertTrue(rejected)
        self.assertEqual(state, "recovery")

        rejected, _, state, rejected_count, healthy_count = apply_pose_gate(
            state, rejected_count, healthy_count, ["translation_jump_x"], []
        )
        self.assertFalse(rejected)
        for _ in range(3):
            rejected, _, state, rejected_count, healthy_count = apply_pose_gate(
                state, rejected_count, healthy_count, [], []
            )
        self.assertEqual(state, "tracking")

    def test_evaluate_results_ignores_gate_columns(self):
        rows = []
        for frame_id, rejected in (("1", "False"), ("2", "True")):
            row = {"frame_id": frame_id, "optimizer_rejected": rejected}
            for name, _ in DOFS:
                row[f"initial_{name}"] = "1"
                row[f"final_{name}"] = "0"
            rows.append(row)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "results.csv"
            with path.open("w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=rows[0])
                writer.writeheader()
                writer.writerows(rows)
            loaded = load_errors(path)
            self.assertEqual(len(loaded), 2)
            self.assertTrue(all(row["final_x_m"] == 0.0 for row in loaded))


if __name__ == "__main__":
    unittest.main()
