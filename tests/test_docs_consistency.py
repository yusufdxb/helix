"""Lightweight docs/results consistency check.

Verifies that key numeric claims in RESULTS.md match the authoritative
JSON artifacts in results/.  This catches the class of regression where
docs are updated but JSON is not (or vice versa).
"""

import json
import os
import re
import unittest

ROOT = os.path.join(os.path.dirname(__file__), os.pardir)
RESULTS_DIR = os.path.join(ROOT, "results")


def _load_json(name: str) -> dict:
    path = os.path.join(RESULTS_DIR, name)
    with open(path) as f:
        return json.load(f)


def _read_results_md() -> str:
    with open(os.path.join(ROOT, "RESULTS.md")) as f:
        return f.read()


class TestOverheadNumbers(unittest.TestCase):
    """Peak Z-score and consecutive violations vs JSON."""

    def setUp(self):
        self.data = _load_json("helix_overhead_with_adapter.json")
        self.md = _read_results_md()

    def test_peak_zscore_in_docs(self):
        # Extract the highest Z-score from fault details
        zscores = []
        for evt in self.data["fault_details"]:
            m = re.search(r"Z-score ([\d.]+)", evt["detail"])
            if m:
                zscores.append(float(m.group(1)))
        peak = max(zscores)
        # Docs must contain this value (5.52), not the old 146.91
        self.assertIn(f"{peak}", self.md,
                      f"Peak Z-score {peak} not found in RESULTS.md")
        self.assertNotIn("146.91", self.md,
                         "Stale Z-score 146.91 still present in RESULTS.md")

    def test_consecutive_violations_in_docs(self):
        for evt in self.data["fault_details"]:
            self.assertIn("3 consecutive samples", evt["detail"])
        self.assertNotIn("Consecutive violations | 6", self.md,
                         "Stale 'Consecutive violations 6' still in RESULTS.md")


class TestLogParserThroughput(unittest.TestCase):
    """Log parser throughput vs JSON."""

    def setUp(self):
        self.data = _load_json("log_parser_results.json")
        self.md = _read_results_md()

    def test_throughput_present(self):
        mps = self.data["throughput"]["messages_per_sec"]
        # Should find ~777,000 or 777029 in the doc
        self.assertTrue(
            "777" in self.md,
            f"Log parser throughput {mps} not referenced in RESULTS.md"
        )

    def test_elapsed_present(self):
        elapsed = self.data["throughput"]["elapsed_sec"]
        self.assertIn(str(elapsed)[:5], self.md)  # "0.012" at minimum


class TestLaplaceFPR(unittest.TestCase):
    """Laplace Z=2.0, K=12 FPR must be 0.5%, not 0.0%."""

    def setUp(self):
        self.data = _load_json("realistic_anomaly_results.json")
        self.md = _read_results_md()

    def test_fpr_at_z2_k12(self):
        for entry in self.data["scenarios"]["1_laplace_noise"]["results"]:
            if entry["zscore_threshold"] == 2.0 and entry["spike_K"] == 12.0:
                self.assertEqual(entry["fpr"], 0.005)
                break
        else:
            self.fail("Z=2.0 K=12 entry not found in JSON")
        # The doc table row for Z=2.0 should contain "0.5%" not "0.0%"
        # Find the Z=2.0 row
        z2_line = [
            line for line in self.md.split("\n")
            if line.strip().startswith("| 2.0")
        ]
        self.assertTrue(len(z2_line) > 0, "Z=2.0 row not found in RESULTS.md")
        self.assertIn("0.5%", z2_line[0],
                      f"FPR should be 0.5% in Z=2.0 row, got: {z2_line[0]}")


class TestAttachabilityScore(unittest.TestCase):
    """Attachability with_adapters must not exceed 1.0."""

    def setUp(self):
        self.data = _load_json("attachability_matrix.json")

    def test_with_adapters_bounded(self):
        score = self.data["scores"]["with_adapters"]
        self.assertLessEqual(score, 1.0)
        self.assertGreaterEqual(score, 0.0)


class TestBagRateDurations(unittest.TestCase):
    """Topic durations must match bag_rate_analysis_single.json."""

    def setUp(self):
        self.data = _load_json("bag_rate_analysis_single.json")
        self.md = _read_results_md()

    def test_robot_pose_duration(self):
        dur = self.data["results"]["/utlidar/robot_pose"]["duration_sec"]
        self.assertIn(str(dur), self.md)

    def test_gnss_duration(self):
        dur = self.data["results"]["/gnss"]["duration_sec"]
        self.assertIn(str(dur), self.md)


if __name__ == "__main__":
    unittest.main()
