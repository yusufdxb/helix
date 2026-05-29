"""Lightweight docs/results consistency check.

Verifies that key numeric claims in RESULTS.md match the authoritative
JSON artifacts in results/.  This catches the class of regression where
docs are updated but JSON is not (or vice versa).
"""

import glob
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


class TestNoOrphanSurveyReference(unittest.TestCase):
    """No tracked doc or source may cite the never-committed local-LLM
    survey note.

    Guards a specific orphan-reference regression: `llm_explainer.py` and
    `LLAMA_SERVER_JETSON_SETUP.md` both cited
    `notes/local_llm_survey_2026-04-18.md` as a source of record, but that
    file was never committed and does not exist anywhere in the tree
    (verified: `find . -name local_llm_survey_2026-04-18.md` returns
    nothing). The model-selection content it described now lives inside
    `LLAMA_SERVER_JETSON_SETUP.md` itself, so the citations must point
    there instead of at a phantom file.

    This test is intentionally narrow: it does not flag session-relative
    `hardware_eval_*/notes/*.md` references (those are real artifacts in
    the per-session dirs) nor forward references to unrun protocols.
    """

    ORPHAN = "local_llm_survey_2026-04-18.md"
    SKIP_DIRS = ("/build/", "/install/", "/log/", "/hardware_eval_",
                 "/.git/", "/__pycache__/")

    def _scan_files(self):
        patterns = ("docs/**/*.md", "src/**/*.py", "*.md")
        seen = set()
        for pat in patterns:
            for path in glob.glob(os.path.join(ROOT, pat), recursive=True):
                norm = path.replace(os.sep, "/")
                if any(s in norm for s in self.SKIP_DIRS):
                    continue
                seen.add(path)
        return sorted(seen)

    def test_survey_note_actually_absent(self):
        # If someone later commits the survey note, this test should be
        # revisited rather than silently passing on a stale assumption.
        matches = glob.glob(
            os.path.join(ROOT, "**", self.ORPHAN), recursive=True)
        repo_matches = [
            m for m in matches
            if not any(s in m.replace(os.sep, "/") for s in self.SKIP_DIRS)
        ]
        self.assertEqual(
            repo_matches, [],
            f"{self.ORPHAN} now exists in the tree; update this guard.",
        )

    def test_no_orphan_survey_citation(self):
        offenders = []
        for path in self._scan_files():
            with open(path, encoding="utf-8") as f:
                text = f.read()
            if self.ORPHAN in text:
                offenders.append(os.path.relpath(path, ROOT))
        self.assertEqual(
            offenders, [],
            f"Dangling citation of {self.ORPHAN} in:\n  "
            + "\n  ".join(offenders),
        )


if __name__ == "__main__":
    unittest.main()
