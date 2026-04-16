"""Offline tests for scripts/attachability_matrix.py scoring logic."""

import os
import sys
import unittest

# Allow importing from the scripts directory without installing.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), os.pardir, "scripts"))

from attachability_matrix import HELIX_INPUTS, compute_attachability  # noqa: E402


def _make_topic(name: str, msg_type: str) -> dict:
    return {"topic": name, "type": msg_type}


class TestAttachabilityScoreBounds(unittest.TestCase):
    """with_adapters must always be in [0, 1]."""

    def test_no_topics(self):
        result = compute_attachability([])
        self.assertEqual(result["scores"]["native_coverage"], 0.0)
        self.assertEqual(result["scores"]["with_adapters"], 0.0)

    def test_all_native(self):
        topics = [
            _make_topic(t, spec["type"])
            for t, spec in HELIX_INPUTS.items()
        ]
        result = compute_attachability(topics)
        self.assertAlmostEqual(result["scores"]["native_coverage"], 1.0)
        self.assertAlmostEqual(result["scores"]["with_adapters"], 1.0)

    def test_many_adaptable_topics_cannot_exceed_one(self):
        """Regression: with 2 native + 54 adaptable, score was 1.5."""
        topics = [
            _make_topic("/diagnostics", "diagnostic_msgs/msg/DiagnosticArray"),
            _make_topic("/rosout", "rcl_interfaces/msg/Log"),
        ]
        # Add 100 String topics — more than enough to exceed 1.0 under old formula
        for i in range(100):
            topics.append(_make_topic(f"/extra/{i}", "std_msgs/msg/String"))
        result = compute_attachability(topics)
        self.assertLessEqual(result["scores"]["with_adapters"], 1.0)
        self.assertGreaterEqual(result["scores"]["with_adapters"], 0.0)

    def test_only_custom_topics(self):
        topics = [
            _make_topic("/custom/a", "unitree_api/msg/Request"),
            _make_topic("/custom/b", "unitree_api/msg/Response"),
        ]
        result = compute_attachability(topics)
        self.assertEqual(result["scores"]["native_coverage"], 0.0)
        self.assertEqual(result["scores"]["with_adapters"], 0.0)

    def test_partial_native_with_adapters(self):
        topics = [
            _make_topic("/rosout", "rcl_interfaces/msg/Log"),
            _make_topic("/imu", "sensor_msgs/msg/Imu"),
        ]
        result = compute_attachability(topics)
        self.assertEqual(result["scores"]["native_coverage"], 0.25)
        # 1 native + min(1 adaptable, 3 gaps) = 2/4 = 0.5
        self.assertAlmostEqual(result["scores"]["with_adapters"], 0.5)
        self.assertLessEqual(result["scores"]["with_adapters"], 1.0)


class TestAttachabilityClassification(unittest.TestCase):
    """Mixed topics should be classified correctly."""

    def test_demonstrated_vs_candidate(self):
        topics = [
            _make_topic("/imu", "sensor_msgs/msg/Imu"),                # demonstrated
            _make_topic("/odom", "nav_msgs/msg/Odometry"),              # demonstrated
            _make_topic("/gnss", "std_msgs/msg/String"),                # candidate
            _make_topic("/custom", "unitree_api/msg/Request"),          # unreachable
            _make_topic("/rosout", "rcl_interfaces/msg/Log"),           # native (HELIX input)
        ]
        result = compute_attachability(topics)
        self.assertEqual(result["scores"]["demonstrated_adapter_count"], 2)
        self.assertEqual(result["scores"]["candidate_adapter_count"], 1)
        self.assertEqual(result["scores"]["unreachable_topics_count"], 1)

    def test_helix_inputs_not_counted_as_adaptable(self):
        """HELIX's own input topics should not appear in adaptable list."""
        topics = [
            _make_topic("/rosout", "rcl_interfaces/msg/Log"),
            _make_topic("/helix/heartbeat", "std_msgs/msg/String"),
        ]
        result = compute_attachability(topics)
        # /helix/heartbeat is a HELIX input (native), not an adaptable String topic
        self.assertEqual(result["scores"]["candidate_adapter_count"], 0)
        self.assertEqual(result["scores"]["native_coverage"], 0.5)

    def test_duplicate_topics_deduplicated(self):
        """Duplicate topic names should be deduplicated (first occurrence wins)."""
        topics = [
            _make_topic("/imu", "sensor_msgs/msg/Imu"),
            _make_topic("/imu", "std_msgs/msg/String"),  # duplicate, different type
        ]
        result = compute_attachability(topics)
        # First occurrence (Imu) should win → demonstrated, not candidate
        self.assertEqual(result["scores"]["demonstrated_adapter_count"], 1)
        self.assertEqual(result["scores"]["candidate_adapter_count"], 0)
        self.assertEqual(result["platform_summary"]["total_topics"], 1)

    def test_type_mismatch_counted_as_gap(self):
        """A HELIX input present but with wrong type should be a gap, not native."""
        topics = [
            _make_topic("/diagnostics", "std_msgs/msg/String"),  # wrong type
        ]
        result = compute_attachability(topics)
        self.assertEqual(result["scores"]["native_coverage"], 0.0)
        self.assertEqual(result["helix_input_coverage"]["/diagnostics"], "type_mismatch")
        # The wrong-type topic is std_msgs/String but it's a HELIX input name, so
        # it should NOT be counted as an adaptable candidate either
        self.assertEqual(result["scores"]["candidate_adapter_count"], 0)


if __name__ == "__main__":
    unittest.main()
