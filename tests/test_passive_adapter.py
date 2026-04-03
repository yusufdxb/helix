"""Offline tests for scripts/passive_adapter.py — no ROS 2 required.

Tests the pure-Python helpers: _parse_bool_metric, _safe_float,
and TopicRateMonitor.
"""

import math
import sys
import os
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), os.pardir, "scripts"))

from passive_adapter import _parse_bool_metric, _safe_float, TopicRateMonitor


# ── Boolean parsing ────────────────────────────────────────────────────────


class TestParseBoolMetric(unittest.TestCase):
    """Regression: string 'false' must NOT become 1.0."""

    def test_python_bool(self):
        self.assertEqual(_parse_bool_metric(True), 1.0)
        self.assertEqual(_parse_bool_metric(False), 0.0)

    def test_int_values(self):
        self.assertEqual(_parse_bool_metric(1), 1.0)
        self.assertEqual(_parse_bool_metric(0), 0.0)
        self.assertEqual(_parse_bool_metric(42), 1.0)

    def test_string_false_variants(self):
        for val in ["false", "False", "FALSE", "0", "no", "off", "none", ""]:
            self.assertEqual(
                _parse_bool_metric(val), 0.0,
                f"Expected 0.0 for string {val!r}"
            )

    def test_string_true_variants(self):
        for val in ["true", "True", "TRUE", "1", "yes", "on"]:
            self.assertEqual(
                _parse_bool_metric(val), 1.0,
                f"Expected 1.0 for string {val!r}"
            )

    def test_none(self):
        self.assertEqual(_parse_bool_metric(None), 0.0)

    def test_float_zero(self):
        self.assertEqual(_parse_bool_metric(0.0), 0.0)

    def test_float_nonzero(self):
        self.assertEqual(_parse_bool_metric(3.14), 1.0)

    def test_nan_returns_zero(self):
        """Regression: float('nan') == 0 is False, so NaN fell through to 1.0."""
        self.assertEqual(_parse_bool_metric(float("nan")), 0.0)

    def test_inf_returns_zero(self):
        self.assertEqual(_parse_bool_metric(float("inf")), 0.0)
        self.assertEqual(_parse_bool_metric(float("-inf")), 0.0)


# ── _safe_float ────────────────────────────────────────────────────────────


class TestSafeFloat(unittest.TestCase):
    """Regression: float('nan') and float('inf') must not reach the Z-score window."""

    def test_normal_values(self):
        self.assertEqual(_safe_float(42), 42.0)
        self.assertEqual(_safe_float("3.14"), 3.14)
        self.assertEqual(_safe_float(0), 0.0)

    def test_nan_rejected(self):
        self.assertIsNone(_safe_float(float("nan")))
        self.assertIsNone(_safe_float("nan"))
        self.assertIsNone(_safe_float("NaN"))

    def test_inf_rejected(self):
        self.assertIsNone(_safe_float(float("inf")))
        self.assertIsNone(_safe_float(float("-inf")))
        self.assertIsNone(_safe_float("inf"))
        self.assertIsNone(_safe_float("-inf"))

    def test_unparseable_rejected(self):
        self.assertIsNone(_safe_float("not_a_number"))
        self.assertIsNone(_safe_float(None))
        self.assertIsNone(_safe_float([1, 2]))

    def test_string_zero(self):
        self.assertEqual(_safe_float("0"), 0.0)


# ── TopicRateMonitor ───────────────────────────────────────────────────────


class TestTopicRateMonitorStartup(unittest.TestCase):
    """Startup: returns 0.0 until enough data."""

    def test_no_messages(self):
        m = TopicRateMonitor(window_sec=1.0)
        self.assertEqual(m.hz(), 0.0)
        self.assertTrue(m.is_stale())

    def test_single_message(self):
        m = TopicRateMonitor(window_sec=1.0)
        m.record()
        self.assertEqual(m.hz(), 0.0)  # need >= 2 for a rate
        self.assertFalse(m.is_stale())


class TestTopicRateMonitorSteady(unittest.TestCase):
    """Steady cadence at known rate."""

    def test_known_rate(self):
        m = TopicRateMonitor(window_sec=5.0)
        # Simulate 10 Hz for 1 second (inject timestamps directly)
        base = time.monotonic()
        for i in range(11):
            m._timestamps.append(base + i * 0.1)
        rate = m.hz()
        self.assertAlmostEqual(rate, 10.0, delta=0.5)


class TestTopicRateMonitorBursty(unittest.TestCase):
    """Bursty arrivals: many messages in a short burst."""

    def test_burst(self):
        m = TopicRateMonitor(window_sec=5.0)
        base = time.monotonic()
        # 50 messages in 0.1 seconds
        for i in range(50):
            m._timestamps.append(base + i * 0.002)
        rate = m.hz()
        # ~500 Hz burst rate — should be high
        self.assertGreater(rate, 100.0)


class TestTopicRateMonitorSparse(unittest.TestCase):
    """Sparse arrivals: only a few messages across the window."""

    def test_sparse(self):
        m = TopicRateMonitor(window_sec=5.0)
        base = time.monotonic()
        # 3 messages across 4 seconds
        m._timestamps.append(base)
        m._timestamps.append(base + 2.0)
        m._timestamps.append(base + 4.0)
        rate = m.hz()
        self.assertAlmostEqual(rate, 0.5, delta=0.1)


class TestTopicRateMonitorStale(unittest.TestCase):
    """Silent/stale: no messages within the window."""

    def test_stale_after_window_expires(self):
        m = TopicRateMonitor(window_sec=0.05)
        m.record()
        self.assertFalse(m.is_stale())
        time.sleep(0.08)  # wait for window to expire
        self.assertTrue(m.is_stale())
        self.assertEqual(m.hz(), 0.0)


class TestTopicRateMonitorRateOrNan(unittest.TestCase):
    """Atomic rate_or_nan() method tests."""

    def test_stale_returns_nan(self):
        m = TopicRateMonitor(window_sec=0.05)
        # No messages ever → stale → NaN
        result = m.rate_or_nan()
        self.assertTrue(math.isnan(result))

    def test_single_message_returns_zero(self):
        """One message in window = startup, not stale → 0.0, not NaN."""
        m = TopicRateMonitor(window_sec=5.0)
        m.record()
        result = m.rate_or_nan()
        self.assertEqual(result, 0.0)
        self.assertFalse(math.isnan(result))

    def test_steady_returns_finite_rate(self):
        m = TopicRateMonitor(window_sec=5.0)
        base = time.monotonic()
        for i in range(11):
            m._timestamps.append(base + i * 0.1)
        result = m.rate_or_nan()
        self.assertTrue(math.isfinite(result))
        self.assertAlmostEqual(result, 10.0, delta=0.5)

    def test_expired_window_returns_nan(self):
        m = TopicRateMonitor(window_sec=0.05)
        m.record()
        time.sleep(0.08)
        result = m.rate_or_nan()
        self.assertTrue(math.isnan(result))


if __name__ == "__main__":
    unittest.main()
