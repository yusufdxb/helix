"""Unit tests for DisplacementTracker (pure-function, no ROS)."""
import math

import pytest

from helix_adapter.pose_drift import DisplacementTracker


def test_no_update_returns_nan():
    d = DisplacementTracker()
    assert math.isnan(d.rate_or_nan(now=1.0))


def test_single_update_rate_is_zero():
    d = DisplacementTracker()
    d.update(0.0, 0.0, 0.0, t=0.0)
    assert d.rate_or_nan(now=0.1) == 0.0


def test_straight_line_motion_matches_speed():
    d = DisplacementTracker()
    d.update(0.0, 0.0, 0.0, t=0.0)
    d.update(1.0, 0.0, 0.0, t=1.0)
    assert math.isclose(d.rate_or_nan(now=1.0), 1.0, rel_tol=1e-9)


def test_three_dimensional_euclidean_distance():
    d = DisplacementTracker()
    d.update(0.0, 0.0, 0.0, t=0.0)
    d.update(3.0, 4.0, 0.0, t=1.0)
    assert math.isclose(d.rate_or_nan(now=1.0), 5.0, rel_tol=1e-9)


def test_min_dt_guard_drops_tight_burst():
    d = DisplacementTracker(min_dt=1e-3)
    d.update(0.0, 0.0, 0.0, t=0.0)
    d.update(1000.0, 0.0, 0.0, t=1e-6)
    assert d.rate_or_nan(now=0.001) == 0.0


def test_min_dt_guard_still_refreshes_staleness():
    d = DisplacementTracker(stale_sec=1.0, min_dt=1e-3)
    d.update(0.0, 0.0, 0.0, t=0.0)
    d.update(0.0, 0.0, 0.0, t=1e-6)
    assert d.rate_or_nan(now=0.5) == 0.0


def test_stale_after_window():
    d = DisplacementTracker(stale_sec=2.0)
    d.update(0.0, 0.0, 0.0, t=0.0)
    d.update(1.0, 0.0, 0.0, t=1.0)
    assert math.isnan(d.rate_or_nan(now=100.0))


def test_stationary_reports_zero():
    d = DisplacementTracker()
    d.update(5.0, 5.0, 5.0, t=0.0)
    d.update(5.0, 5.0, 5.0, t=1.0)
    d.update(5.0, 5.0, 5.0, t=2.0)
    assert d.rate_or_nan(now=2.0) == 0.0


def test_invalid_params_raise():
    with pytest.raises(ValueError):
        DisplacementTracker(stale_sec=0)
    with pytest.raises(ValueError):
        DisplacementTracker(min_dt=0)
