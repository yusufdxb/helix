"""Unit tests for RateWindow (pure-function, no ROS)."""
import math

from helix_adapter.rate_window import RateWindow


class FakeClock:
    """Driveable monotonic clock."""

    def __init__(self, start: float = 0.0) -> None:
        self.t = start

    def __call__(self) -> float:
        return self.t


def test_empty_window_returns_nan():
    rw = RateWindow(window_sec=5.0, clock=FakeClock())
    assert math.isnan(rw.rate_or_nan())


def test_single_sample_returns_zero():
    clock = FakeClock()
    rw = RateWindow(window_sec=5.0, clock=clock)
    rw.record()
    assert rw.rate_or_nan() == 0.0


def test_two_samples_one_second_apart_is_one_hz():
    clock = FakeClock()
    rw = RateWindow(window_sec=5.0, clock=clock)
    rw.record()
    clock.t = 1.0
    rw.record()
    assert rw.rate_or_nan() == 1.0


def test_ten_samples_in_one_second_is_nine_hz():
    clock = FakeClock()
    rw = RateWindow(window_sec=5.0, clock=clock)
    for i in range(10):
        clock.t = i * 0.1
        rw.record()
    rate = rw.rate_or_nan()
    assert math.isclose(rate, 10.0, rel_tol=0.01)


def test_stale_after_window_expires():
    clock = FakeClock()
    rw = RateWindow(window_sec=5.0, clock=clock)
    rw.record()
    clock.t = 0.5
    rw.record()
    clock.t = 10.0
    assert math.isnan(rw.rate_or_nan())


def test_eviction_drops_old_samples():
    clock = FakeClock()
    rw = RateWindow(window_sec=2.0, clock=clock)
    clock.t = 0.0
    rw.record()
    clock.t = 1.0
    rw.record()
    clock.t = 5.0
    rw.record()
    clock.t = 5.5
    rw.record()
    rate = rw.rate_or_nan()
    assert math.isclose(rate, 2.0, rel_tol=0.01)


def test_identical_timestamps_returns_zero():
    clock = FakeClock()
    rw = RateWindow(window_sec=5.0, clock=clock)
    rw.record()
    rw.record()
    rw.record()
    assert rw.rate_or_nan() == 0.0


def test_invalid_window_raises():
    import pytest
    with pytest.raises(ValueError):
        RateWindow(window_sec=0)
    with pytest.raises(ValueError):
        RateWindow(window_sec=-1.0)
