"""SinkFeedback: controller ack matching and measured-stop detection."""
import math

from helix_arbiter.sport_sink_core import (
    API_MOVE,
    API_STOP_MOVE,
    SinkFeedback,
    StillThresholds,
)


def test_stop_ack_requires_own_request_and_code_zero():
    fb = SinkFeedback()
    fb.on_sent(10, API_STOP_MOVE, 'ZERO', 1.0)
    assert not fb.on_response(99, API_STOP_MOVE, 0, 1.01)   # someone else's
    assert not fb.stop_acked
    assert fb.on_response(10, API_STOP_MOVE, 3, 1.02)       # refused
    assert not fb.stop_acked
    fb.on_sent(11, API_STOP_MOVE, 'ZERO', 1.5)
    assert fb.on_response(11, API_STOP_MOVE, 0, 1.52)
    assert fb.stop_acked
    assert fb.stop_ack_sent_at == 1.5
    assert fb.stop_acked_at == 1.52


def test_api_id_mismatch_is_not_an_ack():
    fb = SinkFeedback()
    fb.on_sent(10, API_STOP_MOVE, 'ZERO', 1.0)
    assert not fb.on_response(10, API_MOVE, 0, 1.01)
    assert not fb.stop_acked


def test_move_after_stop_clears_ack_and_late_stop_ack_ignored():
    fb = SinkFeedback()
    fb.on_sent(10, API_STOP_MOVE, 'ZERO', 1.0)
    fb.on_sent(11, API_MOVE, 'MOVE', 1.1)
    # ack of the StopMove that predates the Move must not count
    fb.on_response(10, API_STOP_MOVE, 0, 1.2)
    assert not fb.stop_acked
    fb.on_sent(12, API_STOP_MOVE, 'DEADMAN', 1.3)
    fb.on_response(12, API_STOP_MOVE, 0, 1.33)
    assert fb.stop_acked
    fb.on_sent(13, API_MOVE, 'MOVE', 2.0)
    assert not fb.stop_acked


def test_ack_stamp_follows_the_newest_acknowledged_stopmove():
    # A supervisor that requested a hold at t=1.2 must see that a StopMove
    # sent after 1.2 was answered, although StopMove was already repeating.
    fb = SinkFeedback()
    fb.on_sent(1, API_STOP_MOVE, 'ZERO', 1.0)
    fb.on_response(1, API_STOP_MOVE, 0, 1.02)
    fb.on_sent(2, API_STOP_MOVE, 'ZERO', 1.5)
    assert fb.stop_ack_sent_at == 1.0
    fb.on_response(2, API_STOP_MOVE, 0, 1.52)
    assert fb.stop_ack_sent_at == 1.5
    assert fb.stop_acked_at == 1.52
    # a late answer to the older request does not move the stamp back
    fb.on_response(1, API_STOP_MOVE, 0, 1.6)
    assert fb.stop_ack_sent_at == 1.5


def test_stopped_needs_still_s_below_thresholds():
    fb = SinkFeedback(StillThresholds(0.05, 0.1, 0.3))
    fb.on_state(0.2, 0.0, 0.0, 3, 0.0)
    assert not fb.stopped
    fb.on_state(0.01, 0.01, 0.02, 3, 0.1)
    fb.on_state(0.01, 0.0, 0.0, 3, 0.3)
    assert not fb.stopped
    fb.on_state(0.0, 0.0, 0.0, 3, 0.41)
    assert fb.stopped and fb.stopped_since == 0.41
    fb.on_state(0.0, 0.0, 0.3, 3, 0.5)    # yaw counts as motion
    assert not fb.stopped


def test_stale_state_is_not_stopped():
    fb = SinkFeedback(StillThresholds(0.05, 0.1, 0.0), feedback_timeout_s=1.0)
    assert not fb.feedback_available(0.0)
    fb.on_state(0.0, 0.0, 0.0, 1, 1.0)
    assert fb.stopped_now(1.5)
    assert not fb.stopped_now(2.6)


def test_nan_state_is_not_stopped():
    fb = SinkFeedback(StillThresholds(0.05, 0.1, 0.0))
    fb.on_state(math.nan, 0.0, 0.0, 1, 1.0)
    assert not fb.stopped


def test_tracking_is_bounded():
    fb = SinkFeedback()
    for i in range(SinkFeedback.MAX_TRACKED + 50):
        fb.on_sent(i, API_STOP_MOVE, 'ZERO', float(i))
    assert len(fb._sent) == SinkFeedback.MAX_TRACKED
    assert not fb.on_response(0, API_STOP_MOVE, 0, 1e4)
