"""Unit tests for helix_diagnosis.rules — pure function, no ROS 2 runtime."""
from types import SimpleNamespace

from helix_diagnosis.rules import STATE_IDLE, STATE_STOP_AND_HOLD, evaluate


def _fault(fault_type='ANOMALY', metric='rate_hz/utlidar_cloud', severity=2,
           node_name='rate_hz/utlidar_cloud'):
    """Build a FaultEvent-shaped object matching the live emitter schema.

    helix_core.anomaly_detector writes ``context_keys=['metric_name', ...]``
    with value like ``'rate_hz/utlidar_cloud'`` and uses ``node_name`` as the
    correlation identifier (FaultEvent has no fault_id field).
    """
    return SimpleNamespace(
        node_name=node_name,
        fault_type=fault_type,
        severity=severity,          # 1=WARN, 2=ERROR, 3=CRITICAL
        detail='',
        timestamp=0.0,
        context_keys=['metric_name'],
        context_values=[metric],
    )


def _ctx(anomaly_clear_seconds=0.0):
    return SimpleNamespace(
        rosout_ring=[],
        metrics_json='{}',
        node_health_json='{}',
        snapshot_time=0.0,
        anomaly_clear_seconds=anomaly_clear_seconds,
    )


def test_r1_fires_on_lidar_rate_anomaly_error():
    hint = evaluate(_fault(), _ctx(), current_state=STATE_IDLE)
    assert hint is not None
    assert hint.suggested_action == 'STOP_AND_HOLD'
    assert hint.rule_matched == 'R1'
    assert hint.fault_id == 'rate_hz/utlidar_cloud'  # from fault.node_name


def test_r1_does_not_fire_on_warn_severity():
    hint = evaluate(_fault(severity=1), _ctx(), current_state=STATE_IDLE)
    assert hint is None


def test_r1_does_not_fire_on_unrelated_metric():
    hint = evaluate(_fault(metric='cpu_temp'), _ctx(), current_state=STATE_IDLE)
    assert hint is None


def test_r2_fires_when_anomaly_has_cleared_and_state_is_stop():
    # No fault event, but we have state + clear window
    hint = evaluate(None, _ctx(anomaly_clear_seconds=3.5), current_state=STATE_STOP_AND_HOLD)
    assert hint is not None
    assert hint.suggested_action == 'RESUME'
    assert hint.rule_matched == 'R2'


def test_r2_does_not_fire_before_clear_window():
    hint = evaluate(None, _ctx(anomaly_clear_seconds=1.0), current_state=STATE_STOP_AND_HOLD)
    assert hint is None


def test_r2_does_not_fire_when_idle():
    hint = evaluate(None, _ctx(anomaly_clear_seconds=10.0), current_state=STATE_IDLE)
    assert hint is None


def test_r3_fires_on_critical_log_pattern():
    hint = evaluate(_fault(fault_type='LOG_PATTERN', severity=3), _ctx(), current_state=STATE_IDLE)
    assert hint is not None
    assert hint.suggested_action == 'STOP_AND_HOLD'
    assert hint.rule_matched == 'R3'


def test_r3_does_not_fire_on_error_log_pattern():
    hint = evaluate(_fault(fault_type='LOG_PATTERN', severity=2), _ctx(), current_state=STATE_IDLE)
    assert hint is None


def test_r4_log_only_on_crash():
    hint = evaluate(_fault(fault_type='CRASH', severity=3), _ctx(), current_state=STATE_IDLE)
    assert hint is not None
    assert hint.suggested_action == 'LOG_ONLY'
    assert hint.rule_matched == 'R4'


def test_no_match_returns_none():
    hint = evaluate(_fault(fault_type='NETWORK'), _ctx(), current_state=STATE_IDLE)
    assert hint is None


def test_first_match_wins():
    # Craft a fault that matches both R3 and a hypothetical later rule — R3 must win.
    hint = evaluate(_fault(fault_type='LOG_PATTERN', severity=3), _ctx(), current_state=STATE_IDLE)
    assert hint.rule_matched == 'R1' or hint.rule_matched == 'R3'
    # Specifically R3 in current ordering:
    assert hint.rule_matched == 'R3'
