"""Tests for DiagnosisNode state machine (subscribed to /helix/faults)."""
from types import SimpleNamespace

from helix_diagnosis.diagnosis_node import (
    STATE_IDLE,
    STATE_STOP_AND_HOLD,
    DiagnosisStateMachine,
)


def _fault(ft='ANOMALY', sev=2, metric='rate_hz/utlidar_cloud', node_name=None, ts=0.0):
    return SimpleNamespace(
        node_name=node_name if node_name is not None else metric,
        fault_type=ft,
        severity=sev,
        detail='',
        timestamp=ts,
        context_keys=['metric_name'],
        context_values=[metric],
    )


def test_starts_idle():
    sm = DiagnosisStateMachine()
    assert sm.current_state == STATE_IDLE


def test_anomaly_moves_to_stop():
    sm = DiagnosisStateMachine()
    hint = sm.process_fault(_fault(), now_seconds=1.0)
    assert hint is not None
    assert hint.suggested_action == 'STOP_AND_HOLD'
    assert sm.current_state == STATE_STOP_AND_HOLD


def test_clear_window_resumes():
    sm = DiagnosisStateMachine()
    sm.process_fault(_fault(), now_seconds=1.0)
    # 4 seconds later with no faults, tick produces RESUME hint
    hint = sm.tick(now_seconds=5.0)
    assert hint is not None
    assert hint.suggested_action == 'RESUME'
    assert sm.current_state == STATE_IDLE


def test_clear_window_not_yet_elapsed():
    sm = DiagnosisStateMachine()
    sm.process_fault(_fault(), now_seconds=1.0)
    hint = sm.tick(now_seconds=2.0)   # only 1s
    assert hint is None
    assert sm.current_state == STATE_STOP_AND_HOLD


def test_new_anomaly_resets_clear_timer():
    sm = DiagnosisStateMachine()
    sm.process_fault(_fault(), now_seconds=1.0)
    sm.tick(now_seconds=2.5)                   # 1.5s clear
    sm.process_fault(_fault(node_name='f2'), now_seconds=2.8)  # new fault resets
    hint = sm.tick(now_seconds=5.0)             # 2.2s since reset — should NOT resume
    assert hint is None


def test_r3_stop_is_releasable():
    """Regression: a STOP_AND_HOLD entered via R3 (critical log pattern) with
    no prior ANOMALY must still be releasable. Before the fix the clear timer
    (_last_anomaly_time) stayed None, so tick() returned None forever and the
    robot was held with no RESUME path."""
    sm = DiagnosisStateMachine()
    hint = sm.process_fault(
        _fault(ft='LOG_PATTERN', sev=3, metric='log', node_name='node_x'),
        now_seconds=1.0,
    )
    assert hint is not None
    assert hint.suggested_action == 'STOP_AND_HOLD'
    assert sm.current_state == STATE_STOP_AND_HOLD
    # Before the clear window: still held.
    assert sm.tick(now_seconds=2.0) is None
    # After the clear window: RESUME is produced and state returns to IDLE.
    resume = sm.tick(now_seconds=5.0)
    assert resume is not None
    assert resume.suggested_action == 'RESUME'
    assert sm.current_state == STATE_IDLE


def test_repeated_r3_stop_pushes_resume_out():
    """A repeated R3 critical-log fault must refresh the clear timer, exactly
    as a repeated ANOMALY does, so RESUME does not fire while critical logs
    are still arriving."""
    sm = DiagnosisStateMachine()
    sm.process_fault(
        _fault(ft='LOG_PATTERN', sev=3, metric='log', node_name='node_x'),
        now_seconds=1.0,
    )
    sm.tick(now_seconds=2.5)                                    # 1.5s clear
    sm.process_fault(
        _fault(ft='LOG_PATTERN', sev=3, metric='log', node_name='node_x'),
        now_seconds=2.8,
    )                                                          # refresh timer
    assert sm.tick(now_seconds=5.0) is None                    # 2.2s — no RESUME
    assert sm.current_state == STATE_STOP_AND_HOLD
