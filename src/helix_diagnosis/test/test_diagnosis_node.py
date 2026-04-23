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
