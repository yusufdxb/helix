"""Tests for the pure template-rendering function."""
from types import SimpleNamespace

from helix_explanation.llm_explainer import render_template


def _fault():
    return SimpleNamespace(
        fault_id='f1',
        fault_type='ANOMALY',
        severity=2,
        detail='',
        timestamp=1.234,
        context_keys=['metric'],
        context_values=['utlidar_rate'],
    )


def _hint():
    return SimpleNamespace(
        fault_id='f1',
        suggested_action='STOP_AND_HOLD',
        confidence=0.9,
        reasoning='LiDAR rate degraded',
        rule_matched='R1',
    )


def test_template_contains_key_fields():
    s = render_template(_fault(), _hint())
    assert 'ANOMALY' in s
    assert 'utlidar_rate' in s
    assert 'STOP_AND_HOLD' in s
    assert 'R1' in s


def test_template_handles_missing_fault():
    s = render_template(None, _hint())
    # Resume hints have no fault; render something still useful
    assert 'STOP_AND_HOLD' in s or 'RESUME' in s


def test_template_handles_empty_context():
    f = _fault()
    f.context_keys = []
    f.context_values = []
    s = render_template(f, _hint())
    assert 'ANOMALY' in s
