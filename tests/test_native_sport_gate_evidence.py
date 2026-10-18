"""Recheck safety claims against the committed native simulation trace."""
import json
from pathlib import Path

import pytest

RESULT = json.loads((Path(__file__).resolve().parents[1] /
                     'results/native_sport_gate_integration_sim.json').read_text())


@pytest.mark.parametrize('case', RESULT['cases'], ids=lambda c: c['case'])
def test_native_trace_has_no_unauthorized_move(case):
    events = case['events']
    assert case['passed'] and all(case['checks'].values())
    assert not any(e['kind'] == 'unauthorized_move' for e in events)
    assert RESULT['hardware_motion'] is False
    assert RESULT['controller_mode_changes'] is False
    requests = [e for e in events if e['kind'] == 'request']
    assert {e['api'] for e in requests} <= {1003, 1008}
    releases = [e['t'] for e in events if e['kind'] == 'operator_guard' and not e['stale']]
    assert len(releases) == 2
    assert any(e['api'] == 1008 for e in requests)
    assert not any(e['api'] == 1008 and e['t'] < releases[0] for e in requests)
    injection = next(e['t'] for e in events if e['kind'] == 'injection')
    stops = [e['t'] for e in requests if e['t'] > injection and e['api'] == 1003]
    boundary = stops[0] if stops else injection + .5
    assert not any(e['api'] == 1008 and boundary < e['t'] < releases[1] for e in requests)
    assert any(e['kind'] == 'operator_native' and not e['accepted'] and
               e['t'] > injection for e in events)
    assert any(e['kind'] == 'operator_guard' and e['stale'] and e['t'] > injection
               for e in events)


@pytest.mark.parametrize('name', ['guard', 'request_loss', 'reject_stop', 'ack_only_stop', 'drop_stop'])
def test_final_output_failures_are_preserved_as_unresolved(name):
    case = next(c for c in RESULT['cases'] if c['case'] == name)
    assert case['latency_ms']['detection_to_simulated_still'] is None
    assert case['speed_before_repair'] > .15
    assert case['failure_state'] == 'STOP_UNCONFIRMED'
    if name == 'ack_only_stop':
        assert case['latency_ms']['detection_to_stop_ack'] is not None
    if name in ('guard', 'request_loss'):
        assert case['latency_ms']['detection_to_stop_delivery'] is None
