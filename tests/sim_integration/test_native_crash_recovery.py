"""Crash recovery of the armed native stack against the fake GO2.

Runs scripts/sim_native_crash_recovery.py (about 2 minutes). Needs the built
workspace and the unitree message packages sourced; localhost only.
"""
import json
import subprocess
import sys
from pathlib import Path

import pytest

pytest.importorskip('unitree_api')
pytest.importorskip('helix_msgs')

REPO = Path(__file__).resolve().parents[2]


@pytest.fixture(scope='module')
def cases(tmp_path_factory):
    out = tmp_path_factory.mktemp('crash') / 'result.json'
    subprocess.run([sys.executable, str(REPO / 'scripts' / 'sim_native_crash_recovery.py'),
                    '--out', str(out), '--domain', '94'], timeout=600, check=True)
    return {c['case']: c for c in json.loads(out.read_text())['cases']}


@pytest.mark.timeout(660)
@pytest.mark.parametrize('case', ['detector', 'supervisor', 'arbiter'])
def test_a_killed_node_ends_motion(cases, case):
    c = cases[case]
    assert c['checks']['stopmove_after_kill'], c
    assert c['moves_after_first_stop'] == 0, c


@pytest.mark.timeout(660)
def test_detector_restart_waits_for_operator_release(cases):
    checks = cases['detector']['checks']
    assert checks['await_release_after_restart']
    assert checks['no_move_while_awaiting_release']
    assert checks['unhealthy_release_refused']
    assert checks['prehealth_release_stays_invalid']
    assert checks['nominal_after_release'] and checks['move_resumes_after_release']


@pytest.mark.timeout(660)
def test_supervisor_restart_requires_operator_release(cases):
    c = cases['supervisor']
    assert c['moves_after_restart'] == 0, c
    for check in ('held_before_supervisor_kill', 'supervisor_restart_keeps_hold',
                  'repeated_restarts_keep_hold', 'stale_releases_cannot_clear_hold',
                  'stale_service_release_refused',
                  'lifecycle_restart_keeps_hold', 'stale_lifecycle_release_refused',
                  'operator_release_accepted', 'move_resumes_after_release'):
        assert c['checks'][check], c


@pytest.mark.timeout(660)
def test_arbiter_startup_requires_operator_release(cases):
    for c in cases.values():
        assert c['checks']['startup_requires_release'], c
        assert c['checks']['startup_release_accepted'], c


@pytest.mark.timeout(660)
def test_sink_crash_remains_an_unresolved_failure(cases):
    c = cases['sink']
    assert c['checks']['no_request_after_kill'], c
    assert c['first_stop_after_kill_ms'] is None, c
    assert c['safety_result'] == 'UNRESOLVED_NO_STOP_PATH', c
    assert c['passed'] is False, c
