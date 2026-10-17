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
                    '--out', str(out), '--domain', '94'], timeout=600)
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
    assert checks['nominal_after_release'] and checks['move_resumes_after_release']


@pytest.mark.timeout(660)
def test_known_gap_supervisor_restart_resumes_motion(cases):
    """Documents the open gap: a restarted supervisor does not keep the hold.

    When this starts failing, the gap is closed: update docs/EVIDENCE_AUDIT.md.
    """
    assert cases['supervisor']['moves_after_restart'] > 0
