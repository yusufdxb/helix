"""Opt-in complete native stack regression, never a hardware test.

HELIX_RUN_NATIVE_GATE_SIM=1 python3 -m pytest tests/sim_integration/test_native_sport_gate.py -v
Requires a built workspace with vendor ROS messages. Each case owns a separate
localhost domain and starts the fixed fake-controller launch through the CLI.
"""
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

CASES = ('sink', 'supervisor', 'arbiter', 'guard', 'intent_loss', 'status_loss',
         'request_loss', 'response_loss', 'state_loss', 'reject_stop', 'ack_only_stop',
         'drop_stop')


@pytest.mark.skipif(os.environ.get('HELIX_RUN_NATIVE_GATE_SIM') != '1',
                    reason='opt-in localhost native/vendor simulation')
@pytest.mark.parametrize('case', CASES)
def test_complete_native_gate(case, tmp_path):
    script = Path(__file__).resolve().parents[2] / 'scripts/sim_native_sport_gate.py'
    output = tmp_path / 'result.json'
    with (tmp_path / 'driver.log').open('w') as log:
        run = subprocess.run([sys.executable, str(script), '--cases', case,
                              '--domain', str(180 + CASES.index(case)), '--out', str(output)],
                             stdout=log, stderr=subprocess.STDOUT, timeout=120)
    assert output.exists(), (tmp_path / 'driver.log').read_text()
    result = json.loads(output.read_text())
    assert run.returncode == 0, result['cases'][0].get('error')
    assert result['hardware_motion'] is False
    assert result['controller_mode_changes'] is False
    assert result['cases'][0]['passed']
