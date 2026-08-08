"""R4 regression: a node crash is reported but never actuates cmd_vel.

R4 semantics (rules.py): CRASH → LOG_ONLY. The SafetyEnvelope marks
LOG_ONLY as ACCEPTED but `publish=false`, so no zero-Twist is emitted on
/helix/cmd_vel — the recovery layer intentionally stays out of the way
and leaves the decision to a higher tier.

REQUIREMENT: Isaac Sim bridge running before invoking.
"""
import os
import signal
import subprocess
from pathlib import Path

import pytest
from rosbags.highlevel import AnyReader

from scripts.validate_closed_loop_bag import helix_typestore

SCENARIO_DURATION_S = 40.0
CRASH_DELAY_S = 15.0
TARGET_NODE = 'helix_context_buffer'


@pytest.fixture(scope='module')
def scenario_run(tmp_path_factory):
    out_dir = tmp_path_factory.mktemp('sim_run_r4')

    scenario = subprocess.Popen(
        ['python3', 'scripts/sim_faults/run_closed_loop_scenario.py',
         '--duration', str(SCENARIO_DURATION_S),
         '--schedule', f'{int(SCENARIO_DURATION_S + 10)}:10',
         '--artifact-dir', str(out_dir)],
        preexec_fn=os.setsid,
    )

    crasher = subprocess.Popen(
        ['python3', 'scripts/sim_faults/inject_node_crash.py',
         '--node', TARGET_NODE,
         '--after', str(CRASH_DELAY_S)],
        preexec_fn=os.setsid,
    )

    try:
        scenario.wait(timeout=SCENARIO_DURATION_S + 60.0)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(scenario.pid), signal.SIGTERM)
        scenario.wait(timeout=10.0)
    finally:
        if crasher.poll() is None:
            os.killpg(os.getpgid(crasher.pid), signal.SIGTERM)
        crash_rc = crasher.wait(timeout=5.0)

    # Without this the injector can fail to find the node, kill nothing, and
    # every assertion below reports "no CRASH observed" as if the detector
    # were broken. Fail at the real cause instead.
    assert crash_rc == 0, f'crash injector did not kill {TARGET_NODE} (rc={crash_rc})'

    return out_dir / 'bag'


def _read_topic(bag_dir: Path, topic: str):
    msgs = []
    with AnyReader([bag_dir], default_typestore=helix_typestore()) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        for conn, ts, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, conn.msgtype)
            msgs.append((ts, msg))
    return msgs


def test_crash_fault_fires(scenario_run):
    faults = _read_topic(scenario_run, '/helix/faults')
    crash_faults = [m for _, m in faults if m.fault_type == 'CRASH']
    assert crash_faults, 'no CRASH FaultEvent observed after node kill'


def test_r4_emits_log_only_hint(scenario_run):
    hints = _read_topic(scenario_run, '/helix/recovery_hints')
    r4_hints = [m for _, m in hints
                if m.rule_matched == 'R4' and m.suggested_action == 'LOG_ONLY']
    assert r4_hints, 'no R4 LOG_ONLY hint observed'


def test_r4_never_turns_a_crash_into_actuation(scenario_run):
    """The crash must be audited as LOG_ONLY and never as a stop.

    Counting cmd_vel messages in a window after the audit does not test this.
    Any concurrent rule holding a stop keeps a 20 Hz zero-twist stream running
    for its own reasons, and without an upstream /utlidar/cloud publisher R1
    reads the topic as stale and does exactly that for the whole scenario. So
    assert attribution instead: nothing derived from the CRASH fault may ever
    become an actuating action.
    """
    faults = _read_topic(scenario_run, '/helix/faults')
    crashes = [m for _, m in faults if m.fault_type == 'CRASH']
    assert crashes, 'no CRASH FaultEvent observed'
    crash_ids = {m.node_name for m in crashes}

    actions = _read_topic(scenario_run, '/helix/recovery_actions')
    from_crash = [m for _, m in actions if m.fault_id in crash_ids]
    assert from_crash, f'no recovery action attributed to {crash_ids}'

    assert all(m.action == 'LOG_ONLY' for m in from_crash), (
        'CRASH produced a non-LOG_ONLY action: '
        f'{sorted({m.action for m in from_crash})}'
    )
    assert any(m.status == 'ACCEPTED' for m in from_crash), (
        'LOG_ONLY was never ACCEPTED: '
        f'{sorted({m.status for m in from_crash})}'
    )
