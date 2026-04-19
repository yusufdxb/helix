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
import time
from pathlib import Path

import pytest

from rosbags.highlevel import AnyReader


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
        crasher.wait(timeout=5.0)

    return out_dir / 'bag'


def _read_topic(bag_dir: Path, topic: str):
    msgs = []
    with AnyReader([bag_dir]) as reader:
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


def test_r4_audits_accepted_without_publishing_cmd_vel(scenario_run):
    # R4's ACCEPTED LOG_ONLY action must not produce any helix/cmd_vel publish
    # after it fires. We locate the first LOG_ONLY audit and confirm no
    # subsequent Twist was emitted on /helix/cmd_vel as a direct consequence.
    actions = _read_topic(scenario_run, '/helix/recovery_actions')
    log_only = [(ts, m) for ts, m in actions
                if m.action == 'LOG_ONLY' and m.status == 'ACCEPTED']
    assert log_only, 'no ACCEPTED LOG_ONLY audit observed'

    first_log_only_ts = log_only[0][0]
    cmds = _read_topic(scenario_run, '/helix/cmd_vel')
    # Any cmd_vel already in flight from an earlier ANOMALY run is fine; we
    # assert that the LOG_ONLY path itself does NOT start a new publish stream
    # within the 500 ms window after the audit.
    window_ns = 500_000_000
    within_window = [ts for ts, _ in cmds
                     if 0 <= ts - first_log_only_ts < window_ns]
    # This is a soft assertion: if any anomalies were already active, the
    # 20 Hz STOP stream would produce messages — but R4 alone must never
    # cause a fresh start. The closed-loop orchestrator sets the schedule to
    # flat 10 Hz, so no anomaly stream is expected in an R4-only scenario.
    assert not within_window, (
        f'R4 LOG_ONLY should not publish cmd_vel; saw {len(within_window)} '
        f'message(s) within 500 ms of the audit'
    )
