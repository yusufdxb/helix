"""R3 regression: a CRITICAL log pattern triggers STOP_AND_HOLD recovery.

Same structure as test_lidar_occlusion_recovery. Instead of throttling
the LiDAR rate, we inject a FATAL-level /rosout line that matches the
`nav2_costmap_fail` rule (severity=3, fault_type=LOG_PATTERN).

REQUIREMENT: Isaac Sim bridge running (for /utlidar/cloud) before invoking.
"""
import os
import signal
import subprocess
import time
from pathlib import Path

import pytest

from rosbags.highlevel import AnyReader


SCENARIO_DURATION_S = 40.0
LOG_INJECT_DELAY_S = 15.0


@pytest.fixture(scope='module')
def scenario_run(tmp_path_factory):
    out_dir = tmp_path_factory.mktemp('sim_run_r3')

    # Start the orchestrator without a LiDAR-rate-drop schedule — we just need
    # the closed loop up and ready to receive a log event. Run with schedule
    # '60:10' so /utlidar/cloud_throttled is steady at 10 Hz for the whole run.
    scenario = subprocess.Popen(
        ['python3', 'scripts/sim_faults/run_closed_loop_scenario.py',
         '--duration', str(SCENARIO_DURATION_S),
         '--schedule', f'{int(SCENARIO_DURATION_S + 10)}:10',
         '--artifact-dir', str(out_dir)],
        preexec_fn=os.setsid,
    )

    # Inject the critical log line partway through.
    injector = subprocess.Popen(
        ['python3', 'scripts/sim_faults/inject_critical_log.py',
         '--after', str(LOG_INJECT_DELAY_S),
         '--message', 'Costmap failed to initialize'],
        preexec_fn=os.setsid,
    )

    try:
        scenario.wait(timeout=SCENARIO_DURATION_S + 60.0)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(scenario.pid), signal.SIGTERM)
        scenario.wait(timeout=10.0)
    finally:
        if injector.poll() is None:
            os.killpg(os.getpgid(injector.pid), signal.SIGTERM)
        injector.wait(timeout=5.0)

    return out_dir / 'bag'


def _read_topic(bag_dir: Path, topic: str):
    msgs = []
    with AnyReader([bag_dir]) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        for conn, ts, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, conn.msgtype)
            msgs.append((ts, msg))
    return msgs


def test_log_pattern_fault_fires(scenario_run):
    faults = _read_topic(scenario_run, '/helix/faults')
    log_pattern_faults = [m for _, m in faults if m.fault_type == 'LOG_PATTERN']
    assert log_pattern_faults, 'no LOG_PATTERN FaultEvent observed'


def test_r3_emits_stop_and_hold_hint(scenario_run):
    hints = _read_topic(scenario_run, '/helix/recovery_hints')
    r3_hints = [m for _, m in hints
                if m.rule_matched == 'R3' and m.suggested_action == 'STOP_AND_HOLD']
    assert r3_hints, 'no R3 STOP_AND_HOLD hint observed'


def test_r3_recovery_action_accepted(scenario_run):
    actions = _read_topic(scenario_run, '/helix/recovery_actions')
    # The envelope keys cooldown by fault_type, so the first LOG_PATTERN action
    # should be ACCEPTED regardless of any earlier ANOMALY events.
    accepted = [m for _, m in actions
                if m.status == 'ACCEPTED' and m.action == 'STOP_AND_HOLD']
    assert accepted, 'no ACCEPTED STOP_AND_HOLD action observed'
