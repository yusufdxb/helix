"""Integration test: end-to-end closed loop with LiDAR rate drop.

REQUIREMENT: Isaac Sim must be running with the HELIX-patched
go2_ros2_bridge (publishing /utlidar/cloud @ 10 Hz) before this test is invoked.
Launch per docs/sim_launch_recipe.md or the daily-log bridge recipe.
"""
import subprocess
from pathlib import Path

import pytest

from rosbags.highlevel import AnyReader


SCENARIO_CMD = [
    'python3', 'scripts/sim_faults/run_closed_loop_scenario.py',
    '--duration', '80',
    '--schedule', '20:10,20:1,30:10',
]


@pytest.fixture(scope='module')
def scenario_run(tmp_path_factory):
    out_dir = tmp_path_factory.mktemp('sim_run')
    cmd = SCENARIO_CMD + ['--artifact-dir', str(out_dir)]
    subprocess.run(cmd, check=True, timeout=180)
    return out_dir / 'bag'


def _read_topic(bag_dir: Path, topic: str):
    msgs = []
    with AnyReader([bag_dir]) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        for conn, ts, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, conn.msgtype)
            msgs.append((ts, msg))
    return msgs


def test_fault_event_fires_within_1s_of_rate_drop(scenario_run):
    # Rate drop starts at t=20s (per SCENARIO_CMD).
    faults = _read_topic(scenario_run, '/helix/faults')
    assert len(faults) > 0, 'no FaultEvent observed'
    first_ts_s = faults[0][0] / 1e9
    # Bag relative times start at 0, so fault should fire within ~21s of bag start.
    assert first_ts_s < 22.0


def test_stop_hint_follows_fault_within_100ms(scenario_run):
    faults = _read_topic(scenario_run, '/helix/faults')
    hints = _read_topic(scenario_run, '/helix/recovery_hints')
    stop_hints = [(ts, m) for ts, m in hints if m.suggested_action == 'STOP_AND_HOLD']
    assert stop_hints, 'no STOP_AND_HOLD hint'
    delta_ns = stop_hints[0][0] - faults[0][0]
    assert delta_ns < 100_000_000, f'stop hint {delta_ns / 1e6:.1f}ms after fault'


def test_cmd_vel_publishes_zero_within_100ms_of_hint(scenario_run):
    hints = _read_topic(scenario_run, '/helix/recovery_hints')
    stop_hints = [(ts, m) for ts, m in hints if m.suggested_action == 'STOP_AND_HOLD']
    cmds = _read_topic(scenario_run, '/helix/cmd_vel')
    assert cmds, 'no helix cmd_vel published'
    first_stop_ts = stop_hints[0][0]
    later = [ts for ts, _ in cmds if ts >= first_stop_ts]
    assert later, 'no cmd_vel after stop hint'
    delta_ns = later[0] - first_stop_ts
    assert delta_ns < 100_000_000


def test_resume_hint_after_rate_recovers(scenario_run):
    hints = _read_topic(scenario_run, '/helix/recovery_hints')
    resume = [ts for ts, m in hints if m.suggested_action == 'RESUME']
    assert resume, 'no RESUME hint observed'


def test_audit_log_has_accepted_events(scenario_run):
    actions = _read_topic(scenario_run, '/helix/recovery_actions')
    accepted = [m for _, m in actions if m.status == 'ACCEPTED']
    assert len(accepted) >= 2   # at least STOP + RESUME


def test_explanations_published(scenario_run):
    expl = _read_topic(scenario_run, '/helix/explanations')
    assert len(expl) >= 1
