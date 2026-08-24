"""Pure regression tests for the physical-closure evidence gate."""

from scripts.validate_closed_loop_bag import evaluate


def _passing_records():
    return {
        "markers": [{"ts": 20.0, "target_hz": 0.0}],
        "faults": [{
            "ts": 26.0,
            "fault_id": "rate_hz/utlidar_cloud_throttled",
            "fault_type": "ANOMALY",
            "metric_name": "rate_hz/utlidar_cloud_throttled",
            "violation_type": "stale",
        }],
        "hints": [{
            "ts": 26.1,
            "fault_id": "rate_hz/utlidar_cloud_throttled",
            "action": "STOP_AND_HOLD",
        }],
        "actions": [{
            "ts": 26.2,
            "fault_id": "rate_hz/utlidar_cloud_throttled",
            "action": "STOP_AND_HOLD",
            "status": "ACCEPTED",
        }],
        "helix_cmd_vel": [{"ts": 26.3, "zero": True}],
        "cmd_vel": [
            {"ts": 18.0, "zero": False},
            {"ts": 26.4, "zero": True},
        ],
        "odom": [
            {"ts": 18.0, "speed_m_s": 0.20},
            {"ts": 19.0, "speed_m_s": 0.19},
            {"ts": 27.5, "speed_m_s": 0.01},
            {"ts": 28.5, "speed_m_s": 0.02},
        ],
    }


def test_complete_physical_evidence_passes():
    result = evaluate(_passing_records())
    assert result["passed"] is True
    assert all(check["passed"] for check in result["checks"])


def test_zero_commands_without_physical_stop_fail():
    records = _passing_records()
    records["odom"][-2:] = [
        {"ts": 27.5, "speed_m_s": 0.18},
        {"ts": 28.5, "speed_m_s": 0.17},
    ]
    result = evaluate(records)
    assert result["passed"] is False
    failed = {check["name"] for check in result["checks"] if not check["passed"]}
    assert "odometry proves the robot stopped" in failed


def test_unmarked_bag_fails_closed():
    records = _passing_records()
    records["markers"] = []
    result = evaluate(records)
    assert result["passed"] is False


# --- evidence bags must describe themselves ---------------------------------
# Humble's sqlite3 writer records no message definitions, so a custom-typed bag
# can only be decoded by a reader that already knows helix_msgs. MCAP embeds the
# schemas, which is what an audit artifact needs: readable years later, by any
# reader, without the source tree that produced it.

def test_scenario_records_a_self_describing_bag_when_possible(monkeypatch):
    import ament_index_python.packages as packages

    from scripts.sim_faults.run_closed_loop_scenario import default_storage

    monkeypatch.setattr(packages, 'get_packages_with_prefixes',
                        lambda: {'rosbag2_storage_mcap': '/opt/ros/humble'})
    assert default_storage() == 'mcap'


def test_scenario_falls_back_when_the_mcap_plugin_is_absent(monkeypatch):
    import ament_index_python.packages as packages

    from scripts.sim_faults.run_closed_loop_scenario import default_storage

    monkeypatch.setattr(packages, 'get_packages_with_prefixes',
                        lambda: {'rosbag2_storage_default_plugins': '/opt/ros/humble'})
    assert default_storage() == 'sqlite3'


def test_record_command_passes_the_storage_plugin_through():
    from argparse import Namespace
    from pathlib import Path

    from scripts.sim_faults.run_closed_loop_scenario import build_commands

    args = Namespace(schedule='20:10', nav_speed=0.2, storage='mcap')
    record = build_commands(args, Path('/tmp/run'))['record']
    assert record[:5] == ['ros2', 'bag', 'record', '-s', 'mcap']


# --- the stop is measured while the stop is in force ------------------------
# R2 releases the hold once the anomaly has been clear for its quiet window,
# and /nav/cmd_vel then legitimately drives the robot again. A fixed window
# straddled that release whenever detection ran long, reporting a correctly
# resumed robot as a failure to stop.

def _records_with_resume(resume_at, resume_speed=0.22, n_moving=60):
    r = _passing_records()
    accept_ts = 26.2
    # At rest for the whole hold, then moving again after the release.
    odom = [{"ts": accept_ts + 1.0 + i * 0.02, "speed_m_s": 0.005} for i in range(120)]
    odom += [{"ts": resume_at + i * 0.02, "speed_m_s": resume_speed} for i in range(n_moving)]
    r["odom"] = r["odom"] + odom
    r["actions"] = r["actions"] + [{
        "ts": resume_at, "fault_id": "rate_hz/utlidar_cloud_throttled",
        "action": "RESUME", "status": "ACCEPTED",
    }]
    return r


def test_resumed_motion_after_the_release_is_not_a_failure_to_stop():
    result = evaluate(_records_with_resume(resume_at=28.4))
    stop = [c for c in result["checks"] if c["name"] == "odometry proves the robot stopped"][0]
    assert stop["passed"], stop["evidence"]
    assert stop["evidence"]["ended_at_release"] is True


def test_a_hold_too_short_to_measure_does_not_pass():
    # Released almost immediately: only a couple of samples fall in the window.
    result = evaluate(_records_with_resume(resume_at=27.25))
    stop = [c for c in result["checks"] if c["name"] == "odometry proves the robot stopped"][0]
    assert not stop["passed"], stop["evidence"]
    assert stop["evidence"]["observed_span_s"] < stop["evidence"]["minimum_span_s"]


def test_a_robot_that_never_stopped_still_fails():
    r = _records_with_resume(resume_at=40.0)
    r["odom"] = [o if o["ts"] < 27.0 else {**o, "speed_m_s": 0.25} for o in r["odom"]]
    result = evaluate(r)
    stop = [c for c in result["checks"] if c["name"] == "odometry proves the robot stopped"][0]
    assert not stop["passed"], stop["evidence"]
