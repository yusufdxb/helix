"""Pure regression tests for the physical-closure evidence gate."""

import sys
from types import ModuleType

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

def _install_fake_ament_index(monkeypatch, installed_packages):
    """Provide the one ament API used by the scenario without requiring ROS."""
    ament = ModuleType("ament_index_python")
    packages = ModuleType("ament_index_python.packages")
    packages.get_packages_with_prefixes = lambda: installed_packages
    ament.packages = packages
    monkeypatch.setitem(sys.modules, "ament_index_python", ament)
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", packages)


def test_scenario_records_a_self_describing_bag_when_possible(monkeypatch):
    _install_fake_ament_index(
        monkeypatch, {"rosbag2_storage_mcap": "/opt/ros/humble"}
    )

    from scripts.sim_faults.run_closed_loop_scenario import default_storage

    assert default_storage() == "mcap"


def test_scenario_falls_back_when_the_mcap_plugin_is_absent(monkeypatch):
    _install_fake_ament_index(
        monkeypatch, {"rosbag2_storage_default_plugins": "/opt/ros/humble"}
    )

    from scripts.sim_faults.run_closed_loop_scenario import default_storage

    assert default_storage() == "sqlite3"


def test_record_command_passes_the_storage_plugin_through():
    from argparse import Namespace
    from pathlib import Path

    from scripts.sim_faults.run_closed_loop_scenario import build_commands

    args = Namespace(schedule='20:10', nav_speed=0.2, storage='mcap')
    record = build_commands(args, Path('/tmp/run'))['record']
    assert record[:5] == ['ros2', 'bag', 'record', '-s', 'mcap']
