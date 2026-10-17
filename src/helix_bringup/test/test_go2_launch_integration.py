"""Run the installed preparation graph on localhost with no hardware inputs."""
import os
import signal
import subprocess
import time
from pathlib import Path

import pytest

pytest.importorskip("rclpy")

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from lifecycle_msgs.srv import GetState  # noqa: E402


@pytest.mark.timeout(60)
def test_installed_go2_profile_activates_monitors_without_actuation(tmp_path):
    # Protect this integration test even when invoked outside deploy_go2.sh.
    env = os.environ.copy()
    offline = Path(__file__).resolve().parents[1] / "config/cyclonedds_offline.xml"
    env.update(RMW_IMPLEMENTATION="rmw_cyclonedds_cpp", ROS_LOCALHOST_ONLY="0",
               CYCLONEDDS_URI=f"file://{offline}", ROS_DOMAIN_ID="89")
    old = {key: os.environ.get(key) for key in env if key in (
        "RMW_IMPLEMENTATION", "ROS_LOCALHOST_ONLY", "CYCLONEDDS_URI", "ROS_DOMAIN_ID")}
    for key in old:
        os.environ[key] = env[key]
    log_path = tmp_path / "go2-preparation.log"
    process = None
    node = None
    outputs = []
    try:
        rclpy.init()
        node = rclpy.create_node("helix_preparation_test")
        node.create_subscription(Twist, "/helix_dry/cmd_vel", outputs.append, 100)
        with log_path.open("w") as log:
            process = subprocess.Popen(
                ["ros2", "launch", "helix_bringup", "helix_go2.launch.py"],
                stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True,
            )
        expected = {
            "helix_heartbeat_monitor": "active",
            "helix_anomaly_detector": "active",
            "helix_log_parser": "active",
            "helix_topic_rate_monitor": "active",
            "helix_json_state_parser": "active",
            "helix_pose_drift_monitor": "active",
            "helix_context_buffer": "active",
            "helix_diagnosis_node": "active",
            "helix_recovery_node": "unconfigured",
            "helix_arbiter": "active",
        }
        clients = {name: node.create_client(GetState, f"/{name}/get_state") for name in expected}
        states = {}
        pending = {}
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            assert process.poll() is None, log_path.read_text()
            rclpy.spin_once(node, timeout_sec=0.05)
            for name, client in clients.items():
                future = pending.get(name)
                if future is not None and future.done():
                    states[name] = future.result().current_state.label
                    pending.pop(name)
                if name not in pending and states.get(name) != expected[name] and client.service_is_ready():
                    pending[name] = client.call_async(GetState.Request())
            if states == expected and len(outputs) >= 5:
                break
        assert states == expected, (states, log_path.read_text())
        assert len(outputs) >= 5, log_path.read_text()
        assert all(getattr(getattr(msg, vector), axis) == 0.0
                   for msg in outputs for vector in ("linear", "angular") for axis in ("x", "y", "z"))
        assert "helix_go2_sport_sink" in node.get_node_names()
        assert not node.get_publishers_info_by_topic("/api/sport/request")
        assert not node.get_publishers_info_by_topic("/cmd_vel")
        assert not node.get_publishers_info_by_topic("/helix/cmd_vel")
        assert not node.get_publishers_info_by_topic("/helix/hold")
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for key, value in old.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
