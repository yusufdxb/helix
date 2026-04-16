"""
Tests for LogParser lifecycle node.

Publishes a Log message matching the slam_diverged pattern.
Asserts FaultEvent with fault_type=="LOG_PATTERN" and rule_id=="slam_diverged".
rclpy initialized by conftest.py session fixture.
"""
import os
import threading
import time

import pytest
import rclpy
from rcl_interfaces.msg import Log
from rclpy.executors import SingleThreadedExecutor

# Locate log_rules.yaml in source tree (resolved to absolute path at import time)
RULES_FILE = os.path.abspath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "helix_bringup", "config", "log_rules.yaml"
))


@pytest.fixture(scope="module")
def parser_node():
    """Create, configure, and activate the LogParser node with rules file."""
    from helix_core.log_parser import LogParser

    node = LogParser()
    # Set rules_file_path before configure
    node.set_parameters([
        rclpy.parameter.Parameter(
            "rules_file_path",
            rclpy.parameter.Parameter.Type.STRING,
            RULES_FILE,  # already absolute from module-level constant
        )
    ])
    node.trigger_configure()
    node.trigger_activate()
    yield node
    node.trigger_deactivate()
    node.trigger_cleanup()
    node.destroy_node()


def _spin_until(executor, predicate, timeout_sec: float) -> bool:
    """Spin until predicate() is true or the deadline passes."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if predicate():
            return True
        executor.spin_once(timeout_sec=0.02)
    return predicate()


def test_log_pattern_detection(parser_node):
    """
    Publish a Log message matching slam_diverged rule.
    Assert FaultEvent with fault_type==LOG_PATTERN and matched_rule_id==slam_diverged.
    """
    received_faults = []
    lock = threading.Lock()

    from helix_msgs.msg import FaultEvent
    sub_node = rclpy.create_node("test_log_sub")
    sub_node.create_subscription(
        FaultEvent,
        "/helix/faults",
        lambda msg: (lock.acquire(), received_faults.append(msg), lock.release()),
        10,
    )

    pub_node = rclpy.create_node("test_log_pub")
    pub = pub_node.create_publisher(Log, "/rosout", 10)

    executor = SingleThreadedExecutor()
    executor.add_node(parser_node)
    executor.add_node(sub_node)
    executor.add_node(pub_node)

    # Wait for DDS discovery — LogParser must see our /rosout publisher AND
    # our FaultEvent subscriber must see LogParser's /helix/faults publisher.
    # Without this wait the single publish below races discovery and the test
    # flakes when run alongside other tests / launches.
    assert _spin_until(
        executor, lambda: pub.get_subscription_count() >= 1, timeout_sec=2.0
    ), "LogParser never discovered the /rosout publisher"

    log_msg = Log()
    log_msg.level = 40  # ERROR
    log_msg.name = "fake_slam_node"
    log_msg.msg = "SLAM diverged: covariance exceeded maximum threshold"
    log_msg.file = "slam_toolbox.cpp"
    log_msg.function = "update"
    log_msg.line = 42

    # Republish until a LOG_PATTERN FaultEvent shows up (or we time out). The
    # log_parser dedups identical (rule_id, node_name) within dedup_window_sec
    # so re-emitting only inflates the count if discovery was slow; the dedup
    # guarantees at most one FaultEvent for the matching window.
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        pub.publish(log_msg)
        for _ in range(5):
            executor.spin_once(timeout_sec=0.02)
        with lock:
            if any(f.fault_type == "LOG_PATTERN" for f in received_faults):
                break

    executor.remove_node(parser_node)
    executor.remove_node(sub_node)
    executor.remove_node(pub_node)
    sub_node.destroy_node()
    pub_node.destroy_node()

    log_faults = [f for f in received_faults if f.fault_type == "LOG_PATTERN"]
    assert len(log_faults) >= 1, (
        f"Expected LOG_PATTERN FaultEvent, got: {[(f.fault_type, f.node_name) for f in received_faults]}"
    )

    fault = log_faults[0]
    assert fault.node_name == "fake_slam_node"

    # Verify context contains matched_rule_id == slam_diverged
    rule_id_idx = None
    for i, key in enumerate(fault.context_keys):
        if key == "matched_rule_id":
            rule_id_idx = i
            break
    assert rule_id_idx is not None, "context_keys missing 'matched_rule_id'"
    assert fault.context_values[rule_id_idx] == "slam_diverged"
