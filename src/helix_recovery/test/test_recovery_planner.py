"""
Integration test for RecoveryPlanner lifecycle node.

Publishes a FaultEvent to /helix/faults and asserts a RecoveryAction
arrives on /helix/recovery_actions within 15 seconds.
Also verifies state_db has a persisted attempt record.
"""
import os
import time
import tempfile
import threading
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from helix_msgs.msg import FaultEvent, RecoveryAction

# Path to the recovery_policy.yaml in the source tree
POLICY_FILE = os.path.abspath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "helix_bringup", "config", "recovery_policy.yaml"
))


@pytest.fixture(scope="module")
def planner_node(tmp_path_factory):
    """Create, configure, and activate the RecoveryPlanner with a temp DB."""
    from helix_recovery.recovery_planner import RecoveryPlanner

    db_path = str(tmp_path_factory.mktemp("helix_db") / "test_state.db")
    node = RecoveryPlanner(db_path_override=db_path)
    node.set_parameters([
        Parameter(
            "policy_file_path",
            Parameter.Type.STRING,
            POLICY_FILE,
        )
    ])
    node.trigger_configure()
    node.trigger_activate()
    yield node
    node.trigger_deactivate()
    node.trigger_cleanup()
    node.destroy_node()


def test_crash_fault_produces_recovery_action(planner_node):
    """
    Publish a CRASH FaultEvent for 'fake_nav_node'.
    Assert RecoveryAction arrives on /helix/recovery_actions within 15s.
    """
    received_actions = []
    lock = threading.Lock()

    sub_node = rclpy.create_node("test_recovery_sub")
    sub_node.create_subscription(
        RecoveryAction,
        "/helix/recovery_actions",
        lambda msg: (lock.acquire(), received_actions.append(msg), lock.release()),
        10,
    )

    pub_node = rclpy.create_node("test_fault_pub")
    fault_pub = pub_node.create_publisher(FaultEvent, "/helix/faults", 10)

    ex = SingleThreadedExecutor()
    ex.add_node(planner_node)
    ex.add_node(sub_node)
    ex.add_node(pub_node)

    spin_thread = threading.Thread(
        target=lambda: _spin_until_or_timeout(ex, received_actions, 15.0),
        daemon=True,
    )
    spin_thread.start()

    # Small delay so the executor is running before we publish
    time.sleep(0.3)

    fault_msg = FaultEvent()
    fault_msg.node_name = "fake_nav_node"
    fault_msg.fault_type = "CRASH"
    fault_msg.severity = 3
    fault_msg.detail = "Test CRASH fault from integration test"
    fault_msg.timestamp = time.time()
    fault_msg.context_keys = []
    fault_msg.context_values = []
    fault_pub.publish(fault_msg)

    spin_thread.join(timeout=15.0)

    ex.remove_node(planner_node)
    ex.remove_node(sub_node)
    ex.remove_node(pub_node)
    sub_node.destroy_node()
    pub_node.destroy_node()

    assert len(received_actions) >= 1, (
        "Expected at least 1 RecoveryAction message on /helix/recovery_actions"
    )
    action = received_actions[0]
    assert action.fault_type == "CRASH"
    assert action.node_name == "fake_nav_node"
    assert action.tier >= 1


def test_recovery_action_persisted_to_db(planner_node):
    """After test above, the state_db must contain at least one recovery attempt."""
    stats = planner_node._state_db.get_all_stats()
    assert stats["total_faults"] >= 1
    assert stats["total_recovery_attempts"] >= 1


def _spin_until_or_timeout(ex: SingleThreadedExecutor, results: list, timeout: float) -> None:
    deadline = time.time() + timeout
    while not results and time.time() < deadline:
        ex.spin_once(timeout_sec=0.1)
