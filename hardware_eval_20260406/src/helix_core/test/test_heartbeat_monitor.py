"""
Tests for HeartbeatMonitor lifecycle node.

Uses pytest + rclpy directly. No pytest-ros needed.
rclpy is initialized once for the entire session via conftest.py (autouse).
"""
import time
import threading
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String


@pytest.fixture(scope="module")
def monitor_node():
    """Create, configure, and activate the HeartbeatMonitor node."""
    from helix_core.heartbeat_monitor import HeartbeatMonitor

    node = HeartbeatMonitor()
    node.trigger_configure()
    node.trigger_activate()
    yield node
    node.trigger_deactivate()
    node.trigger_cleanup()
    node.destroy_node()


def test_crash_detection(monitor_node):
    """
    Publish heartbeats for 'test_node', stop publishing, wait for timeout,
    then assert a CRASH FaultEvent arrives on /helix/faults with fault_type=='CRASH'
    and node_name=='test_node'.
    """
    received_faults = []
    lock = threading.Lock()

    from helix_msgs.msg import FaultEvent
    sub_node = rclpy.create_node("test_crash_sub")
    sub_node.create_subscription(
        FaultEvent,
        "/helix/faults",
        lambda msg: (lock.acquire(), received_faults.append(msg), lock.release()),
        10,
    )

    pub_node = rclpy.create_node("test_hb_pub")
    pub = pub_node.create_publisher(String, "/helix/heartbeat", 10)

    executor = SingleThreadedExecutor()
    executor.add_node(monitor_node)
    executor.add_node(sub_node)
    executor.add_node(pub_node)

    # Publish heartbeats at 10 Hz for 1 second
    start = time.time()
    while time.time() - start < 1.0:
        msg = String()
        msg.data = "test_node"
        pub.publish(msg)
        executor.spin_once(timeout_sec=0.1)

    # Stop publishing — wait 4 seconds for miss_threshold (3) to trigger
    deadline = time.time() + 4.0
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.05)

    executor.remove_node(monitor_node)
    executor.remove_node(sub_node)
    executor.remove_node(pub_node)
    sub_node.destroy_node()
    pub_node.destroy_node()

    crash_faults = [f for f in received_faults
                    if f.fault_type == "CRASH" and f.node_name == "test_node"]
    assert len(crash_faults) >= 1, (
        f"Expected at least 1 CRASH FaultEvent for 'test_node', got: "
        f"{[(f.fault_type, f.node_name) for f in received_faults]}"
    )
    assert crash_faults[0].severity == 3
