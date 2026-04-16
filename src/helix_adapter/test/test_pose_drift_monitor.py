"""Integration tests for PoseDriftMonitor lifecycle node."""
import math
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float64MultiArray


@pytest.fixture
def node():
    from helix_adapter.pose_drift_monitor import PoseDriftMonitor

    n = PoseDriftMonitor()
    n.trigger_configure()
    n.trigger_activate()
    yield n
    n.trigger_deactivate()
    n.trigger_cleanup()
    n.destroy_node()


def _spin_until(executor, predicate, timeout_sec: float) -> bool:
    """Spin the executor until predicate() is true or the deadline passes.

    Used for DDS-discovery synchronization: in-process publishers/subscribers
    are not connected until both ends complete discovery, and a publish that
    happens before that is silently dropped. Without this wait the first 1–2
    publishes in a test race the discovery and the test flakes under load.
    """
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if predicate():
            return True
        executor.spin_once(timeout_sec=0.02)
    return predicate()


def test_publishes_displacement_rate_metric(node):
    received: list = []
    helper = rclpy.create_node("test_pose_helper")
    pose_pub = helper.create_publisher(PoseStamped, "/utlidar/robot_pose", 10)
    helper.create_subscription(
        Float64MultiArray, "/helix/metrics",
        lambda m: received.append((m.layout.dim[0].label, m.data[0])),
        10,
    )
    exec_ = SingleThreadedExecutor()
    exec_.add_node(node)
    exec_.add_node(helper)

    assert _spin_until(
        exec_, lambda: pose_pub.get_subscription_count() >= 1, timeout_sec=2.0
    ), "PoseDriftMonitor never discovered the pose publisher"

    # Move 1m in x, wait 0.1s between, expect ~10 m/s.
    pose = PoseStamped()
    pose.pose.position.x = 0.0
    pose_pub.publish(pose)
    start = time.monotonic()
    while time.monotonic() - start < 0.1:
        exec_.spin_once(timeout_sec=0.01)
    pose.pose.position.x = 1.0
    pose_pub.publish(pose)
    while time.monotonic() - start < 1.0:
        exec_.spin_once(timeout_sec=0.02)

    helper.destroy_node()
    exec_.shutdown()

    rates = [v for label, v in received
             if label == "pose/displacement_rate_m_s"]
    assert rates, "no pose/displacement metric received"
    finite = [r for r in rates if math.isfinite(r)]
    assert finite, "no finite displacement rate seen"
    # Rate should be nonzero and bounded (loose bounds — executor jitter).
    assert 1.0 < max(finite) < 100.0


def test_stale_pose_emits_nan(node):
    """With no updates, published rate is NaN."""
    received: list = []
    helper = rclpy.create_node("test_stale_helper")
    helper.create_subscription(
        Float64MultiArray, "/helix/metrics",
        lambda m: received.append(m.data[0]),
        10,
    )
    exec_ = SingleThreadedExecutor()
    exec_.add_node(node)
    exec_.add_node(helper)

    # Wait until at least one metric arrives, then collect a couple more so
    # the assertion has a real sample, not zero. publish_period_sec is 0.5s
    # by default, so allow up to 1.5s.
    _spin_until(exec_, lambda: len(received) >= 2, timeout_sec=1.5)

    helper.destroy_node()
    exec_.shutdown()

    assert received, "no metrics published"
    assert all(math.isnan(r) for r in received), received
