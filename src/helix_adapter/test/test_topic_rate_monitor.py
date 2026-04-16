"""Integration tests for TopicRateMonitor lifecycle node.

Drives the node's subscriptions synthetically and verifies /helix/metrics
publishes the expected rate labels. Does not require a live ROS 2 graph —
uses an inline executor and rclpy.Node publishers.
"""
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, String


@pytest.fixture
def node():
    from helix_adapter.topic_rate_monitor import TopicRateMonitor

    n = TopicRateMonitor()
    n.set_parameters([
        rclpy.parameter.Parameter(
            "topics", rclpy.Parameter.Type.STRING_ARRAY,
            ["/utlidar/imu", "/gnss"],
        ),
        rclpy.parameter.Parameter(
            "window_sec", rclpy.Parameter.Type.DOUBLE, 2.0),
        rclpy.parameter.Parameter(
            "publish_period_sec", rclpy.Parameter.Type.DOUBLE, 0.1),
    ])
    n.trigger_configure()
    n.trigger_activate()
    yield n
    n.trigger_deactivate()
    n.trigger_cleanup()
    n.destroy_node()


def test_publishes_rate_metrics_for_subscribed_topics(node):
    """Publish IMU + GNSS messages, spin briefly, verify metrics emitted."""
    received: list = []
    helper = rclpy.create_node("test_rate_helper")
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    be_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

    imu_pub = helper.create_publisher(Imu, "/utlidar/imu", be_qos)
    gnss_pub = helper.create_publisher(String, "/gnss", 10)
    helper.create_subscription(
        Float64MultiArray, "/helix/metrics",
        lambda m: received.append(m.layout.dim[0].label),
        10,
    )

    exec_ = SingleThreadedExecutor()
    exec_.add_node(node)
    exec_.add_node(helper)

    # Pump 20 IMU + 20 GNSS messages over ~0.5s.
    start = time.monotonic()
    for i in range(20):
        imu_pub.publish(Imu())
        gnss_pub.publish(String(data='{"satellite_total": 5}'))
        exec_.spin_once(timeout_sec=0.02)
    # Drain the 0.1s publish timer at least twice.
    while time.monotonic() - start < 1.0:
        exec_.spin_once(timeout_sec=0.02)

    helper.destroy_node()
    exec_.shutdown()

    labels = set(received)
    assert any("rate_hz/utlidar_imu" == label for label in labels), labels
    assert any("rate_hz/gnss" == label for label in labels), labels


def test_unknown_topic_is_skipped_not_fatal():
    """Configuring an unknown topic logs an error but does not crash or add it."""
    from helix_adapter.topic_rate_monitor import TopicRateMonitor
    from rclpy.lifecycle import TransitionCallbackReturn

    n = TopicRateMonitor()
    n.set_parameters([
        rclpy.parameter.Parameter(
            "topics", rclpy.Parameter.Type.STRING_ARRAY,
            ["/utlidar/imu", "/unknown/topic"],
        ),
    ])
    assert n.trigger_configure() == TransitionCallbackReturn.SUCCESS
    assert "/utlidar/imu" in n._windows
    assert "/unknown/topic" not in n._windows
    n.trigger_cleanup()
    n.destroy_node()


def test_sim_mode_remaps_cloud_topic():
    """sim_mode=True swaps /utlidar/cloud -> /utlidar/cloud_throttled."""
    from helix_adapter.topic_rate_monitor import TopicRateMonitor

    n = TopicRateMonitor()
    n.set_parameters([
        rclpy.parameter.Parameter(
            "topics", rclpy.Parameter.Type.STRING_ARRAY,
            ["/utlidar/cloud"],
        ),
        rclpy.parameter.Parameter(
            "sim_mode", rclpy.Parameter.Type.BOOL, True),
    ])
    n.trigger_configure()
    assert "/utlidar/cloud_throttled" in n._windows
    assert "/utlidar/cloud" not in n._windows
    n.trigger_cleanup()
    n.destroy_node()
