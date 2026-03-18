"""
Tests for AnomalyDetector lifecycle node.

Publishes normal samples to build a window, then injects spikes.
Asserts a FaultEvent with fault_type=="ANOMALY" is emitted.
rclpy initialized by conftest.py session fixture.
"""
import time
import threading
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout


@pytest.fixture(scope="module")
def detector_node():
    """Create, configure, and activate the AnomalyDetector node."""
    from helix_core.anomaly_detector import AnomalyDetector

    node = AnomalyDetector()
    node.trigger_configure()
    node.trigger_activate()
    yield node
    node.trigger_deactivate()
    node.trigger_cleanup()
    node.destroy_node()


def _make_metric_msg(label: str, value: float) -> Float64MultiArray:
    """Helper: create a Float64MultiArray with the given label and single value."""
    msg = Float64MultiArray()
    dim = MultiArrayDimension()
    dim.label = label
    dim.size = 1
    dim.stride = 1
    msg.layout = MultiArrayLayout()
    msg.layout.dim = [dim]
    msg.data = [value]
    return msg


def test_anomaly_detection(detector_node):
    """
    Publish 15 normal samples with slight noise (to create non-zero std),
    then 3 spike samples (value=100.0). Assert ANOMALY FaultEvent emitted.
    """
    received_faults = []
    lock = threading.Lock()

    from helix_msgs.msg import FaultEvent
    sub_node = rclpy.create_node("test_anomaly_sub")
    sub_node.create_subscription(
        FaultEvent,
        "/helix/faults",
        lambda msg: (lock.acquire(), received_faults.append(msg), lock.release()),
        10,
    )

    pub_node = rclpy.create_node("test_metric_pub")
    pub = pub_node.create_publisher(Float64MultiArray, "/helix/metrics", 10)

    executor = SingleThreadedExecutor()
    executor.add_node(detector_node)
    executor.add_node(sub_node)
    executor.add_node(pub_node)

    # Publish 25 normal samples with slight noise so std > 1e-6.
    # N=25 ensures the asymptotic Z-score for the 3rd consecutive spike
    # (sqrt(N/2) ≈ 3.54) stays above the default threshold of 3.0.
    for i in range(25):
        pub.publish(_make_metric_msg("test_metric_noisy", 10.0 + (i % 3) * 0.1))
        executor.spin_once(timeout_sec=0.05)
        time.sleep(0.05)

    # Publish 3 spike samples — Z-score is computed against the baseline window
    # (before appending), so each spike tests against the stable historical mean.
    for _ in range(3):
        pub.publish(_make_metric_msg("test_metric_noisy", 100.0))
        executor.spin_once(timeout_sec=0.05)
        time.sleep(0.05)

    # Spin more to let the fault propagate
    for _ in range(20):
        executor.spin_once(timeout_sec=0.05)
        time.sleep(0.02)

    executor.remove_node(detector_node)
    executor.remove_node(sub_node)
    executor.remove_node(pub_node)
    sub_node.destroy_node()
    pub_node.destroy_node()

    anomaly_faults = [
        f for f in received_faults
        if f.fault_type == "ANOMALY" and "test_metric_noisy" in f.node_name
    ]
    assert len(anomaly_faults) >= 1, (
        f"Expected ANOMALY FaultEvent, got: {[(f.fault_type, f.node_name) for f in received_faults]}"
    )
    assert anomaly_faults[0].severity == 2
