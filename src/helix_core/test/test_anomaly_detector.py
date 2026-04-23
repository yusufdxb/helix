"""
Tests for AnomalyDetector lifecycle node.

Publishes normal samples to build a window, then injects spikes.
Asserts a FaultEvent with fault_type=="ANOMALY" is emitted.
rclpy initialized by conftest.py session fixture.
"""
import threading
import time

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


def _spin_until(executor, predicate, timeout_sec: float) -> bool:
    """Spin until predicate() is true or the deadline passes."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if predicate():
            return True
        executor.spin_once(timeout_sec=0.02)
    return predicate()


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

    # Wait for both DDS links: detector must see our metric publisher, and our
    # FaultEvent subscriber must see the detector's publisher. Otherwise the
    # first samples / first FaultEvent race discovery and the assertion flakes.
    assert _spin_until(
        executor, lambda: pub.get_subscription_count() >= 1, timeout_sec=2.0
    ), "AnomalyDetector never discovered the metric publisher"

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


def test_stale_topic_fires_anomaly(detector_node):
    """NaN metric values (topic_rate_monitor's stale signal) must produce an
    ANOMALY FaultEvent tagged with violation_type='stale' in context.

    Regression coverage for the product gap found in Session 8 (2026-04-23):
    a silent topic produced 0 anomalies because z-score of NaN is NaN and
    failed every threshold comparison.

    This hits _process_sample directly (rather than round-tripping through
    /helix/metrics) because the node-level integration test above
    (test_anomaly_detection) is load-sensitive to whatever else is
    publishing on /helix/metrics during the run; unit-level coverage is
    stable and faster.
    """
    from helix_msgs.msg import FaultEvent

    emitted = []
    original = detector_node._fault_pub.publish
    detector_node._fault_pub.publish = lambda msg: emitted.append(msg)
    try:
        nan_label = "test_metric_stale_unit"
        for _ in range(5):
            detector_node._process_sample(nan_label, float("nan"))
    finally:
        detector_node._fault_pub.publish = original

    stale_faults = [
        f for f in emitted
        if f.fault_type == "ANOMALY" and f.node_name == nan_label
    ]
    assert len(stale_faults) >= 1, (
        f"Expected stale ANOMALY FaultEvent, got: "
        f"{[(f.fault_type, f.node_name) for f in emitted]}"
    )
    first = stale_faults[0]
    assert isinstance(first, FaultEvent)
    keys = list(first.context_keys)
    vals = list(first.context_values)
    assert "violation_type" in keys, (
        f"Expected 'violation_type' in context_keys, got {keys}"
    )
    assert vals[keys.index("violation_type")] == "stale"
    assert first.severity == 2
    assert "stale" in first.detail.lower()
    # NaN must NOT poison the window — no floats landed in the rolling buffer.
    assert nan_label in detector_node._windows
    assert len(detector_node._windows[nan_label]) == 0
