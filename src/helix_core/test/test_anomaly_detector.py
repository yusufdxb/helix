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
import rclpy.parameter
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout


@pytest.fixture(scope="module")
def detector_node():
    """Create, configure, and activate the AnomalyDetector node.

    min_anomaly_duration_s is set to 0.0 so that existing tests (which
    fire spikes in rapid succession) are not affected by the duration gate.
    """
    from helix_core.anomaly_detector import AnomalyDetector

    node = AnomalyDetector()
    # Override min_anomaly_duration_s to 0.0 for backward-compatible tests.
    node.set_parameters(
        [rclpy.parameter.Parameter("min_anomaly_duration_s", value=0.0)]
    )
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


# ── Min-anomaly-duration gate tests ─────────────────────────────────────
# These tests create fresh AnomalyDetector nodes with controlled
# min_anomaly_duration_s settings. They call _process_sample directly
# (unit-level) to avoid DDS timing flakiness.


def _make_duration_detector(min_duration: float):
    """Create, configure, and activate a fresh AnomalyDetector with the
    given min_anomaly_duration_s. Uses 0.0 consecutive_trigger so the
    gate is purely about duration. Returns the node.
    """
    from helix_core.anomaly_detector import AnomalyDetector

    node = AnomalyDetector()
    node.set_parameters([
        rclpy.parameter.Parameter("min_anomaly_duration_s", value=min_duration),
    ])
    node.trigger_configure()
    node.trigger_activate()
    return node


def _teardown_node(node):
    node.trigger_deactivate()
    node.trigger_cleanup()
    node.destroy_node()


def test_brief_spike_no_fault():
    """A brief spike (< min_anomaly_duration_s) must NOT emit a fault."""
    node = _make_duration_detector(min_duration=2.0)
    emitted = []
    original = node._fault_pub.publish
    node._fault_pub.publish = lambda msg: emitted.append(msg)
    try:
        metric = "test_brief_spike"
        # Build baseline
        for i in range(25):
            node._process_sample(metric, 10.0 + (i % 3) * 0.1)
        # 3 rapid spikes — consecutive_trigger met, but duration < 2.0s
        for _ in range(3):
            node._process_sample(metric, 100.0)
        assert len(emitted) == 0, (
            f"Expected no fault for brief spike, got {len(emitted)}"
        )
    finally:
        node._fault_pub.publish = original
        _teardown_node(node)


def test_sustained_anomaly_emits_fault():
    """A sustained anomaly (>= min_anomaly_duration_s) MUST emit a fault."""
    node = _make_duration_detector(min_duration=0.5)
    emitted = []
    original = node._fault_pub.publish
    node._fault_pub.publish = lambda msg: emitted.append(msg)
    try:
        metric = "test_sustained"
        # Build baseline
        for i in range(25):
            node._process_sample(metric, 10.0 + (i % 3) * 0.1)
        # First spike: starts the duration timer
        node._process_sample(metric, 100.0)
        # Wait past the duration gate
        time.sleep(0.6)
        # Two more spikes — now consecutive_trigger is met AND duration >= 0.5s
        node._process_sample(metric, 100.0)
        node._process_sample(metric, 100.0)
        assert len(emitted) >= 1, (
            f"Expected fault for sustained anomaly, got {len(emitted)}"
        )
    finally:
        node._fault_pub.publish = original
        _teardown_node(node)


def test_timer_resets_on_normal():
    """When metric returns to normal, the anomaly start timer must reset.
    A subsequent brief spike should not carry forward the old start time.
    """
    node = _make_duration_detector(min_duration=0.5)
    emitted = []
    original = node._fault_pub.publish
    node._fault_pub.publish = lambda msg: emitted.append(msg)
    try:
        metric = "test_timer_reset"
        # Build baseline
        for i in range(25):
            node._process_sample(metric, 10.0 + (i % 3) * 0.1)
        # Start an anomaly streak
        node._process_sample(metric, 100.0)
        # Wait so time has passed
        time.sleep(0.3)
        # Return to normal — resets the timer
        node._process_sample(metric, 10.1)
        # Verify timer was reset
        assert metric not in node._anomaly_start, (
            "Expected anomaly_start timer to be cleared after normal sample"
        )
        # New brief spike streak — should NOT emit because timer was reset
        # and the new streak hasn't lasted 0.5s
        node._process_sample(metric, 100.0)
        node._process_sample(metric, 100.0)
        node._process_sample(metric, 100.0)
        assert len(emitted) == 0, (
            f"Expected no fault after timer reset, got {len(emitted)}"
        )
    finally:
        node._fault_pub.publish = original
        _teardown_node(node)


def test_default_min_duration_is_2s():
    """The default min_anomaly_duration_s must be 2.0 seconds."""
    from helix_core.anomaly_detector import DEFAULT_MIN_ANOMALY_DURATION_S

    assert DEFAULT_MIN_ANOMALY_DURATION_S == 2.0


def test_zero_duration_disables_gate():
    """Setting min_anomaly_duration_s=0.0 must emit faults immediately
    (backwards-compatible with the pre-gate behavior).
    """
    node = _make_duration_detector(min_duration=0.0)
    emitted = []
    original = node._fault_pub.publish
    node._fault_pub.publish = lambda msg: emitted.append(msg)
    try:
        metric = "test_zero_duration"
        # Build baseline
        for i in range(25):
            node._process_sample(metric, 10.0 + (i % 3) * 0.1)
        # 3 rapid spikes — should emit immediately despite no elapsed time
        for _ in range(3):
            node._process_sample(metric, 100.0)
        assert len(emitted) >= 1, (
            f"Expected immediate fault with duration=0.0, got {len(emitted)}"
        )
    finally:
        node._fault_pub.publish = original
        _teardown_node(node)


def test_stale_nan_respects_duration_gate():
    """NaN (stale) path must also respect min_anomaly_duration_s."""
    node = _make_duration_detector(min_duration=2.0)
    emitted = []
    original = node._fault_pub.publish
    node._fault_pub.publish = lambda msg: emitted.append(msg)
    try:
        metric = "test_stale_duration"
        # 5 rapid NaN samples — consecutive_trigger met, but < 2.0s elapsed
        for _ in range(5):
            node._process_sample(metric, float("nan"))
        assert len(emitted) == 0, (
            f"Expected no stale fault for brief NaN burst, got {len(emitted)}"
        )
    finally:
        node._fault_pub.publish = original
        _teardown_node(node)
