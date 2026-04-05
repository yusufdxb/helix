"""
FaultInjector — HELIX demo node.

Simulates all three fault types in sequence to verify the sensing pipeline:
  1. Heartbeat injection: publishes fake_nav_node heartbeat at 10 Hz for 4s, then stops.
  2. Metric spike injection: 25 normal values then 3 large spikes.
  3. Log pattern injection: fake SLAM divergence ERROR to /rosout.

Run with: ros2 run helix_bringup helix_fault_injector

The node is spun in a background thread so DDS can deliver messages
while the injection sequence runs in the main thread.
"""
import threading
import time

import rclpy
from rcl_interfaces.msg import Log
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import (
    Float64MultiArray,
    MultiArrayDimension,
    MultiArrayLayout,
    String,
)

# ── Constants ────────────────────────────────────────────────────────────────
HEARTBEAT_TOPIC: str = "/helix/heartbeat"
METRICS_TOPIC: str = "/helix/metrics"
ROSOUT_TOPIC: str = "/rosout"

HEARTBEAT_RATE_HZ: float = 10.0
HEARTBEAT_DURATION_SEC: float = 4.0
POST_STOP_WAIT_SEC: float = 3.0

NORMAL_VALUE: float = 10.0
NORMAL_SAMPLE_COUNT: int = 25    # ≥19 so Z-score asymptote > 3.0 for 3rd spike
SPIKE_VALUE: float = 100.0
SPIKE_SAMPLE_COUNT: int = 3
METRIC_PUBLISH_RATE_HZ: float = 2.0

GAP_BETWEEN_PHASES_SEC: float = 2.0


class FaultInjector(Node):
    """Plain rclpy Node that sequentially injects all three fault types."""

    def __init__(self) -> None:
        """Initialize publishers."""
        super().__init__("helix_fault_injector")
        self._hb_pub = self.create_publisher(String, HEARTBEAT_TOPIC, 10)
        self._metric_pub = self.create_publisher(Float64MultiArray, METRICS_TOPIC, 10)
        self._log_pub = self.create_publisher(Log, ROSOUT_TOPIC, 10)

    def run_injection_sequence(self) -> None:
        """Execute heartbeat, metric spike, and log pattern injections in order."""
        self._inject_heartbeat_then_stop()
        time.sleep(GAP_BETWEEN_PHASES_SEC)

        self._inject_metric_spike()
        time.sleep(GAP_BETWEEN_PHASES_SEC)

        self._inject_log_pattern()
        self.get_logger().info("All fault injections complete. Watch /helix/faults.")

    def _inject_heartbeat_then_stop(self) -> None:
        """
        Publish heartbeat for fake_nav_node at 10 Hz for 4 seconds,
        then stop completely. The heartbeat monitor should detect a CRASH.
        """
        print(
            "\n[FaultInjector] Phase 1: Heartbeat injection starting — "
            "publishing fake_nav_node at 10 Hz for 4s then stopping.\n"
            f"Watch for CRASH FaultEvent on /helix/faults in ~{POST_STOP_WAIT_SEC}s after stop."
        )
        msg = String()
        msg.data = "fake_nav_node"

        interval = 1.0 / HEARTBEAT_RATE_HZ
        deadline = time.time() + HEARTBEAT_DURATION_SEC
        while time.time() < deadline:
            self._hb_pub.publish(msg)
            time.sleep(interval)

        print(f"[FaultInjector] Heartbeat stopped. Waiting {POST_STOP_WAIT_SEC}s...")
        time.sleep(POST_STOP_WAIT_SEC)

    def _inject_metric_spike(self) -> None:
        """
        Publish normal metric values, then inject clear spikes.
        The anomaly detector should emit an ANOMALY FaultEvent.
        """
        print(
            "\n[FaultInjector] Phase 2: Metric spike injection — "
            f"{NORMAL_SAMPLE_COUNT} normal samples (mean={NORMAL_VALUE}), "
            f"then {SPIKE_SAMPLE_COUNT} spikes (value={SPIKE_VALUE}).\n"
            "Watch for ANOMALY FaultEvent on /helix/faults."
        )
        interval = 1.0 / METRIC_PUBLISH_RATE_HZ

        for i in range(NORMAL_SAMPLE_COUNT):
            noise = 0.1 * ((i % 3) - 1)  # cycles: -0.1, 0, +0.1
            self._metric_pub.publish(
                self._make_metric_msg("cpu_load", NORMAL_VALUE + noise)
            )
            time.sleep(interval)

        print(f"[FaultInjector] Injecting {SPIKE_SAMPLE_COUNT} spike samples...")
        for _ in range(SPIKE_SAMPLE_COUNT):
            self._metric_pub.publish(self._make_metric_msg("cpu_load", SPIKE_VALUE))
            time.sleep(interval)

    def _inject_log_pattern(self) -> None:
        """
        Publish a fake /rosout ERROR log matching the slam_diverged rule.
        The log parser should emit a LOG_PATTERN FaultEvent.
        """
        print(
            "\n[FaultInjector] Phase 3: Log pattern injection — "
            "publishing fake SLAM divergence ERROR to /rosout.\n"
            "Watch for LOG_PATTERN FaultEvent on /helix/faults."
        )
        log_msg = Log()
        log_msg.stamp.sec = int(time.time())
        log_msg.level = 40  # ERROR
        log_msg.name = "fake_slam_node"
        log_msg.msg = "SLAM diverged: covariance exceeded maximum threshold"
        log_msg.file = "slam_toolbox.cpp"
        log_msg.function = "processScan"
        log_msg.line = 247
        self._log_pub.publish(log_msg)

    @staticmethod
    def _make_metric_msg(label: str, value: float) -> Float64MultiArray:
        """Create a Float64MultiArray metric message with given label and value."""
        msg = Float64MultiArray()
        dim = MultiArrayDimension()
        dim.label = label
        dim.size = 1
        dim.stride = 1
        msg.layout = MultiArrayLayout()
        msg.layout.dim = [dim]
        msg.data = [value]
        return msg


def main(args=None) -> None:
    """Entry point for helix_fault_injector console script."""
    rclpy.init(args=args)
    node = FaultInjector()

    # Spin the node in a background thread so DDS can deliver published messages
    # while run_injection_sequence() blocks in the main thread.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_injection_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
