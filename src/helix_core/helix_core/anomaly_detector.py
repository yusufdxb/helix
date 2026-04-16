"""
AnomalyDetector — HELIX Phase 1 fault sensing node.

Subscribes to /diagnostics (DiagnosticArray) and /helix/metrics (Float64MultiArray).
Maintains a rolling window of `window_size` samples per metric.
Emits ANOMALY FaultEvent when Z-score exceeds `zscore_threshold` for
`consecutive_trigger` consecutive samples.
"""
import math
import threading
import time
from collections import deque
from typing import Deque, Dict

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from std_msgs.msg import Float64MultiArray

from helix_msgs.msg import FaultEvent

# ── Constants ────────────────────────────────────────────────────────────────
DEFAULT_ZSCORE_THRESHOLD: float = 3.0
DEFAULT_CONSECUTIVE_TRIGGER: int = 3
DEFAULT_WINDOW_SIZE: int = 60
FLAT_SIGNAL_EPSILON: float = 1e-6


class AnomalyDetector(LifecycleNode):
    """Lifecycle node that detects statistical anomalies in numeric metrics."""

    def __init__(self) -> None:
        """Initialize node and declare parameters."""
        super().__init__("helix_anomaly_detector")

        self.declare_parameter("zscore_threshold", DEFAULT_ZSCORE_THRESHOLD)
        self.declare_parameter("consecutive_trigger", DEFAULT_CONSECUTIVE_TRIGGER)
        self.declare_parameter("window_size", DEFAULT_WINDOW_SIZE)

        # metric_name -> deque of float samples
        self._windows: Dict[str, Deque[float]] = {}
        # metric_name -> consecutive violation count
        self._consecutive: Dict[str, int] = {}
        self._data_lock: threading.Lock = threading.Lock()

        self._fault_pub = None
        self._diagnostics_sub = None
        self._metrics_sub = None

    # ── Lifecycle callbacks ──────────────────────────────────────────────────

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Read parameters, create publishers/subscribers."""
        self._zscore_threshold = self.get_parameter("zscore_threshold").value
        self._consecutive_trigger = self.get_parameter("consecutive_trigger").value
        self._window_size = self.get_parameter("window_size").value

        self._fault_pub = self.create_publisher(FaultEvent, "/helix/faults", 10)
        self._diagnostics_sub = self.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diagnostics, 10
        )
        self._metrics_sub = self.create_subscription(
            Float64MultiArray, "/helix/metrics", self._on_metric, 100
        )
        self.get_logger().info(
            f"AnomalyDetector configured — zscore_threshold={self._zscore_threshold} "
            f"consecutive_trigger={self._consecutive_trigger} "
            f"window_size={self._window_size}"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate the anomaly detector."""
        self.get_logger().info("AnomalyDetector activated.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate the anomaly detector."""
        self.get_logger().info("AnomalyDetector deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Destroy publishers/subscribers and clear data."""
        if self._fault_pub:
            self.destroy_publisher(self._fault_pub)
        if self._diagnostics_sub:
            self.destroy_subscription(self._diagnostics_sub)
        if self._metrics_sub:
            self.destroy_subscription(self._metrics_sub)
        with self._data_lock:
            self._windows.clear()
            self._consecutive.clear()
        return TransitionCallbackReturn.SUCCESS

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        """Extract numeric values from DiagnosticArray and run anomaly check."""
        for status in msg.status:
            for kv in status.values:
                try:
                    value = float(kv.value)
                except ValueError:
                    continue
                metric_name = f"{status.name}/{kv.key}"
                self._process_sample(metric_name, value)

    def _on_metric(self, msg: Float64MultiArray) -> None:
        """Process a metric from /helix/metrics; label comes from layout.dim[0].label."""
        if not msg.layout.dim:
            self.get_logger().warn(
                "Received Float64MultiArray with no dim labels — skipping"
            )
            return
        metric_name = msg.layout.dim[0].label
        if not metric_name:
            return
        for value in msg.data:
            self._process_sample(metric_name, value)

    def _process_sample(self, metric_name: str, value: float) -> None:
        """
        Compute Z-score against the existing window, then append new sample.

        Z-score is evaluated BEFORE appending so that anomalous values do not
        immediately contaminate the baseline statistics. This ensures consecutive
        violations remain detectable even when multiple spikes occur in sequence.
        """
        with self._data_lock:
            if metric_name not in self._windows:
                self._windows[metric_name] = deque(maxlen=self._window_size)
                self._consecutive[metric_name] = 0
            window = self._windows[metric_name]

            if len(window) >= 2:
                # Compute mean/std from historical samples (before this new value)
                samples = list(window)
                mean = sum(samples) / len(samples)
                variance = sum((s - mean) ** 2 for s in samples) / len(samples)
                std = math.sqrt(variance)

                if std < FLAT_SIGNAL_EPSILON:
                    self.get_logger().debug(
                        f"Metric '{metric_name}' is flat (std={std:.2e}) — skipping Z-score"
                    )
                else:
                    zscore = abs((value - mean) / std)

                    if zscore > self._zscore_threshold:
                        self._consecutive[metric_name] += 1
                        consecutive = self._consecutive[metric_name]

                        self.get_logger().warn(
                            f"Metric '{metric_name}' Z-score={zscore:.2f} "
                            f"(consecutive violation #{consecutive})"
                        )

                        if consecutive >= self._consecutive_trigger:
                            self._emit_anomaly_fault(
                                metric_name, value, mean, std, zscore, consecutive
                            )
                    else:
                        if self._consecutive[metric_name] > 0:
                            self.get_logger().debug(
                                f"Metric '{metric_name}' Z-score dropped to {zscore:.2f} — "
                                "resetting consecutive counter"
                            )
                        self._consecutive[metric_name] = 0

            # Always append after evaluating — keeps baseline from being poisoned
            window.append(value)

    def _emit_anomaly_fault(
        self,
        metric_name: str,
        value: float,
        mean: float,
        std: float,
        zscore: float,
        consecutive: int,
    ) -> None:
        """Build and publish an ANOMALY FaultEvent."""
        msg = FaultEvent()
        msg.node_name = metric_name  # spec: use metric_name as identifier for ANOMALY
        msg.fault_type = "ANOMALY"
        msg.severity = 2
        msg.detail = (
            f"Metric '{metric_name}' Z-score {zscore:.2f} exceeded threshold "
            f"on {self._consecutive_trigger} consecutive samples"
        )
        msg.timestamp = time.time()
        msg.context_keys = [
            "metric_name", "current_value", "window_mean",
            "window_std", "zscore", "consecutive_count",
        ]
        msg.context_values = [
            metric_name,
            str(round(value, 4)),
            str(round(mean, 4)),
            str(round(std, 6)),
            str(round(zscore, 2)),
            str(consecutive),
        ]
        self._fault_pub.publish(msg)
        self.get_logger().info(
            f"FaultEvent emitted: ANOMALY for '{metric_name}' "
            f"(zscore={zscore:.2f}, consecutive={consecutive})"
        )


def main(args=None) -> None:
    """Entry point for helix_anomaly_detector console script."""
    rclpy.init(args=args)
    node = AnomalyDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # try_shutdown is a no-op if rclpy's signal handler already shut the
        # context down (the common path under `ros2 launch` SIGINT). Plain
        # rclpy.shutdown() raises RCLError("rcl_shutdown already called") here.
        rclpy.try_shutdown()
