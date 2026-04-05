"""
HeartbeatMonitor — HELIX Phase 1 fault sensing node.

Subscribes to /helix/heartbeat (std_msgs/String). Each managed node publishes
its own name there at 10 Hz. If a node misses `miss_threshold` consecutive
checks (at `check_interval_sec` interval) after `heartbeat_timeout_sec`, a
CRASH FaultEvent is emitted to /helix/faults.

Also publishes /helix/node_health (diagnostic_msgs/DiagnosticArray) every 1s.
"""
import threading
import time
from typing import Dict, Set, Tuple

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.timer import Timer
from std_msgs.msg import String

from helix_msgs.msg import FaultEvent

# ── Constants ────────────────────────────────────────────────────────────────
DEFAULT_TIMEOUT_SEC: float = 0.3
DEFAULT_MISS_THRESHOLD: int = 3
DEFAULT_CHECK_INTERVAL_SEC: float = 0.5
HEALTH_PUBLISH_INTERVAL_SEC: float = 1.0

STATUS_GREEN: int = DiagnosticStatus.OK
STATUS_YELLOW: int = DiagnosticStatus.WARN
STATUS_RED: int = DiagnosticStatus.ERROR


class HeartbeatMonitor(LifecycleNode):
    """Lifecycle node that monitors other nodes via periodic heartbeat messages."""

    def __init__(self) -> None:
        """Initialize node, declare parameters. Does NOT start timers yet."""
        super().__init__("helix_heartbeat_monitor")

        # Declare ROS 2 parameters with defaults
        self.declare_parameter("heartbeat_timeout_sec", DEFAULT_TIMEOUT_SEC)
        self.declare_parameter("miss_threshold", DEFAULT_MISS_THRESHOLD)
        self.declare_parameter("check_interval_sec", DEFAULT_CHECK_INTERVAL_SEC)

        # node_name -> (last_seen_timestamp, consecutive_miss_count)
        self._registry: Dict[str, Tuple[float, int]] = {}
        self._registry_lock: threading.Lock = threading.Lock()
        # Nodes for which a CRASH fault has already been emitted; prevents
        # flooding /helix/faults on every check cycle. Cleared when heartbeat resumes.
        self._crashed_nodes: Set[str] = set()

        self._fault_pub = None
        self._health_pub = None
        self._hb_sub = None
        self._check_timer: Timer | None = None
        self._health_timer: Timer | None = None

    # ── Lifecycle callbacks ──────────────────────────────────────────────────

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Read parameters, create publishers/subscribers. Called on configure transition."""
        self._timeout = self.get_parameter("heartbeat_timeout_sec").value
        self._miss_threshold = self.get_parameter("miss_threshold").value
        self._check_interval = self.get_parameter("check_interval_sec").value

        self._fault_pub = self.create_publisher(FaultEvent, "/helix/faults", 10)
        self._health_pub = self.create_publisher(
            DiagnosticArray, "/helix/node_health", 10
        )
        self._hb_sub = self.create_subscription(
            String, "/helix/heartbeat", self._on_heartbeat, 100
        )
        self.get_logger().info(
            f"HeartbeatMonitor configured — timeout={self._timeout}s "
            f"miss_threshold={self._miss_threshold} "
            f"check_interval={self._check_interval}s"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Start check timer and health timer."""
        self._check_timer = self.create_timer(
            self._check_interval, self._check_heartbeats
        )
        self._health_timer = self.create_timer(
            HEALTH_PUBLISH_INTERVAL_SEC, self._publish_health
        )
        self.get_logger().info("HeartbeatMonitor activated.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Stop timers."""
        if self._check_timer:
            self._check_timer.cancel()
        if self._health_timer:
            self._health_timer.cancel()
        self.get_logger().info("HeartbeatMonitor deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Destroy publishers/subscribers and clear registry."""
        if self._fault_pub:
            self.destroy_publisher(self._fault_pub)
        if self._health_pub:
            self.destroy_publisher(self._health_pub)
        if self._hb_sub:
            self.destroy_subscription(self._hb_sub)
        with self._registry_lock:
            self._registry.clear()
        self._crashed_nodes.clear()
        return TransitionCallbackReturn.SUCCESS

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _on_heartbeat(self, msg: String) -> None:
        """Record latest heartbeat timestamp for the publishing node."""
        node_name = msg.data.strip()
        if not node_name:
            return
        now = time.time()
        with self._registry_lock:
            self._registry[node_name] = (now, 0)
        # If this node had been marked crashed, clear it so a future outage
        # can emit a fresh CRASH fault.
        self._crashed_nodes.discard(node_name)
        self.get_logger().debug(f"Heartbeat received from '{node_name}'")

    def _check_heartbeats(self) -> None:
        """
        Periodic check: if a node has been silent for > timeout_sec on
        `miss_threshold` consecutive checks, emit a CRASH FaultEvent (once).
        """
        now = time.time()
        with self._registry_lock:
            snapshot = dict(self._registry)

        for node_name, (last_seen, miss_count) in snapshot.items():
            elapsed = now - last_seen
            if elapsed > self._timeout:
                new_miss_count = miss_count + 1
                with self._registry_lock:
                    # Guard against concurrent modification
                    if node_name in self._registry:
                        self._registry[node_name] = (last_seen, new_miss_count)

                self.get_logger().warn(
                    f"Node '{node_name}' has not sent a heartbeat for "
                    f"{elapsed:.2f}s (miss #{new_miss_count})"
                )

                if new_miss_count >= self._miss_threshold:
                    if node_name not in self._crashed_nodes:
                        self._crashed_nodes.add(node_name)
                        self._emit_crash_fault(node_name, elapsed, new_miss_count)
                    # else: already emitted; suppress until heartbeat resumes
            # else: node is healthy, miss count already reset in _on_heartbeat

    def _emit_crash_fault(
        self, node_name: str, elapsed: float, miss_count: int
    ) -> None:
        """Build and publish a CRASH FaultEvent for the given node."""
        msg = FaultEvent()
        msg.node_name = node_name
        msg.fault_type = "CRASH"
        msg.severity = 3
        msg.detail = (
            f"Node {node_name} missed {self._miss_threshold} heartbeats. "
            f"Last seen {elapsed:.2f}s ago."
        )
        msg.timestamp = time.time()
        msg.context_keys = ["elapsed_sec", "miss_count", "timeout_threshold"]
        msg.context_values = [
            str(round(elapsed, 3)),
            str(miss_count),
            str(self._timeout),
        ]
        self._fault_pub.publish(msg)
        self.get_logger().info(
            f"FaultEvent emitted: CRASH for '{node_name}' "
            f"(elapsed={elapsed:.2f}s, misses={miss_count})"
        )

    def _publish_health(self) -> None:
        """Publish DiagnosticArray summarising GREEN/YELLOW/RED per known node."""
        now = time.time()
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()

        with self._registry_lock:
            snapshot = dict(self._registry)

        for node_name, (last_seen, miss_count) in snapshot.items():
            elapsed = now - last_seen
            status = DiagnosticStatus()
            status.name = node_name
            status.values.append(
                KeyValue(key="elapsed_sec", value=str(round(elapsed, 3)))
            )
            status.values.append(
                KeyValue(key="miss_count", value=str(miss_count))
            )
            if elapsed < self._timeout:
                status.level = STATUS_GREEN
                status.message = "OK"
            elif miss_count < self._miss_threshold:
                status.level = STATUS_YELLOW
                status.message = "WARN: missed heartbeats"
            else:
                status.level = STATUS_RED
                status.message = "ERROR: node presumed crashed"
            array.status.append(status)

        self._health_pub.publish(array)


def main(args=None) -> None:
    """Entry point for helix_heartbeat_monitor console script."""
    rclpy.init(args=args)
    node = HeartbeatMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
