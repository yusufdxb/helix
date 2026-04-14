#!/usr/bin/env python3
"""
HELIX Passive Adapter — bridges GO2 standard topics to HELIX inputs.

Demonstrates "adapter-based observability" for non-standard robot platforms.
Requires NO changes to the GO2 software stack.

Three adapter channels:
  1. Topic Rate Monitor  — publishes rolling Hz for monitored topics to /helix/metrics
  2. JSON State Parser   — extracts numeric fields from /gnss, /multiplestate strings
  3. Pose Drift Monitor  — computes displacement rate from /utlidar/robot_pose

All derived metrics are published as Float64MultiArray on /helix/metrics,
compatible with HELIX's AnomalyDetector without modification.

Usage:
  source /opt/ros/humble/setup.bash
  source /tmp/helix_ws/install/setup.bash
  python3 scripts/passive_adapter.py
"""

import argparse
import json
import math
import time
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry


# ── Helpers ─────────────────────────────────────────────────────────────────

_FALSY_STRINGS = frozenset({"false", "0", "no", "off", "none", ""})


def _safe_float(value) -> float | None:
    """Convert *value* to float, returning None for non-finite or unparseable values.

    Prevents NaN/inf from reaching the anomaly detector's Z-score window,
    where they would corrupt the rolling mean and standard deviation.
    """
    try:
        f = float(value)
        return f if math.isfinite(f) else None
    except (ValueError, TypeError):
        return None


def _parse_bool_metric(value) -> float:
    """Convert a JSON boolean/int/string to 0.0 or 1.0.

    Handles the common GO2 patterns where boolean fields arrive as
    ``true``/``false`` (JSON bool), ``0``/``1`` (int), or their string
    representations.  Plain Python truthiness is wrong here because
    ``bool("false") is True``.
    """
    if isinstance(value, bool):
        return 1.0 if value else 0.0
    if isinstance(value, (int, float)):
        if isinstance(value, float) and not math.isfinite(value):
            return 0.0  # NaN / inf are not valid booleans
        return 0.0 if value == 0 else 1.0
    if isinstance(value, str):
        return 0.0 if value.strip().lower() in _FALSY_STRINGS else 1.0
    return 0.0


# ── Topic Rate Monitor ──────────────────────────────────────────────────────

class TopicRateMonitor:
    """Sliding-window **callback-arrival** rate estimator.

    Keeps the last ``window_sec`` seconds of ``time.monotonic()`` timestamps
    recorded at ROS 2 callback dispatch time, and computes instantaneous Hz as
    ``(n - 1) / span`` where *span* is the time between the oldest and newest
    timestamp in the window.

    **Important semantic caveat:** the timestamps are *receiver-side callback
    dispatch times*, **not** the source's ``header.stamp``.  Under executor
    queueing or scheduling jitter the measured rate can diverge from the true
    source publish rate — for example, burst-dequeued messages will appear
    closer together than they were actually published.  For ``std_msgs/String``
    topics (no header) this is the only option; for sensor topics with headers,
    a header-based estimator would be more accurate but is not implemented here.

    **Startup behaviour:** returns 0.0 until at least 2 messages have arrived
    within the window — callers should treat 0.0 during the first window as
    "insufficient data", not "topic is dead".

    **Stale detection:** if no message has arrived within the window,
    ``hz()`` returns 0.0 and ``is_stale()`` returns True.  Prefer
    ``rate_or_nan()`` to avoid TOCTOU between ``is_stale()`` and ``hz()``.
    """

    def __init__(self, window_sec: float = 5.0):
        self._window_sec = window_sec
        self._timestamps: deque = deque()
        self._lock = threading.Lock()

    def record(self):
        now = time.monotonic()
        with self._lock:
            self._timestamps.append(now)
            cutoff = now - self._window_sec
            while self._timestamps and self._timestamps[0] < cutoff:
                self._timestamps.popleft()

    def hz(self) -> float:
        now = time.monotonic()
        with self._lock:
            cutoff = now - self._window_sec
            while self._timestamps and self._timestamps[0] < cutoff:
                self._timestamps.popleft()
            n = len(self._timestamps)
            if n < 2:
                return 0.0
            span = self._timestamps[-1] - self._timestamps[0]
            if span < 1e-6:
                return 0.0
            return (n - 1) / span

    def is_stale(self) -> bool:
        """True when no messages have arrived within the window."""
        now = time.monotonic()
        with self._lock:
            cutoff = now - self._window_sec
            while self._timestamps and self._timestamps[0] < cutoff:
                self._timestamps.popleft()
            return len(self._timestamps) == 0

    def rate_or_nan(self) -> float:
        """Atomic combined stale-check + rate computation.

        Returns the rate in Hz if the window has >= 2 samples, or ``NaN``
        when the topic is stale (no messages within the window) or has
        insufficient data.  Using this instead of separate ``is_stale()`` /
        ``hz()`` calls avoids a TOCTOU race where the window state changes
        between the two calls.
        """
        now = time.monotonic()
        with self._lock:
            cutoff = now - self._window_sec
            while self._timestamps and self._timestamps[0] < cutoff:
                self._timestamps.popleft()
            n = len(self._timestamps)
            if n < 2:
                return math.nan if n == 0 else 0.0  # stale → NaN, startup → 0.0
            span = self._timestamps[-1] - self._timestamps[0]
            if span < 1e-6:
                return 0.0
            return (n - 1) / span


# ── Main Adapter Node ───────────────────────────────────────────────────────

class PassiveAdapter(Node):
    """
    Adapter node that translates GO2 standard topics into HELIX /helix/metrics.

    This node demonstrates that a monitoring architecture designed for standard
    ROS 2 inputs can observe a non-standard robot platform through lightweight
    adapters that require zero changes to the robot's software.
    """

    def __init__(self, sim: bool = False):
        super().__init__("helix_passive_adapter")
        self._sim = sim
        # In sim mode, the Isaac Sim bridge publishes synthetic /utlidar/cloud
        # and an injector relays to /utlidar/cloud_throttled. Monitor the
        # throttled topic so the fault-injection schedule is what
        # AnomalyDetector sees.
        self._utlidar_cloud_topic = (
            "/utlidar/cloud_throttled" if sim else "/utlidar/cloud"
        )

        # Publisher for derived metrics
        self._metrics_pub = self.create_publisher(
            Float64MultiArray, "/helix/metrics", 10
        )

        # ── Rate monitors ───────────────────────────────────────────────
        self._rate_monitors = {}
        rate_topics = {
            "/utlidar/robot_pose": PoseStamped,
            "/gnss": String,
            "/multiplestate": String,
        }

        # Try to subscribe to high-rate sensor topics with best-effort QoS
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        for topic, msg_type in rate_topics.items():
            monitor = TopicRateMonitor(window_sec=5.0)
            self._rate_monitors[topic] = monitor
            self.create_subscription(
                msg_type, topic,
                lambda msg, t=topic: self._on_rate_msg(t),
                10,
            )

        # Sensor topics often need best-effort QoS
        for topic, msg_type in [
            ("/utlidar/imu", Imu),
            (self._utlidar_cloud_topic, PointCloud2),
        ]:
            monitor = TopicRateMonitor(window_sec=5.0)
            self._rate_monitors[topic] = monitor
            self.create_subscription(
                msg_type, topic,
                lambda msg, t=topic: self._on_rate_msg(t),
                sensor_qos,
            )

        # ── JSON state parsers ──────────────────────────────────────────
        self._last_gnss = {}
        self._last_multiplestate = {}

        self.create_subscription(String, "/gnss", self._on_gnss, 10)
        self.create_subscription(
            String, "/multiplestate", self._on_multiplestate, 10
        )

        # ── Pose drift monitor ──────────────────────────────────────────
        self._last_pose = None
        self._last_pose_time = None
        self._displacement_rate = 0.0
        self._pose_stale_sec = 5.0  # same window as rate monitors

        self.create_subscription(
            PoseStamped, "/utlidar/robot_pose", self._on_pose, 10
        )

        # ── Periodic metric publisher (2 Hz) ────────────────────────────
        self._timer = self.create_timer(0.5, self._publish_metrics)

        self.get_logger().info(
            f"PassiveAdapter started — monitoring {len(self._rate_monitors)} "
            f"topics, 2 JSON streams, 1 pose stream"
        )

    # ── Callbacks ────────────────────────────────────────────────────────

    def _on_rate_msg(self, topic: str):
        self._rate_monitors[topic].record()

    def _on_gnss(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._last_gnss = data
        except (json.JSONDecodeError, TypeError):
            pass

    def _on_multiplestate(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._last_multiplestate = data
        except (json.JSONDecodeError, TypeError):
            pass

    def _on_pose(self, msg: PoseStamped):
        pos = msg.pose.position
        now = time.monotonic()

        if self._last_pose is not None and self._last_pose_time is not None:
            dt = now - self._last_pose_time
            if dt > 0.001:
                dx = pos.x - self._last_pose.x
                dy = pos.y - self._last_pose.y
                dz = pos.z - self._last_pose.z
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                self._displacement_rate = dist / dt

        self._last_pose = pos
        self._last_pose_time = now

    # ── Metric publishing ────────────────────────────────────────────────

    def _publish_metric(self, name: str, value: float):
        msg = Float64MultiArray()
        dim = MultiArrayDimension()
        dim.label = name
        dim.size = 1
        dim.stride = 1
        msg.layout.dim = [dim]
        msg.data = [value]
        self._metrics_pub.publish(msg)

    def _publish_metrics(self):
        """Publish all derived metrics at 2 Hz."""

        # 1. Topic rates — publish NaN when the topic is stale so downstream
        #    consumers can distinguish "no data" from "rate is 0 Hz".
        #    Uses rate_or_nan() for atomic stale-check + rate computation.
        for topic, monitor in self._rate_monitors.items():
            clean_name = topic.replace("/", "_").lstrip("_")
            self._publish_metric(f"rate_hz/{clean_name}", monitor.rate_or_nan())

        # 2. GNSS metrics — reject NaN/inf to protect downstream Z-score windows
        if self._last_gnss:
            for key in ["satellite_total", "satellite_inuse", "hdop"]:
                if key in self._last_gnss:
                    val = _safe_float(self._last_gnss[key])
                    if val is not None:
                        self._publish_metric(f"gnss/{key}", val)

        # 3. Multiplestate metrics — same NaN/inf guard
        if self._last_multiplestate:
            for key in ["volume", "brightness"]:
                if key in self._last_multiplestate:
                    val = _safe_float(self._last_multiplestate[key])
                    if val is not None:
                        self._publish_metric(f"go2_state/{key}", val)
            # Boolean as 0/1 — JSON values may arrive as bool, int, or string.
            for key in ["obstaclesAvoidSwitch", "uwbSwitch"]:
                if key in self._last_multiplestate:
                    val = _parse_bool_metric(self._last_multiplestate[key])
                    self._publish_metric(f"go2_state/{key}", val)

        # 4. Pose drift rate — publish NaN if no pose update within staleness window
        now = time.monotonic()
        if (self._last_pose_time is not None
                and (now - self._last_pose_time) < self._pose_stale_sec):
            self._publish_metric("pose/displacement_rate_m_s", self._displacement_rate)
        else:
            self._publish_metric("pose/displacement_rate_m_s", math.nan)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sim",
        action="store_true",
        help="Monitor /utlidar/cloud_throttled (Isaac Sim harness) "
             "instead of the real /utlidar/cloud.",
    )
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = PassiveAdapter(sim=args.sim)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
