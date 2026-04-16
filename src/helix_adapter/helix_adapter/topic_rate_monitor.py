"""TopicRateMonitor — HELIX adapter lifecycle node.

Subscribes to configured topics, tracks callback-arrival rate per topic over
a rolling window, and publishes each topic's Hz as a labeled scalar metric on
``/helix/metrics``. Stale topics (no message in window) emit NaN so downstream
consumers can distinguish "no data" from "0 Hz".

Parameters:
    window_sec (float, default 5.0): rolling window for rate estimation.
    publish_period_sec (float, default 0.5): metric publish cadence.
    topics (list[str], default GO2 defaults): topics to subscribe to.
    sim_mode (bool, default False): if true, remap /utlidar/cloud to
        /utlidar/cloud_throttled for Isaac Sim fault-injection harness.
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Float64MultiArray, String

from helix_adapter._metrics import make_metric
from helix_adapter.rate_window import RateWindow

# Default topic set for the GO2: rates confirmed from hardware_eval bag captures.
_DEFAULT_TOPICS = [
    "/utlidar/imu",
    "/utlidar/robot_odom",
    "/utlidar/robot_pose",
    "/utlidar/cloud",
    "/gnss",
    "/multiplestate",
]

# ROS message type registry — kept small & explicit so failing imports surface
# at configure time, not later as KeyError in a callback.
_TYPE_MAP = {
    "/utlidar/imu": Imu,
    "/utlidar/robot_odom": Odometry,
    "/utlidar/robot_pose": PoseStamped,
    "/utlidar/cloud": PointCloud2,
    "/utlidar/cloud_throttled": PointCloud2,
    "/gnss": String,
    "/multiplestate": String,
}

# Sensor topics use best-effort QoS on the GO2.
_BEST_EFFORT_TYPES = {Imu, PointCloud2}


class TopicRateMonitor(LifecycleNode):
    def __init__(self) -> None:
        super().__init__("helix_topic_rate_monitor")
        self.declare_parameter("window_sec", 5.0)
        self.declare_parameter("publish_period_sec", 0.5)
        self.declare_parameter("topics", _DEFAULT_TOPICS)
        self.declare_parameter("sim_mode", False)

        self._windows: dict = {}
        self._subs: list = []
        self._metrics_pub = None
        self._timer = None

    def _resolve_topics(self) -> list:
        topics = list(self.get_parameter("topics").value)
        if self.get_parameter("sim_mode").value:
            topics = [
                "/utlidar/cloud_throttled" if t == "/utlidar/cloud" else t
                for t in topics
            ]
        return topics

    def _qos_for(self, msg_type) -> int:
        if msg_type in _BEST_EFFORT_TYPES:
            return QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
        return 10

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        window_sec = float(self.get_parameter("window_sec").value)
        topics = self._resolve_topics()

        for topic in topics:
            if topic not in _TYPE_MAP:
                self.get_logger().error(
                    f"Unknown topic type for {topic!r}; skipping")
                continue
            self._windows[topic] = RateWindow(window_sec=window_sec)
            msg_type = _TYPE_MAP[topic]
            sub = self.create_subscription(
                msg_type, topic,
                lambda _msg, t=topic: self._windows[t].record(),
                self._qos_for(msg_type),
            )
            self._subs.append(sub)

        self._metrics_pub = self.create_publisher(
            Float64MultiArray, "/helix/metrics", 10)

        self.get_logger().info(
            f"TopicRateMonitor configured — {len(self._windows)} topics, "
            f"window={window_sec}s"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        period = float(self.get_parameter("publish_period_sec").value)
        self._timer = self.create_timer(period, self._publish)
        self.get_logger().info("TopicRateMonitor activated.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        if self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)
            self._timer = None
        self.get_logger().info("TopicRateMonitor deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        for sub in self._subs:
            self.destroy_subscription(sub)
        self._subs.clear()
        self._windows.clear()
        if self._metrics_pub is not None:
            self.destroy_publisher(self._metrics_pub)
            self._metrics_pub = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        return self.on_cleanup(state)

    def _publish(self) -> None:
        if self._metrics_pub is None:
            return
        for topic, window in self._windows.items():
            clean = topic.replace("/", "_").lstrip("_")
            self._metrics_pub.publish(
                make_metric(f"rate_hz/{clean}", window.rate_or_nan())
            )


def main(args=None):
    rclpy.init(args=args)
    node = TopicRateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
