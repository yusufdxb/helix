"""PoseDriftMonitor — HELIX adapter lifecycle node.

Subscribes to a ``geometry_msgs/PoseStamped`` topic, tracks 3D displacement
between consecutive poses, and publishes the instantaneous displacement rate
(m/s) as a labeled metric on ``/helix/metrics``. Stale (no pose update within
``stale_sec``) emits NaN.

Parameters:
    topic (str, default /utlidar/robot_pose)
    stale_sec (float, default 5.0): NaN-threshold for no-update staleness.
    min_dt (float, default 1e-3): drop updates closer than this in time.
    publish_period_sec (float, default 0.5)
    metric_name (str, default pose/displacement_rate_m_s)
"""
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from std_msgs.msg import Float64MultiArray

from helix_adapter._metrics import make_metric
from helix_adapter.pose_drift import DisplacementTracker


class PoseDriftMonitor(LifecycleNode):
    def __init__(self) -> None:
        super().__init__("helix_pose_drift_monitor")
        self.declare_parameter("topic", "/utlidar/robot_pose")
        self.declare_parameter("stale_sec", 5.0)
        self.declare_parameter("min_dt", 1e-3)
        self.declare_parameter("publish_period_sec", 0.5)
        self.declare_parameter("metric_name", "pose/displacement_rate_m_s")

        self._tracker = None
        self._sub = None
        self._metrics_pub = None
        self._timer = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._tracker = DisplacementTracker(
            stale_sec=float(self.get_parameter("stale_sec").value),
            min_dt=float(self.get_parameter("min_dt").value),
        )
        topic = str(self.get_parameter("topic").value)
        self._sub = self.create_subscription(
            PoseStamped, topic, self._on_pose, 10
        )
        self._metrics_pub = self.create_publisher(
            Float64MultiArray, "/helix/metrics", 10
        )
        self.get_logger().info(
            f"PoseDriftMonitor configured — topic={topic}"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        period = float(self.get_parameter("publish_period_sec").value)
        self._timer = self.create_timer(period, self._publish)
        self.get_logger().info("PoseDriftMonitor activated.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        if self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)
            self._timer = None
        self.get_logger().info("PoseDriftMonitor deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        if self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None
        if self._metrics_pub is not None:
            self.destroy_publisher(self._metrics_pub)
            self._metrics_pub = None
        self._tracker = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        return self.on_cleanup(state)

    def _on_pose(self, msg: PoseStamped) -> None:
        if self._tracker is None:
            return
        p = msg.pose.position
        self._tracker.update(p.x, p.y, p.z, t=time.monotonic())

    def _publish(self) -> None:
        if self._metrics_pub is None or self._tracker is None:
            return
        name = str(self.get_parameter("metric_name").value)
        self._metrics_pub.publish(
            make_metric(name, self._tracker.rate_or_nan(time.monotonic()))
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseDriftMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
