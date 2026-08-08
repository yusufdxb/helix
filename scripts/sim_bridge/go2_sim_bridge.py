#!/usr/bin/env python3
"""Bridge go2_omniverse (Isaac Sim) topic names onto the real GO2 names HELIX expects.

HELIX's SENSE tier and its physical-closure gate are written against the topic
names the real Unitree GO2 publishes (/utlidar/cloud, /utlidar/robot_odom) and
the muxed velocity topic the sport-mode bridge consumes (/cmd_vel).
go2_omniverse publishes and subscribes under a per-robot namespace instead
(robot0/point_cloud2, robot0/odom, robot0/cmd_vel). This node closes that gap
so the same HELIX stack, unmodified, runs against the simulator.

Three of the four mappings are pure renames. The fourth is not, and it is the
reason this is a node rather than a set of launch remappings:

    go2_omniverse's RobotBaseNode.publish_odom (ros2.py) populates only the
    pose half of nav_msgs/Odometry. It never writes twist. `grep -n twist
    ros2.py` returns nothing. The real GO2 fills twist, and HELIX's closure
    validator reads twist.twist.linear to decide whether the robot was moving
    before the fault and stopped after the recovery. Relayed verbatim, that
    field is identically zero and the odometry halves of the gate can never
    pass, no matter how correctly the robot actually moves.

So this bridge reconstructs twist by differentiating the simulator's own
ground-truth pose over a short window. That is the same quantity a real
odometry node reports, derived from the same source the simulator uses to draw
the robot. It supplies a field the simulator omits; it does not relax anything
the gate asks for.

Frames: go2_omniverse reports pose in the world/odom frame, so the linear
velocity written here is world-frame. The validator takes the magnitude of the
linear vector, which is frame-invariant, so this matches what the real GO2
odometry yields for the same motion.
"""

from __future__ import annotations

import argparse
import math
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

# go2_omniverse creates every publisher and subscriber with a bare
# QoSProfile(depth=10), which is RELIABLE / VOLATILE. Match it exactly on the
# simulator-facing side so discovery does not silently drop.
SIM_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

# scripts/sim_faults/inject_lidar_rate_drop.py subscribes to /utlidar/cloud
# with BEST_EFFORT sensor QoS, mirroring the real GO2 LiDAR driver. A RELIABLE
# publisher would still satisfy it, but publishing BEST_EFFORT keeps the sim
# path byte-for-byte equivalent to the hardware path.
SENSOR_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def _stamp_seconds(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class Go2SimBridge(Node):
    """Rename go2_omniverse topics to GO2 names and reconstruct odometry twist."""

    def __init__(self, robot_index: int, velocity_window_s: float) -> None:
        super().__init__("go2_sim_bridge")
        prefix = f"robot{robot_index}"
        self._velocity_window_s = velocity_window_s
        # (t, x, y, z, yaw) samples spanning at most velocity_window_s.
        self._history: deque[tuple[float, float, float, float, float]] = deque()
        self._cloud_count = 0
        self._odom_count = 0
        self._cmd_count = 0

        # Created lazily, on the first cloud actually received from the sim.
        #
        # This is deliberate and load-bearing. The closure test's
        # `require_sim_bridge` fixture decides whether to skip by counting
        # publishers on /utlidar/cloud. If this bridge advertised the topic
        # unconditionally, a simulator whose LiDAR is dead (attached but
        # producing an empty scan buffer, which is a real failure mode on
        # Isaac Sim 6.0) would still satisfy the guard. The gate would then run
        # against a topic that was never alive, "detect" it as stale, and go
        # green on evidence that does not exist. No cloud from the sim means no
        # publisher here, which means the gate skips honestly.
        self._cloud_pub = None
        self._odom_pub = self.create_publisher(Odometry, "/utlidar/robot_odom", SIM_QOS)
        self._cmd_pub = self.create_publisher(Twist, f"{prefix}/cmd_vel", SIM_QOS)

        self.create_subscription(
            PointCloud2, f"{prefix}/point_cloud2", self._on_cloud, SIM_QOS
        )
        self.create_subscription(Odometry, f"{prefix}/odom", self._on_odom, SIM_QOS)
        # /cmd_vel is twist_mux's muxed output. Routing it (and only it) into the
        # simulator is what makes the "robot stopped" half of the closure gate
        # meaningful: the zero that reaches the body is the arbitrated one.
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, SIM_QOS)

        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f"bridging {prefix}/point_cloud2 -> /utlidar/cloud, "
            f"{prefix}/odom -> /utlidar/robot_odom (twist reconstructed over "
            f"{velocity_window_s:.2f}s), /cmd_vel -> {prefix}/cmd_vel"
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        if msg.width == 0:
            # An attached-but-empty RTX scan buffer. Not evidence of a live sensor.
            return
        self._cloud_count += 1
        if self._cloud_pub is None:
            self._cloud_pub = self.create_publisher(PointCloud2, "/utlidar/cloud", SENSOR_QOS)
            self.get_logger().info(
                f"first non-empty cloud from the sim ({msg.width} points); "
                "advertising /utlidar/cloud"
            )
        self._cloud_pub.publish(msg)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_count += 1
        self._cmd_pub.publish(msg)

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_count += 1
        now = _stamp_seconds(msg.header)
        if now <= 0.0:
            now = self.get_clock().now().nanoseconds * 1e-9

        position = msg.pose.pose.position
        yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        self._history.append((now, position.x, position.y, position.z, yaw))
        # Keep one sample older than the window so the finite difference always
        # spans the full window once enough history exists.
        while len(self._history) > 2 and now - self._history[1][0] >= self._velocity_window_s:
            self._history.popleft()

        oldest = self._history[0]
        dt = now - oldest[0]
        if dt > 1e-6:
            msg.twist.twist.linear.x = (position.x - oldest[1]) / dt
            msg.twist.twist.linear.y = (position.y - oldest[2]) / dt
            msg.twist.twist.linear.z = (position.z - oldest[3]) / dt
            msg.twist.twist.angular.z = _wrap_to_pi(yaw - oldest[4]) / dt
        self._odom_pub.publish(msg)

    def _report(self) -> None:
        speed = 0.0
        if len(self._history) > 1:
            newest, oldest = self._history[-1], self._history[0]
            dt = newest[0] - oldest[0]
            if dt > 1e-6:
                speed = math.dist(newest[1:4], oldest[1:4]) / dt
        self.get_logger().info(
            f"relayed cloud={self._cloud_count} odom={self._odom_count} "
            f"cmd_vel={self._cmd_count} speed={speed:.3f} m/s"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-index", type=int, default=0)
    parser.add_argument(
        "--velocity-window-s",
        type=float,
        default=0.20,
        help="Finite-difference window for the reconstructed odometry twist. "
             "Long enough to smooth per-step pose jitter, short enough that a "
             "stop settles well inside the validator's post-action window.",
    )
    args = parser.parse_args()
    if args.velocity_window_s <= 0.0:
        parser.error("--velocity-window-s must be positive")

    rclpy.init()
    node = Go2SimBridge(args.robot_index, args.velocity_window_s)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
