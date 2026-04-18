#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


class Pub(Node):
    def __init__(self):
        super().__init__("bench_pub")
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(Imu, "/bench/imu", qos)
        self.msg = Imu()
        self.msg.header.frame_id = "bench_imu"
        self.msg.orientation.w = 1.0
        self.msg.orientation_covariance = [0.01] * 9
        self.msg.angular_velocity_covariance = [0.01] * 9
        self.msg.linear_acceleration_covariance = [0.01] * 9
        self.timer = self.create_timer(1.0 / 250.0, self.tick)
        self.sent = 0

    def tick(self):
        now = self.get_clock().now().to_msg()
        self.msg.header.stamp = now
        self.pub.publish(self.msg)
        self.sent += 1


def main():
    rclpy.init()
    node = Pub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"published={node.sent}", flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
