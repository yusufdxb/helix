#!/usr/bin/env python3
"""Throttle-relay that simulates a LiDAR rate drop on /utlidar/cloud."""
import argparse
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class ThrottleRelay(Node):
    """Relay /utlidar/cloud → /utlidar/cloud_throttled with scheduled Hz."""

    def __init__(self, schedule):
        super().__init__('lidar_throttle_relay')
        self._schedule = schedule       # list of (duration_s, target_hz)
        self._current_hz = schedule[0][1]
        self._start = time.time()
        self._last_publish = 0.0
        self._pub = self.create_publisher(PointCloud2, '/utlidar/cloud_throttled', 10)
        self.create_subscription(PointCloud2, '/utlidar/cloud', self._on_msg, 10)
        self.get_logger().info(f'starting schedule: {schedule}')

    def _target_hz(self) -> float:
        elapsed = time.time() - self._start
        cumulative = 0.0
        for dur, hz in self._schedule:
            cumulative += dur
            if elapsed < cumulative:
                return hz
        return self._schedule[-1][1]

    def _on_msg(self, msg: PointCloud2) -> None:
        hz = self._target_hz()
        if hz <= 0:
            return   # drop entirely
        interval = 1.0 / hz
        now = time.time()
        if now - self._last_publish < interval:
            return
        self._last_publish = now
        self._pub.publish(msg)


def _parse_schedule(arg: str):
    # Format: "30:10,20:1,30:10" — duration_s:hz pairs.
    out = []
    for piece in arg.split(','):
        dur, hz = piece.split(':')
        out.append((float(dur), float(hz)))
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--schedule', default='30:10,20:1,30:10')
    args = parser.parse_args()

    rclpy.init()
    node = ThrottleRelay(_parse_schedule(args.schedule))
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
