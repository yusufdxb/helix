#!/usr/bin/env python3
"""Throttle-relay that simulates a LiDAR rate drop on /utlidar/cloud."""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


class ThrottleRelay(Node):
    """Relay /utlidar/cloud to /utlidar/cloud_throttled with scheduled Hz."""

    def __init__(self, schedule):
        super().__init__('lidar_throttle_relay')
        self._schedule = schedule       # list of (duration_s, target_hz)
        self._start = time.monotonic()
        self._last_publish = 0.0
        self._last_phase = None
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(
            PointCloud2, '/utlidar/cloud_throttled', sensor_qos
        )
        self._marker_pub = self.create_publisher(String, '/helix/experiment_marker', 10)
        self.create_subscription(
            PointCloud2, '/utlidar/cloud', self._on_msg, sensor_qos
        )
        self.create_timer(0.05, self._publish_phase_marker)
        self.get_logger().info(f'starting schedule: {schedule}')

    def _phase(self) -> tuple[int, float, float]:
        elapsed = time.monotonic() - self._start
        cumulative = 0.0
        for index, (dur, hz) in enumerate(self._schedule):
            cumulative += dur
            if elapsed < cumulative:
                return index, hz, elapsed
        return len(self._schedule) - 1, self._schedule[-1][1], elapsed

    def _publish_phase_marker(self) -> None:
        index, hz, elapsed = self._phase()
        if index == self._last_phase:
            return
        self._last_phase = index
        payload = {
            'schema_version': 1,
            'phase_index': index,
            'target_hz': hz,
            'elapsed_s': round(elapsed, 6),
        }
        self._marker_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        self.get_logger().info(f'experiment phase: {payload}')

    def _on_msg(self, msg: PointCloud2) -> None:
        _, hz, _ = self._phase()
        if hz <= 0:
            return   # drop entirely
        interval = 1.0 / hz
        now = time.monotonic()
        if now - self._last_publish < interval:
            return
        self._last_publish = now
        self._pub.publish(msg)


def _parse_schedule(arg: str):
    # Format: "30:10,20:1,30:10", using duration_s:hz pairs.
    out = []
    for piece in arg.split(','):
        dur, hz = piece.split(':')
        parsed = (float(dur), float(hz))
        if parsed[0] <= 0 or parsed[1] < 0:
            raise argparse.ArgumentTypeError(
                'schedule durations must be positive and rates must be nonnegative'
            )
        out.append(parsed)
    if not out:
        raise argparse.ArgumentTypeError('schedule must contain at least one phase')
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
