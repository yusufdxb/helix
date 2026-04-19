"""
ContextBuffer — lifecycle node that maintains a bounded /rosout ring,
latest /helix/metrics snapshot, and latest /helix/node_health snapshot.
Serves GetContext srv.
"""
import json
import threading
from collections import deque
from typing import Deque, List

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.msg import Log
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from std_msgs.msg import Float64MultiArray

from helix_msgs.srv import GetContext

ROSOUT_RING_CAPACITY: int = 200


class RosoutRing:
    """Pure bounded ring — unit-testable without ROS 2."""

    def __init__(self, capacity: int = ROSOUT_RING_CAPACITY):
        self._buf: Deque[str] = deque(maxlen=capacity)
        self._lock = threading.Lock()

    def append(self, line: str) -> None:
        with self._lock:
            self._buf.append(line)

    def snapshot(self) -> List[str]:
        with self._lock:
            return list(self._buf)


class ContextBuffer(LifecycleNode):
    """Serves current context snapshot via GetContext service."""

    def __init__(self):
        super().__init__('helix_context_buffer')
        self._ring = RosoutRing()
        self._latest_metrics: str = '{}'
        self._latest_health: str = '{}'
        self._sub_rosout = None
        self._sub_metrics = None
        self._sub_health = None
        self._srv = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._srv = self.create_service(GetContext, '/helix/get_context', self._handle_get_context)
        self.get_logger().info('ContextBuffer configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._sub_rosout = self.create_subscription(Log, '/rosout', self._on_rosout, 100)
        self._sub_metrics = self.create_subscription(
            Float64MultiArray, '/helix/metrics', self._on_metrics, 10)
        self._sub_health = self.create_subscription(
            DiagnosticArray, '/helix/node_health', self._on_health, 10)
        self.get_logger().info('ContextBuffer active')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        for sub in (self._sub_rosout, self._sub_metrics, self._sub_health):
            if sub is not None:
                self.destroy_subscription(sub)
        self._sub_rosout = self._sub_metrics = self._sub_health = None
        return TransitionCallbackReturn.SUCCESS

    def _on_rosout(self, msg: Log) -> None:
        self._ring.append(f'[{msg.level}] {msg.name}: {msg.msg}')

    def _on_metrics(self, msg: Float64MultiArray) -> None:
        self._latest_metrics = json.dumps({'data': list(msg.data)})

    def _on_health(self, msg: DiagnosticArray) -> None:
        statuses = [{'name': s.name, 'level': int(s.level), 'message': s.message}
                    for s in msg.status]
        self._latest_health = json.dumps({'status': statuses})

    def _handle_get_context(self, request, response):
        response.rosout_ring = self._ring.snapshot()
        response.metrics_json = self._latest_metrics
        response.node_health_json = self._latest_health
        response.snapshot_time = self.get_clock().now().nanoseconds / 1e9
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ContextBuffer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
