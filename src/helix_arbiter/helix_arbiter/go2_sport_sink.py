"""GO2 sport sink: arbiter output -> /api/sport/request (unitree_api/Request).

Single input (the arbiter output topic), single robot-facing publisher. Mode
is fixed at startup (read-only parameter) so the operator changes it only by
restarting the process:

  dry_run   (default) nothing is sent to the robot; every decision is traced
  stop_only only StopMove (1003) is ever sent
  armed     Move (1008) allowed within the sink limits

Every decision, in every mode, is published as JSON on /helix/sink/trace.
On SIGINT/SIGTERM a StopMove burst is sent before exit (not in dry_run).

Feedback (feedback:=true, needs unitree_api and unitree_go): the robot's
/api/sport/response is matched to this sink's own request ids, and
SportModeState gives measured motion. Both go out as helix_msgs/SinkStatus on
/helix/sink/status (20 Hz and on every change), stamped with this host's
monotonic clock. That is the controller ack and the measured stop the
supervisor needs before it calls a stop confirmed.
"""
from __future__ import annotations

import json
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from helix_arbiter.sport_sink_core import (
    MODE_DRY_RUN,
    MODES,
    SinkFeedback,
    SinkLimits,
    SinkLogic,
    StillThresholds,
    fill_unitree_request,
    stop_move,
)

IN_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE)
# The core trips the deadman strictly after input_timeout_sec, so the
# deadline timer fires just past it.
DEADLINE_MARGIN_SEC = 0.002
# Best effort accepts both reliable and best-effort publishers.
STATE_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                       reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE)
STATUS_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=20,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE)
TRACE_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE)


class Go2SportSink(Node):

    def __init__(self) -> None:
        super().__init__('helix_go2_sport_sink')
        ro = ParameterDescriptor(read_only=True)
        self.declare_parameter('mode', MODE_DRY_RUN, ro)
        self.declare_parameter('input_topic', '/cmd_vel', ro)
        self.declare_parameter('request_topic', '/api/sport/request', ro)
        self.declare_parameter('trace_topic', '/helix/sink/trace', ro)
        self.declare_parameter('max_vx', 0.25, ro)
        self.declare_parameter('max_vy', 0.20, ro)
        self.declare_parameter('max_wz', 0.50, ro)
        self.declare_parameter('input_timeout_sec', 0.25, ro)
        self.declare_parameter('feedback', True, ro)
        self.declare_parameter('response_topic', '/api/sport/response', ro)
        self.declare_parameter('state_topic', '/lf/sportmodestate', ro)
        self.declare_parameter('status_topic', '/helix/sink/status', ro)
        self.declare_parameter('still_max_speed_xy', 0.05, ro)
        self.declare_parameter('still_max_yaw_speed', 0.10, ro)
        self.declare_parameter('still_s', 0.3, ro)
        mode = self.get_parameter('mode').value
        if mode not in MODES:
            raise ValueError(f'mode must be one of {MODES}')
        self.logic = SinkLogic(
            mode,
            SinkLimits(self.get_parameter('max_vx').value,
                       self.get_parameter('max_vy').value,
                       self.get_parameter('max_wz').value),
            input_timeout_sec=self.get_parameter('input_timeout_sec').value)
        self._req_cls = None
        self._pub_req = None
        if mode != MODE_DRY_RUN:
            # Only needed when talking to the robot; absent on dev machines.
            from unitree_api.msg import Request
            self._req_cls = Request
            self._pub_req = self.create_publisher(
                Request, self.get_parameter('request_topic').value, 10)
        self._pub_trace = self.create_publisher(
            String, self.get_parameter('trace_topic').value, TRACE_QOS)
        self._req_id = int(time.time() * 1000) % 1_000_000_000
        self.fb = SinkFeedback(StillThresholds(
            self.get_parameter('still_max_speed_xy').value,
            self.get_parameter('still_max_yaw_speed').value,
            self.get_parameter('still_s').value))
        self._status_seq = 0
        self._pub_status = None
        if self.get_parameter('feedback').value:
            self._setup_feedback()
        self.create_subscription(
            Twist, self.get_parameter('input_topic').value, self._on_cmd, IN_QOS)
        # The 50 ms tick catches silence at startup and repeats StopMove, but
        # on its own it finds an input timeout up to 50 ms late. This timer is
        # restarted by every input and fires just past the timeout, so the
        # deadman trips at the deadline itself. Steady clock, like the logic.
        self.create_timer(0.05, self._on_tick)
        self._deadline = self.create_timer(
            self.logic.input_timeout_sec + DEADLINE_MARGIN_SEC, self._on_tick,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().warning(f'GO2 sport sink up in mode={mode}')

    def _setup_feedback(self) -> None:
        try:
            from helix_msgs.msg import SinkStatus
            from unitree_api.msg import Response
            from unitree_go.msg import SportModeState
        except ImportError as e:
            self.get_logger().error(f'feedback disabled, missing message package: {e}')
            return
        self._status_cls = SinkStatus
        self._pub_status = self.create_publisher(
            SinkStatus, self.get_parameter('status_topic').value, STATUS_QOS)
        self.create_subscription(
            Response, self.get_parameter('response_topic').value, self._on_response,
            STATE_QOS)
        self.create_subscription(
            SportModeState, self.get_parameter('state_topic').value, self._on_state,
            STATE_QOS)
        self.create_timer(0.05, self._publish_status)

    def _on_response(self, msg) -> None:
        was = self.fb.stop_acked
        mine = self.fb.on_response(msg.header.identity.id, msg.header.identity.api_id,
                                   msg.header.status.code, time.monotonic())
        if mine and self.fb.stop_acked and not was:
            self._publish_status()

    def _on_state(self, msg) -> None:
        was = self.fb.stopped
        self.fb.on_state(float(msg.velocity[0]), float(msg.velocity[1]),
                         float(msg.yaw_speed), int(msg.mode), time.monotonic())
        if self.fb.stopped != was:
            self._publish_status()

    def _publish_status(self) -> None:
        if self._pub_status is None:
            return
        fb, now = self.fb, time.monotonic()
        m = self._status_cls()
        m.mode = self.logic.mode
        m.feedback_available = fb.feedback_available(now)
        m.last_api_id = fb.last_api_id
        m.last_request_id = fb.last_request_id
        m.last_reason = fb.last_reason
        m.last_sent_at = fb.last_sent_at
        m.last_ack_request_id = fb.last_ack_request_id
        m.last_ack_code = fb.last_ack_code
        m.last_ack_at = fb.last_ack_at
        m.stop_acked = fb.stop_acked
        m.stop_ack_sent_at = fb.stop_ack_sent_at
        m.stop_acked_at = fb.stop_acked_at
        m.speed_xy = fb.speed_xy
        m.yaw_speed = fb.yaw_speed
        m.state_at = fb.state_at
        m.robot_mode = fb.robot_mode
        m.stopped = fb.stopped_now(now)
        m.stopped_since = fb.stopped_since
        self._status_seq += 1
        m.seq = self._status_seq
        self._pub_status.publish(m)

    def _on_cmd(self, msg: Twist) -> None:
        self._deadline.reset()
        self._send(self.logic.on_command(
            msg.linear.x, msg.linear.y, msg.angular.z, time.monotonic()),
            (msg.linear.x, msg.linear.y, msg.angular.z))

    def _on_tick(self) -> None:
        self._send(self.logic.on_tick(time.monotonic()), None)

    def _send(self, req, cmd_in) -> None:
        if req is None:
            return
        self._req_id += 1
        sent = False
        if self._pub_req is not None:
            self._pub_req.publish(fill_unitree_request(self._req_cls(), req, self._req_id))
            sent = True
            self.fb.on_sent(self._req_id, req.api_id, req.reason, time.monotonic())
        self._pub_trace.publish(String(data=json.dumps({
            't_wall': time.time(), 't_mono': time.monotonic(), 'mode': self.logic.mode,
            'api_id': req.api_id, 'reason': req.reason, 'x': req.x, 'y': req.y,
            'z': req.z, 'input': cmd_in, 'request_id': self._req_id,
            'sent_to_robot': sent})))
        if req.reason != 'MOVE':
            self.get_logger().info(f'StopMove ({req.reason}) sent_to_robot={sent}',
                                   throttle_duration_sec=1.0)

    def stop_burst(self, n: int = 3) -> None:
        for _ in range(n):
            self._send(stop_move('SHUTDOWN'), None)


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Go2SportSink()
    stop = {'sig': None}
    signal.signal(signal.SIGINT, lambda s, f: stop.__setitem__('sig', s))
    signal.signal(signal.SIGTERM, lambda s, f: stop.__setitem__('sig', s))
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    try:
        while stop['sig'] is None and rclpy.ok():
            ex.spin_once(timeout_sec=0.05)
        node.stop_burst()
        end = time.monotonic() + 0.2
        while time.monotonic() < end:
            ex.spin_once(timeout_sec=0.02)
    finally:
        ex.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
