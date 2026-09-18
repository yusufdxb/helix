"""helix_arbiter lifecycle node: the single authoritative velocity output.

Wraps arbiter_core.Arbiter. Subscribes to every configured upstream source
and to /helix/hold, and publishes the selected command on ``output_topic``
at a FIXED rate (unlike twist_mux, which is event-driven and goes silent on
idle or lock). Every tick also publishes an ArbiterStatus for tracing.

Shutdown safety: rclpy's default SIGINT/SIGTERM handlers shut the context
down before user code runs, after which nothing can be published. main()
therefore installs its own handlers, and on SIGINT, SIGTERM or lifecycle
deactivate the node publishes ``shutdown_zero_count`` zero commands before
going silent. A downstream sink must still carry its own deadman for the
SIGKILL / power-loss case, which no process can handle for itself.
"""
from __future__ import annotations

import signal
import time
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from helix_arbiter.arbiter_core import (
    REASON_SHUTDOWN,
    ZERO,
    Arbiter,
    Command,
    Decision,
    Limits,
    SourceSpec,
)
from helix_msgs.msg import ArbiterStatus, HelixHold

# BEST_EFFORT subscriptions match both reliable and best-effort publishers.
INPUT_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                       reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE)
# RELIABLE/VOLATILE output matches reliable and best-effort subscribers.
OUTPUT_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE)
STATUS_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE)


def to_twist(cmd: Command) -> Twist:
    t = Twist()
    t.linear.x, t.linear.y, t.angular.z = cmd.vx, cmd.vy, cmd.wz
    return t


class ArbiterNode(LifecycleNode):

    def __init__(self, **kwargs) -> None:
        super().__init__(
            'helix_arbiter',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
            **kwargs)
        for name, default in (
                ('output_topic', '/cmd_vel'),
                ('status_topic', '/helix/arbiter/status'),
                ('hold_topic', '/helix/hold'),
                ('rate_hz', 50.0),
                ('hold_timeout_sec', 0.5),
                ('max_abs_linear', 1.0),
                ('max_abs_angular', 1.5),
                ('shutdown_zero_count', 10),
                ('autostart', False)):
            if not self.has_parameter(name):
                self.declare_parameter(name, default)
        self._arb: Optional[Arbiter] = None
        self._pub_out = None
        self._pub_status = None
        self._subs: List = []
        self._timer = None
        self._seq = 0
        self._last_reason: Optional[str] = None
        self._last_source: Optional[str] = None
        self._active = False

    # -- lifecycle ------------------------------------------------------------

    def _source_specs(self) -> List[SourceSpec]:
        raw: Dict[str, Dict[str, object]] = {}
        for key, param in self.get_parameters_by_prefix('sources').items():
            name, _, field = key.partition('.')
            raw.setdefault(name, {})[field] = param.value
        specs = []
        for name, cfg in sorted(raw.items()):
            specs.append(SourceSpec(name, str(cfg['topic']), int(cfg['priority']),
                                    float(cfg['timeout'])))
        return specs

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        try:
            specs = self._source_specs()
            self._arb = Arbiter(
                specs,
                hold_timeout_sec=float(self.get_parameter('hold_timeout_sec').value),
                limits=Limits(float(self.get_parameter('max_abs_linear').value),
                              float(self.get_parameter('max_abs_angular').value)))
        except (KeyError, ValueError, TypeError) as exc:
            self.get_logger().error(f'bad arbiter configuration: {exc!r}')
            return TransitionCallbackReturn.FAILURE
        out = self.get_parameter('output_topic').value
        for s in specs:
            if s.topic == out:
                self.get_logger().error(f'source {s.name} subscribes to the output topic {out}')
                return TransitionCallbackReturn.FAILURE
        self._pub_out = self.create_lifecycle_publisher(Twist, out, OUTPUT_QOS)
        self._pub_status = self.create_lifecycle_publisher(
            ArbiterStatus, self.get_parameter('status_topic').value, STATUS_QOS)
        self.get_logger().info(
            'configured: output=%s sources=%s hold_timeout=%.2fs' % (
                out, [(s.name, s.topic, s.priority, s.timeout_sec) for s in specs],
                self._arb.hold_timeout_sec))
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._arb.reset()
        for name in self._arb.source_names:
            self._subs.append(self.create_subscription(
                Twist, self._arb.spec(name).topic,
                lambda m, n=name: self._on_source(n, m), INPUT_QOS))
        self._subs.append(self.create_subscription(
            HelixHold, self.get_parameter('hold_topic').value, self._on_hold, INPUT_QOS))
        ret = super().on_activate(state)
        self._active = True
        self._timer = self.create_timer(
            1.0 / float(self.get_parameter('rate_hz').value), self._on_tick)
        return ret

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.publish_shutdown_zero()
        self._teardown_runtime()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._teardown_runtime()
        for pub in (self._pub_out, self._pub_status):
            if pub is not None:
                self.destroy_publisher(pub)
        self._pub_out = self._pub_status = None
        self._arb = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.publish_shutdown_zero()
        return self.on_cleanup(state)

    def _teardown_runtime(self) -> None:
        self._active = False
        if self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)
            self._timer = None
        for sub in self._subs:
            self.destroy_subscription(sub)
        self._subs = []
        if self._arb is not None:
            self._arb.reset()

    # -- callbacks ------------------------------------------------------------

    def _now(self) -> float:
        # Monotonic receipt clock: immune to wall-clock steps on the Jetson.
        return time.monotonic()

    def _on_source(self, name: str, msg: Twist) -> None:
        ok = self._arb.on_source(
            name, (msg.linear.x, msg.linear.y, msg.linear.z),
            (msg.angular.x, msg.angular.y, msg.angular.z), self._now())
        if not ok:
            self.get_logger().warning(
                f'rejected malformed command from {name}', throttle_duration_sec=1.0)

    def _on_hold(self, msg: HelixHold) -> None:
        self._arb.on_hold(msg.hold, msg.fault_id, msg.epoch, msg.seq, self._now())
        # Apply a hold assertion on the very next publish, not up to 1/rate later.
        if msg.hold and self._active:
            self._on_tick()

    def _on_tick(self) -> None:
        if not self._active or self._arb is None:
            return
        self._emit(self._arb.decide(self._now()))

    def _emit(self, d: Decision) -> None:
        self._pub_out.publish(to_twist(d.command))
        self._seq += 1
        st = ArbiterStatus()
        st.selected_source = d.source
        st.reason = d.reason
        st.hold_active = d.helix_forced
        st.hold_fault_id = d.hold_fault_id
        st.out_linear_x, st.out_linear_y, st.out_angular_z = (
            d.command.vx, d.command.vy, d.command.wz)
        st.stamp = time.time()
        st.seq = self._seq
        st.rejected_total = self._arb.counters.rejected if self._arb else 0
        st.sink_subscribers = self._pub_out.get_subscription_count()
        self._pub_status.publish(st)
        if (d.reason, d.source) != (self._last_reason, self._last_source):
            self.get_logger().info(
                f'selected reason={d.reason} source={d.source or "-"} '
                f'cmd=({d.command.vx:+.3f},{d.command.vy:+.3f},{d.command.wz:+.3f}) '
                f'fault={d.hold_fault_id or "-"}')
            self._last_reason, self._last_source = d.reason, d.source

    def publish_shutdown_zero(self) -> None:
        """Publish a burst of zero commands while the publisher is still enabled."""
        if not self._active or self._pub_out is None:
            return
        n = int(self.get_parameter('shutdown_zero_count').value)
        for _ in range(max(1, n)):
            self._emit(Decision(ZERO, REASON_SHUTDOWN))
        self._active = False


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ArbiterNode()
    stop = {'sig': None}

    def _handler(signum, _frame):
        stop['sig'] = signum

    signal.signal(signal.SIGINT, _handler)
    signal.signal(signal.SIGTERM, _handler)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    if node.get_parameter('autostart').value:
        node.trigger_configure()
        node.trigger_activate()
    try:
        while stop['sig'] is None and rclpy.ok():
            executor.spin_once(timeout_sec=0.05)
        node.get_logger().info(
            f'signal {stop["sig"]}: publishing zero burst before exit')
        node.publish_shutdown_zero()
        # Give DDS a moment to flush the reliable zero burst.
        end = time.monotonic() + 0.2
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
