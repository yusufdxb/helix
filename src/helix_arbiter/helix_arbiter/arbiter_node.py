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

Every parameter is read and validated at configure (arbiter_core.parse_config,
the contract shared with the C++ port helix_arbiter_cpp); a change takes
effect on the next cleanup + configure.
"""
from __future__ import annotations

import signal
import time
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from helix_arbiter.arbiter_core import (
    DEFAULT_PARAMETERS,
    REASON_SHUTDOWN,
    ZERO,
    Arbiter,
    ArbiterConfig,
    Command,
    ConfigError,
    Decision,
    check_topic_layout,
    parse_config,
    unknown_parameters,
)
from helix_msgs.msg import ArbiterStatus, HelixHold

# Read-only parameter naming the implementation behind /helix_arbiter. The
# hardware stage tool hashes the arbiter's parameters into its evidence, so
# this binds that evidence to the backend that actually ran.
BACKEND_PARAM = 'arbiter_backend'
BACKEND = 'python'
UINT32_MAX = 2 ** 32 - 1

# BEST_EFFORT subscriptions match both reliable and best-effort publishers.
INPUT_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                       reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE)
# Velocity sources keep only their newest command: a backlog of superseded
# commands would be applied in turn, each stamped fresh on receipt. The hold
# topic keeps INPUT_QOS, because its transitions must not be dropped.
SOURCE_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
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
        for name, default in DEFAULT_PARAMETERS:
            if not self.has_parameter(name):
                self.declare_parameter(name, default)
        if self.has_parameter(BACKEND_PARAM):
            # A parameter file cannot relabel the implementation.
            self.undeclare_parameter(BACKEND_PARAM)
        self.declare_parameter(
            BACKEND_PARAM, BACKEND,
            ParameterDescriptor(read_only=True, description='implementation of this node'),
            ignore_override=True)
        self._cfg: Optional[ArbiterConfig] = None
        self._arb: Optional[Arbiter] = None
        self._pub_out = None
        self._pub_status = None
        self._subs: List = []
        self._timer = None
        self._seq = 0
        self._last_reason: Optional[str] = None
        self._last_source: Optional[str] = None
        self._last_decision: Optional[Decision] = None
        self._active = False

    # -- lifecycle ------------------------------------------------------------

    def _LifecycleNodeMixin__change_state(self, transition_id: int) -> TransitionCallbackReturn:
        # Humble's rclpy triggers a requested transition without checking it,
        # so an invalid one (a second activate from a launch file or operator,
        # say) raises inside the change_state service callback. That ends the
        # executor, and with it the only motion publisher. Refuse it instead,
        # as the C++ backend does: the caller gets success=false and the node
        # keeps running in its current state.
        valid = {t[0] for t in self._state_machine.available_transitions}
        if transition_id not in valid:
            self.get_logger().warning(
                f'refusing lifecycle transition {transition_id}: not valid from '
                f'{self._state_machine.current_state[1]}')
            return TransitionCallbackReturn.FAILURE
        return super()._LifecycleNodeMixin__change_state(transition_id)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        params = {n: p.value for n, p in self.get_parameters_by_prefix('').items()}
        try:
            cfg = parse_config(params)
            arb = Arbiter(list(cfg.sources), cfg.hold_timeout_sec, cfg.limits)
            try:
                resolve = self.resolve_topic_name
                check_topic_layout(
                    resolve(cfg.output_topic), resolve(cfg.status_topic),
                    resolve(cfg.hold_topic), [(s.name, resolve(s.topic)) for s in cfg.sources])
            except RuntimeError as exc:   # rclpy RCLError: invalid topic name
                raise ConfigError(f'invalid topic name: {exc}') from exc
        except (ValueError, TypeError) as exc:   # ConfigError is a ValueError
            self.get_logger().error(f'bad arbiter configuration: {exc}')
            return TransitionCallbackReturn.FAILURE
        unknown = unknown_parameters(params)
        if unknown:
            self.get_logger().warning(f'ignoring unknown parameters {unknown}')
        self._cfg, self._arb = cfg, arb
        self._pub_out = self.create_lifecycle_publisher(Twist, cfg.output_topic, OUTPUT_QOS)
        self._pub_status = self.create_lifecycle_publisher(
            ArbiterStatus, cfg.status_topic, STATUS_QOS)
        self.get_logger().info(
            'configured: output=%s sources=%s hold_timeout=%.2fs' % (
                cfg.output_topic,
                [(s.name, s.topic, s.priority, s.timeout_sec) for s in cfg.sources],
                self._arb.hold_timeout_sec))
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._arb.reset()
        for name in self._arb.source_names:
            self._subs.append(self.create_subscription(
                Twist, self._arb.spec(name).topic,
                lambda m, n=name: self._on_source(n, m), SOURCE_QOS))
        self._subs.append(self.create_subscription(
            HelixHold, self._cfg.hold_topic, self._on_hold, INPUT_QOS))
        ret = super().on_activate(state)
        self._last_decision = None
        self._active = True
        self._timer = self.create_timer(1.0 / self._cfg.rate_hz, self._on_tick)
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
        self._cfg = None
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
        # Publish at once when this input changes what the arbiter outputs
        # (including a rejected command invalidating its source), instead of
        # waiting up to one timer period. The full decision runs as on a tick;
        # the timer still refreshes the output at rate_hz.
        if self._active and self._arb is not None:
            d = self._arb.decide(self._now())
            if d != self._last_decision:
                self._emit(d)

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
        self._last_decision = d
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
        # uint32 field: saturate instead of failing the publish.
        st.rejected_total = min(self._arb.counters.rejected, UINT32_MAX) if self._arb else 0
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
        if not self._active or self._pub_out is None or self._cfg is None:
            return
        for _ in range(self._cfg.shutdown_zero_count):
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
    # Strictly boolean, as parse_config requires: the string 'false' is truthy.
    if node.get_parameter('autostart').value is True:
        # A refused configuration leaves the node unconfigured and silent (the
        # lifecycle services stay up so it can be fixed and configured); it
        # must not crash on an activate that is invalid from 'unconfigured'.
        if node.trigger_configure() == TransitionCallbackReturn.SUCCESS:
            node.trigger_activate()
        else:
            node.get_logger().error('autostart: configure failed; staying unconfigured')
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
