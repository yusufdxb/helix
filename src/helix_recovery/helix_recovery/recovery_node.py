"""
RecoveryNode lifecycle node.

HELIX never commands motion. This node turns accepted recovery hints into a
hold STATE on /helix/hold (helix_msgs/HelixHold), published at PUBLISH_HZ for
as long as the node is active. The motion arbiter (helix_arbiter) owns the
only velocity output and forces it to zero while the hold is asserted, while
the hold state is stale, or before any hold state has arrived. RESUME only
clears the hold; it never produces a velocity.

All recovery safety checks live here: enable flag, per-fault cooldown, action
allowlist. The legacy zero-twist on /helix/cmd_vel (fed to twist_mux) is off
by default because it is a second velocity source competing in a mux, and
twist_mux neither publishes zero on idle nor on lock (see
docs/MOTION_ARBITRATION.md).
"""
import time
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
import rclpy.executors
from geometry_msgs.msg import Twist
from helix_core.heartbeat import Heartbeat
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn

from helix_msgs.msg import HelixHold, RecoveryAction, RecoveryHint

ACTION_STOP = 'STOP_AND_HOLD'
ACTION_RESUME = 'RESUME'
ACTION_LOG_ONLY = 'LOG_ONLY'

ALLOWED_ACTIONS = {ACTION_STOP, ACTION_RESUME, ACTION_LOG_ONLY}

# LOG_ONLY is allowlisted but never actuates.
PUBLISHING_ACTIONS = {ACTION_STOP, ACTION_RESUME}

PUBLISH_HZ: float = 20.0


@dataclass
class EnvelopeResult:
    status: str          # 'ACCEPTED' | 'SUPPRESSED_*'
    publish: bool        # whether to actuate
    reason: str


class SafetyEnvelope:
    """Pure and unit-testable without ROS 2."""

    def __init__(self, enabled: bool, cooldown_seconds: float,
                 allowed_actions=None):
        self.enabled = enabled
        self.cooldown_seconds = cooldown_seconds
        self.allowed_actions = set(
            ALLOWED_ACTIONS if allowed_actions is None else allowed_actions
        )
        self._last_action_time: Dict[str, float] = {}

    def evaluate(self, action: str, fault_type: str, now: float,
                 holding: bool = True) -> EnvelopeResult:
        """Decide whether ``action`` may take effect.

        ``holding`` is whether a STOP_AND_HOLD is already in force. Cooldown
        only damps a repeated STOP while the robot is already held; a STOP
        that arrives while NOT holding is always accepted. Otherwise a fault
        that recurs shortly after a RESUME (the diagnosis clear window, 3 s,
        is shorter than the default 5 s cooldown) would be suppressed and
        the robot would keep moving with a live fault.
        """
        if not self.enabled:
            return EnvelopeResult('SUPPRESSED_DISABLED', False, 'recovery.enabled is false')
        if action not in self.allowed_actions:
            return EnvelopeResult('SUPPRESSED_ALLOWLIST', False, f'{action} not in allowlist')
        # RESUME ends a STOP_AND_HOLD. It must never be rate-limited by the
        # cooldown of the stop it is clearing, or a safety stop could suppress
        # its own release and hold the robot longer than the fault lasts.
        # Cooldown exists only to damp STOP_AND_HOLD flapping.
        if action != ACTION_RESUME:
            last = self._last_action_time.get(fault_type)
            in_cooldown = last is not None and (now - last) < self.cooldown_seconds
            if in_cooldown and (holding or action != ACTION_STOP):
                return EnvelopeResult('SUPPRESSED_COOLDOWN', False,
                                      f'cooldown active for {fault_type} ({now - last:.2f}s)')
            self._last_action_time[fault_type] = now
        publish = action in PUBLISHING_ACTIONS
        return EnvelopeResult('ACCEPTED', publish, f'action {action} accepted')


class RecoveryNode(LifecycleNode):

    def __init__(self):
        super().__init__('helix_recovery_node')
        self._heartbeat = Heartbeat(self)
        self.declare_parameter('enabled', False)
        self.declare_parameter('cooldown_seconds', 5.0)
        self.declare_parameter('allowed_actions', sorted(ALLOWED_ACTIONS))
        # Legacy twist_mux path: zero Twist on /helix/cmd_vel while holding.
        # Off by default; the arbiter consumes /helix/hold instead.
        self.declare_parameter('publish_legacy_cmd_vel', False)

        self._envelope: Optional[SafetyEnvelope] = None
        self._sub = None
        self._pub_cmd = None
        self._pub_audit = None
        self._pub_hold = None
        self._publish_timer = None
        self._legacy_cmd_vel = False

        # Recovery state
        self._current_action: Optional[str] = None     # ACTION_STOP when holding; None when idle
        self._last_fault_type: Optional[str] = None
        self._hold_fault_id = ''
        self._hold_reason = 'no hold'
        self._hold_asserted_stamp = 0.0
        # (epoch, seq) lets the arbiter drop reordered or duplicated states.
        self._epoch = time.time_ns()
        self._seq = 0

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        enabled = self.get_parameter('enabled').value
        cooldown = self.get_parameter('cooldown_seconds').value
        allowed_actions = set(self.get_parameter('allowed_actions').value)
        unsupported = allowed_actions - ALLOWED_ACTIONS
        if unsupported:
            self.get_logger().error(
                f'allowed_actions contains unsupported values: {sorted(unsupported)}'
            )
            return TransitionCallbackReturn.FAILURE
        self._envelope = SafetyEnvelope(
            enabled=enabled,
            cooldown_seconds=cooldown,
            allowed_actions=allowed_actions,
        )
        self._legacy_cmd_vel = bool(self.get_parameter('publish_legacy_cmd_vel').value)
        if self._legacy_cmd_vel:
            self._pub_cmd = self.create_lifecycle_publisher(Twist, '/helix/cmd_vel', 10)
        self._pub_hold = self.create_lifecycle_publisher(HelixHold, '/helix/hold', 10)
        self._pub_audit = self.create_lifecycle_publisher(RecoveryAction, '/helix/recovery_actions', 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._heartbeat.start()
        self._sub = self.create_subscription(RecoveryHint, '/helix/recovery_hints', self._on_hint, 10)
        # Publishes the hold state every tick (holding or not) so the arbiter
        # can distinguish "not holding" from "recovery is gone".
        self._publish_timer = self.create_timer(1.0 / PUBLISH_HZ, self._on_publish_tick)
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # Tell the arbiter to hold NOW instead of waiting for the hold state
        # to go stale. Must precede super().on_deactivate, which disables the
        # lifecycle publishers.
        self._publish_hold(True, 'RECOVERY_DEACTIVATING')
        self._heartbeat.stop()
        if self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None
        if self._publish_timer is not None:
            self._publish_timer.cancel()
            self._publish_timer = None
        # Drop any in-progress hold so a later re-activate cannot resurrect a
        # stale STOP with no live fault behind it.
        self._clear_hold()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Destroy publishers created in on_configure; return to unconfigured."""
        if self._pub_cmd is not None:
            self.destroy_publisher(self._pub_cmd)
            self._pub_cmd = None
        if self._pub_audit is not None:
            self.destroy_publisher(self._pub_audit)
            self._pub_audit = None
        if self._pub_hold is not None:
            self.destroy_publisher(self._pub_hold)
            self._pub_hold = None
        self._envelope = None
        self._clear_hold()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Release resources on shutdown from any lifecycle state."""
        return self.on_cleanup(state)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_hint(self, msg: RecoveryHint) -> None:
        # Cooldown is keyed by fault_type. RecoveryHint carries no fault_type,
        # so rule_matched is the proxy (R1 -> ANOMALY, R3 -> LOG_PATTERN,
        # R4 -> CRASH). R2 emits RESUME, which the envelope exempts from
        # cooldown, so its key is never consulted.
        fault_type = _rule_to_fault_type(msg.rule_matched)
        holding = self._current_action == ACTION_STOP
        result = self._envelope.evaluate(
            msg.suggested_action, fault_type, self._now(), holding=holding)
        self._audit(msg, result)
        if not result.publish:
            return
        if msg.suggested_action == ACTION_STOP:
            if not holding:
                self._current_action = ACTION_STOP
                self._hold_fault_id = msg.fault_id
                self._hold_reason = f'{ACTION_STOP} via {msg.rule_matched}'
                self._hold_asserted_stamp = self._now()
            # Publish immediately rather than waiting up to one tick.
            self._on_publish_tick()
        elif msg.suggested_action == ACTION_RESUME:
            # Releases the hold only. No velocity is produced here; the
            # arbiter hands authority back to whichever upstream source
            # publishes a NEW command after the release.
            self._clear_hold()
            self._hold_reason = f'{ACTION_RESUME} via {msg.rule_matched}'
            self._on_publish_tick()

    def _clear_hold(self) -> None:
        self._current_action = None
        self._hold_fault_id = ''
        self._hold_reason = 'no hold'
        self._hold_asserted_stamp = 0.0

    def _publish_hold(self, hold: bool, reason: str) -> None:
        if self._pub_hold is None:
            return
        self._seq += 1
        msg = HelixHold()
        msg.hold = hold
        msg.fault_id = self._hold_fault_id
        msg.reason = reason
        msg.epoch = self._epoch
        msg.seq = self._seq
        msg.stamp = self._now()
        msg.asserted_stamp = self._hold_asserted_stamp if hold else 0.0
        self._pub_hold.publish(msg)

    def _on_publish_tick(self) -> None:
        holding = self._current_action == ACTION_STOP
        self._publish_hold(holding, self._hold_reason)
        if holding and self._legacy_cmd_vel and self._pub_cmd is not None:
            self._pub_cmd.publish(Twist())   # zero velocity, legacy mux path

    def _audit(self, hint: RecoveryHint, result: EnvelopeResult) -> None:
        msg = RecoveryAction()
        msg.fault_id = hint.fault_id
        msg.action = hint.suggested_action
        msg.status = result.status
        msg.timestamp = self._now()
        msg.reason = result.reason
        self._pub_audit.publish(msg)
        self.get_logger().info(f'audit: {msg.action} {msg.status} {msg.reason}')


def _rule_to_fault_type(rule: str) -> str:
    return {
        'R1': 'ANOMALY',
        'R2': 'ANOMALY',     # R2 emits RESUME; the envelope exempts RESUME from cooldown
        'R3': 'LOG_PATTERN',
        'R4': 'CRASH',
    }.get(rule, 'UNKNOWN')


def main(args=None):
    rclpy.init(args=args)
    node = RecoveryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # On exit the hold state simply stops; the arbiter treats a stale
        # HELIX state as HOLD, so dying is fail-safe.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
