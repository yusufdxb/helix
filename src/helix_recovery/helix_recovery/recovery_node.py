"""
RecoveryNode — lifecycle node.

The only node in HELIX that publishes actuation commands.
All safety checks live here: enable flag, per-fault cooldown, action allowlist.
twist_mux fail-safe handles the case where this node crashes (input times out).
"""
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn

from helix_msgs.msg import RecoveryAction, RecoveryHint


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
    """Pure — unit-testable without ROS 2."""

    def __init__(self, enabled: bool, cooldown_seconds: float):
        self.enabled = enabled
        self.cooldown_seconds = cooldown_seconds
        self._last_action_time: Dict[str, float] = {}

    def evaluate(self, action: str, fault_type: str, now: float) -> EnvelopeResult:
        if not self.enabled:
            return EnvelopeResult('SUPPRESSED_DISABLED', False, 'recovery.enabled is false')
        if action not in ALLOWED_ACTIONS:
            return EnvelopeResult('SUPPRESSED_ALLOWLIST', False, f'{action} not in allowlist')
        last = self._last_action_time.get(fault_type)
        if last is not None and (now - last) < self.cooldown_seconds:
            return EnvelopeResult('SUPPRESSED_COOLDOWN', False,
                                  f'cooldown active for {fault_type} ({now - last:.2f}s)')
        self._last_action_time[fault_type] = now
        publish = action in PUBLISHING_ACTIONS
        return EnvelopeResult('ACCEPTED', publish, f'action {action} accepted')


class RecoveryNode(LifecycleNode):

    def __init__(self):
        super().__init__('helix_recovery_node')
        self.declare_parameter('enabled', False)
        self.declare_parameter('cooldown_seconds', 5.0)

        self._envelope: Optional[SafetyEnvelope] = None
        self._sub = None
        self._pub_cmd = None
        self._pub_audit = None
        self._publish_timer = None

        # Recovery state
        self._current_action: Optional[str] = None     # ACTION_STOP when holding; None when idle
        self._last_fault_type: Optional[str] = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        enabled = self.get_parameter('enabled').value
        cooldown = self.get_parameter('cooldown_seconds').value
        self._envelope = SafetyEnvelope(enabled=enabled, cooldown_seconds=cooldown)
        self._pub_cmd = self.create_lifecycle_publisher(Twist, '/helix/cmd_vel', 10)
        self._pub_audit = self.create_lifecycle_publisher(RecoveryAction, '/helix/recovery_actions', 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._sub = self.create_subscription(RecoveryHint, '/helix/recovery_hints', self._on_hint, 10)
        # Publish-timer is idle until _current_action set to STOP.
        self._publish_timer = self.create_timer(1.0 / PUBLISH_HZ, self._on_publish_tick)
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        if self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None
        if self._publish_timer is not None:
            self._publish_timer.cancel()
            self._publish_timer = None
        return super().on_deactivate(state)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_hint(self, msg: RecoveryHint) -> None:
        # Derive fault_type for cooldown keying. RecoveryHint doesn't carry it directly;
        # we use the rule_matched as a rough proxy since rule IDs map 1:1 to fault_type
        # in v1 (R1→ANOMALY, R2→no-fault, R3→LOG_PATTERN, R4→CRASH).
        fault_type = _rule_to_fault_type(msg.rule_matched)
        result = self._envelope.evaluate(msg.suggested_action, fault_type, self._now())
        self._audit(msg, result)
        if not result.publish:
            return
        if msg.suggested_action == ACTION_STOP:
            self._current_action = ACTION_STOP
        elif msg.suggested_action == ACTION_RESUME:
            self._current_action = None

    def _on_publish_tick(self) -> None:
        if self._current_action == ACTION_STOP:
            t = Twist()   # zero velocity
            self._pub_cmd.publish(t)

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
        'R2': 'ANOMALY',     # RESUME also keyed under ANOMALY (same cooldown bucket)
        'R3': 'LOG_PATTERN',
        'R4': 'CRASH',
    }.get(rule, 'UNKNOWN')


def main(args=None):
    rclpy.init(args=args)
    node = RecoveryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
