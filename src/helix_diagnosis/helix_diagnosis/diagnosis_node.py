"""
DiagnosisNode — lifecycle node.

Subscribes to /helix/faults (FaultEvent).
Queries /helix/get_context service on each fault.
Runs rules from helix_diagnosis.rules.evaluate.
Publishes /helix/recovery_hints (RecoveryHint).
Also ticks a timer to fire state-driven rules (R2 RESUME).
"""
from typing import Optional

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn

from helix_diagnosis.rules import STATE_IDLE, STATE_STOP_AND_HOLD, HintShape, evaluate
from helix_msgs.msg import FaultEvent, RecoveryHint
from helix_msgs.srv import GetContext

TICK_HZ: float = 10.0


class DiagnosisStateMachine:
    """Pure state machine — unit-testable without ROS 2."""

    def __init__(self):
        self.current_state = STATE_IDLE
        self._last_anomaly_time: Optional[float] = None

    def process_fault(self, fault_event, now_seconds: float) -> Optional[HintShape]:
        if fault_event.fault_type == 'ANOMALY':
            self._last_anomaly_time = now_seconds
        # Build a tiny context namespace for rules.evaluate()
        ctx = _Ctx(anomaly_clear_seconds=0.0)
        hint = evaluate(fault_event, ctx, self.current_state)
        if hint is not None and hint.suggested_action == 'STOP_AND_HOLD':
            self.current_state = STATE_STOP_AND_HOLD
        return hint

    def tick(self, now_seconds: float) -> Optional[HintShape]:
        if self.current_state != STATE_STOP_AND_HOLD:
            return None
        if self._last_anomaly_time is None:
            return None
        clear = now_seconds - self._last_anomaly_time
        ctx = _Ctx(anomaly_clear_seconds=clear)
        hint = evaluate(None, ctx, self.current_state)
        if hint is not None and hint.suggested_action == 'RESUME':
            self.current_state = STATE_IDLE
        return hint


class _Ctx:
    def __init__(self, anomaly_clear_seconds: float = 0.0):
        self.anomaly_clear_seconds = anomaly_clear_seconds
        self.rosout_ring = []
        self.metrics_json = '{}'
        self.node_health_json = '{}'
        self.snapshot_time = 0.0


class DiagnosisNode(LifecycleNode):

    def __init__(self):
        super().__init__('helix_diagnosis_node')
        self._sm = DiagnosisStateMachine()
        self._sub = None
        self._pub = None
        self._tick_timer = None
        self._ctx_client = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._pub = self.create_lifecycle_publisher(RecoveryHint, '/helix/recovery_hints', 10)
        self._ctx_client = self.create_client(GetContext, '/helix/get_context')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._sub = self.create_subscription(FaultEvent, '/helix/faults', self._on_fault, 10)
        self._tick_timer = self.create_timer(1.0 / TICK_HZ, self._on_tick)
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        if self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None
        if self._tick_timer is not None:
            self._tick_timer.cancel()
            self._tick_timer = None
        return super().on_deactivate(state)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_fault(self, msg: FaultEvent) -> None:
        hint_shape = self._sm.process_fault(msg, self._now())
        if hint_shape is not None:
            self._publish(hint_shape)

    def _on_tick(self) -> None:
        hint_shape = self._sm.tick(self._now())
        if hint_shape is not None:
            self._publish(hint_shape)

    def _publish(self, hint: HintShape) -> None:
        msg = RecoveryHint()
        msg.fault_id = hint.fault_id
        msg.suggested_action = hint.suggested_action
        msg.confidence = hint.confidence
        msg.reasoning = hint.reasoning
        msg.rule_matched = hint.rule_matched
        self._pub.publish(msg)
        self.get_logger().info(
            f'[{hint.rule_matched}] {hint.suggested_action}: {hint.reasoning}')


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosisNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
