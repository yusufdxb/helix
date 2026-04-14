"""
LLMExplainer (v1 stub) — joins FaultEvent + RecoveryHint by fault_id,
emits a templated string on /helix/explanations.

NOT in the critical path. If this node crashes, the robot still recovers.

v2 upgrade: replace render_template() with an Ollama inference call.
"""
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from helix_msgs.msg import FaultEvent, RecoveryHint

SEVERITY_LABEL = {1: 'WARN', 2: 'ERROR', 3: 'CRITICAL'}


def render_template(fault_event, hint) -> str:
    """Pure formatter. Unit-testable without ROS 2."""
    if fault_event is None:
        return (f'Action: {hint.suggested_action} '
                f'(rule {hint.rule_matched}, confidence {hint.confidence:.2f}). '
                f'Reason: {hint.reasoning}.')
    sev = SEVERITY_LABEL.get(fault_event.severity, str(fault_event.severity))
    metric = _extract(fault_event.context_keys, fault_event.context_values, 'metric')
    return (
        f'Fault detected: {fault_event.fault_type}'
        + (f' on {metric}' if metric else '')
        + f' at t={fault_event.timestamp:.2f}. '
        f'Severity: {sev}. '
        f'Recovery action: {hint.suggested_action} '
        f'(rule {hint.rule_matched}, confidence {hint.confidence:.2f}). '
        f'Reason: {hint.reasoning}.'
    )


def _extract(keys, values, target) -> Optional[str]:
    try:
        idx = list(keys).index(target)
        return values[idx]
    except (ValueError, AttributeError, IndexError):
        return None


class LLMExplainer(Node):

    def __init__(self):
        super().__init__('helix_llm_explainer')
        self._pub = self.create_publisher(String, '/helix/explanations', 10)
        self._recent_fault = None         # last FaultEvent seen
        self.create_subscription(FaultEvent, '/helix/faults', self._on_fault, 20)
        self.create_subscription(RecoveryHint, '/helix/recovery_hints', self._on_hint, 10)

    def _on_fault(self, msg: FaultEvent) -> None:
        self._recent_fault = msg

    def _on_hint(self, msg: RecoveryHint) -> None:
        # Join on fault_id when possible; otherwise use most-recent.
        fault = self._recent_fault
        if msg.fault_id and fault is not None and fault.fault_id != msg.fault_id:
            fault = None
        out = String()
        out.data = render_template(fault, msg)
        self._pub.publish(out)
        self.get_logger().info(out.data)


def main(args=None):
    rclpy.init(args=args)
    node = LLMExplainer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
