"""
LLMExplainer (v2) — joins FaultEvent + RecoveryHint by fault_id, emits
a human-readable string on /helix/explanations.

NOT in the critical path. If this node crashes, the robot still
recovers. LLM output is advisory only; the RecoveryNode allowlist is
the hard safety gate.

Two paths:
  1. Deterministic template (render_template), always runs and always
     publishes first. Zero-dependency, zero-latency fallback.
  2. Optional LLM advisory path behind `llm_enabled` param. Submits to
     a ThreadPoolExecutor so the ROS 2 executor never blocks, logs
     diagnostics on /helix/llm_diagnostics, and on failure/timeout
     simply skips — the template string has already been published.

Survey doc: notes/local_llm_survey_2026-04-18.md
"""
from __future__ import annotations

import json
from concurrent.futures import Future
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from helix_msgs.msg import FaultEvent, RecoveryHint

from helix_explanation.llm_client import (
    AsyncLLMClient,
    LLMClient,
    LLMClientError,
    LLMRequest,
    LLMResponse,
    LLMRole,
)

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


def _fault_to_dict(fault) -> dict:
    """Serialize a FaultEvent into the LLM-side JSON schema."""
    return {
        'node_name': fault.node_name,
        'fault_type': fault.fault_type,
        'severity': int(fault.severity),
        'detail': fault.detail,
        'timestamp': float(fault.timestamp),
        'context_keys': list(fault.context_keys),
        'context_values': list(fault.context_values),
    }


def _hint_to_dict(hint) -> dict:
    return {
        'fault_id': hint.fault_id,
        'suggested_action': hint.suggested_action,
        'confidence': float(hint.confidence),
        'reasoning': hint.reasoning,
        'rule_matched': hint.rule_matched,
    }


class LLMExplainer(Node):

    def __init__(self):
        super().__init__('helix_llm_explainer')

        # --- parameters ----------------------------------------------------
        self.declare_parameter('llama_server_url', 'http://localhost:8080')
        self.declare_parameter('llama_model', 'qwen2.5-1.5b-instruct')
        self.declare_parameter('llm_timeout_s', 6.0)
        self.declare_parameter('llm_enabled', False)   # ship OFF
        self.declare_parameter('llm_role', 'healer')   # healer | flagger | predictor

        self._llama_url = self.get_parameter('llama_server_url').value
        self._llama_model = self.get_parameter('llama_model').value
        self._llm_timeout = float(self.get_parameter('llm_timeout_s').value)
        self._llm_enabled = bool(self.get_parameter('llm_enabled').value)
        role_str = str(self.get_parameter('llm_role').value).lower()
        try:
            self._llm_role = LLMRole(role_str)
        except ValueError:
            self.get_logger().warn(
                f"unknown llm_role={role_str!r}; defaulting to 'healer'")
            self._llm_role = LLMRole.HEALER

        # --- ROS wiring ----------------------------------------------------
        self._pub = self.create_publisher(String, '/helix/explanations', 10)
        self._diag_pub = self.create_publisher(String, '/helix/llm_diagnostics', 10)
        self._recent_fault: Optional[FaultEvent] = None
        self.create_subscription(FaultEvent, '/helix/faults', self._on_fault, 20)
        self.create_subscription(RecoveryHint, '/helix/recovery_hints', self._on_hint, 10)

        # --- LLM (optional) ------------------------------------------------
        self._async_llm: Optional[AsyncLLMClient] = None
        if self._llm_enabled:
            self._async_llm = AsyncLLMClient(
                LLMClient(
                    base_url=self._llama_url,
                    model=self._llama_model,
                    timeout_s=self._llm_timeout,
                ),
                max_workers=1,
            )
            self.get_logger().info(
                f'LLM advisory enabled: url={self._llama_url} '
                f'model={self._llama_model} role={self._llm_role.value} '
                f'timeout={self._llm_timeout}s'
            )
        else:
            self.get_logger().info(
                'LLM advisory disabled (llm_enabled=false); '
                'deterministic template only'
            )

    # -- callbacks ----------------------------------------------------------

    def _on_fault(self, msg: FaultEvent) -> None:
        self._recent_fault = msg

    def _on_hint(self, msg: RecoveryHint) -> None:
        # Join on fault_id when possible; otherwise use most-recent.
        fault = self._recent_fault
        if msg.fault_id and fault is not None and getattr(fault, 'fault_id', msg.fault_id) != msg.fault_id:
            fault = None

        # 1. Deterministic template — always publish first.
        out = String()
        out.data = render_template(fault, msg)
        self._pub.publish(out)
        self.get_logger().info(out.data)

        # 2. Optional LLM advisory path — runs off-executor, no back-pressure.
        if self._async_llm is None:
            return
        req = LLMRequest(
            role=self._llm_role,
            fault_event=_fault_to_dict(fault) if fault is not None else None,
            context_snapshot={
                'hint': _hint_to_dict(msg),
                # Future: wire in a rosout ring + metric summary here.
            },
        )
        future = self._async_llm.submit(req)
        future.add_done_callback(self._on_llm_done)

    # -- LLM completion handler --------------------------------------------

    def _on_llm_done(self, future: 'Future[LLMResponse]') -> None:
        """Runs on the ThreadPoolExecutor thread, not the ROS executor."""
        diag = String()
        try:
            resp = future.result()
        except LLMClientError as exc:
            diag.data = json.dumps({
                'status': 'llm_error',
                'error_type': type(exc).__name__,
                'error': str(exc),
            })
            self._diag_pub.publish(diag)
            self.get_logger().warn(f'LLM advisory failed: {exc}')
            return
        except Exception as exc:  # pragma: no cover - defense in depth
            diag.data = json.dumps({
                'status': 'llm_unexpected',
                'error_type': type(exc).__name__,
                'error': str(exc),
            })
            self._diag_pub.publish(diag)
            self.get_logger().error(f'LLM advisory unexpected error: {exc}')
            return

        diag.data = json.dumps({
            'status': 'ok',
            'role': resp.role.value,
            'model': self._llama_model,
            'latency_ms': round(resp.latency_ms, 1),
            'data': resp.data,
        })
        self._diag_pub.publish(diag)
        self.get_logger().info(
            f'LLM {resp.role.value} advisory '
            f'({resp.latency_ms:.0f}ms): {resp.data}'
        )

    # -- shutdown -----------------------------------------------------------

    def destroy_node(self):  # type: ignore[override]
        if self._async_llm is not None:
            self._async_llm.shutdown(wait=False)
        return super().destroy_node()


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
