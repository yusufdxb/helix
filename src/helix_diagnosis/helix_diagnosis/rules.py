"""
Deterministic recovery rules for HELIX diagnosis tier.

Pure function `evaluate()` takes a FaultEvent-shaped object + a context snapshot
+ current recovery state, returns a RecoveryHint-shaped object or None.

No ROS 2 runtime dependency in this file — unit testable standalone.
First match wins. Rule order is the list order in `_RULES`.
"""
from dataclasses import dataclass
from typing import Any, List, Optional

STATE_IDLE = 'IDLE'
STATE_STOP_AND_HOLD = 'STOP_AND_HOLD'

SEVERITY_WARN = 1
SEVERITY_ERROR = 2
SEVERITY_CRITICAL = 3

ANOMALY_CLEAR_WINDOW_SECONDS = 3.0


@dataclass
class HintShape:
    """Shape-compatible with helix_msgs/RecoveryHint. Used so rules.py stays ROS-free."""
    fault_id: str
    suggested_action: str
    confidence: float
    reasoning: str
    rule_matched: str


def _metric_name(fault_event: Any) -> Optional[str]:
    """Pull metric name from fault_event.context_keys/context_values.

    Emitter (helix_core.anomaly_detector) writes the key as ``metric_name``.
    Older callers used ``metric``; we accept either for forward/backward compat.
    """
    try:
        keys = list(fault_event.context_keys)
    except (TypeError, AttributeError):
        return None
    for k in ('metric_name', 'metric'):
        try:
            idx = keys.index(k)
        except ValueError:
            continue
        return fault_event.context_values[idx]
    return None


def _fault_source_id(fault) -> str:
    """FaultEvent has no fault_id field — use node_name as the correlation key."""
    return getattr(fault, 'node_name', '') or ''


def _rule_r1(fault, ctx, state) -> Optional[HintShape]:
    if fault is None:
        return None
    if fault.fault_type != 'ANOMALY':
        return None
    metric = _metric_name(fault)
    # Match any utlidar-family rate metric (cloud, imu, robot_odom, robot_pose).
    # Emitter currently writes 'rate_hz/utlidar_<topic>'; historical tests used
    # the symbolic name 'utlidar_rate'.
    if metric is None:
        return None
    if not (metric.startswith('rate_hz/utlidar') or metric == 'utlidar_rate'):
        return None
    if fault.severity < SEVERITY_ERROR:
        return None
    return HintShape(
        fault_id=_fault_source_id(fault),
        suggested_action='STOP_AND_HOLD',
        confidence=0.9,
        reasoning='LiDAR rate anomaly at ERROR severity — stop and hold',
        rule_matched='R1',
    )


def _rule_r2(fault, ctx, state) -> Optional[HintShape]:
    if state != STATE_STOP_AND_HOLD:
        return None
    clear_seconds = getattr(ctx, 'anomaly_clear_seconds', 0.0)
    if clear_seconds < ANOMALY_CLEAR_WINDOW_SECONDS:
        return None
    return HintShape(
        fault_id='',  # no fault correlation — RESUME is state-driven
        suggested_action='RESUME',
        confidence=0.9,
        reasoning=f'Anomaly cleared for {clear_seconds:.1f}s — resuming',
        rule_matched='R2',
    )


def _rule_r3(fault, ctx, state) -> Optional[HintShape]:
    if fault is None:
        return None
    if fault.fault_type != 'LOG_PATTERN':
        return None
    if fault.severity != SEVERITY_CRITICAL:
        return None
    return HintShape(
        fault_id=_fault_source_id(fault),
        suggested_action='STOP_AND_HOLD',
        confidence=0.7,
        reasoning='Critical log pattern detected — defensive stop',
        rule_matched='R3',
    )


def _rule_r4(fault, ctx, state) -> Optional[HintShape]:
    if fault is None:
        return None
    if fault.fault_type != 'CRASH':
        return None
    return HintShape(
        fault_id=_fault_source_id(fault),
        suggested_action='LOG_ONLY',
        confidence=0.5,
        reasoning='Node crash detected — logged, no actuation (scope limit)',
        rule_matched='R4',
    )


_RULES: List = [_rule_r1, _rule_r2, _rule_r3, _rule_r4]


def evaluate(fault_event: Any, context: Any, current_state: str) -> Optional[HintShape]:
    """Run rules in order; return first non-None hint, else None."""
    for rule in _RULES:
        hint = rule(fault_event, context, current_state)
        if hint is not None:
            return hint
    return None
