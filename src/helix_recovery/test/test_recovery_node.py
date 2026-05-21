"""Tests for the recovery tier: the pure SafetyEnvelope plus node-level
lifecycle and hint-handling behaviour."""
from helix_recovery.recovery_node import (
    ACTION_LOG_ONLY,
    ACTION_RESUME,
    ACTION_STOP,
    RecoveryNode,
    SafetyEnvelope,
)
from rclpy.parameter import Parameter

from helix_msgs.msg import RecoveryHint

# --- SafetyEnvelope (pure, no ROS 2 spin) -----------------------------------

def test_disabled_rejects_everything():
    env = SafetyEnvelope(enabled=False, cooldown_seconds=5.0)
    result = env.evaluate(action=ACTION_STOP, fault_type='ANOMALY', now=1.0)
    assert result.status == 'SUPPRESSED_DISABLED'
    assert result.publish is False


def test_enabled_accepts_allowlisted_action():
    env = SafetyEnvelope(enabled=True, cooldown_seconds=5.0)
    result = env.evaluate(action=ACTION_STOP, fault_type='ANOMALY', now=1.0)
    assert result.status == 'ACCEPTED'
    assert result.publish is True


def test_cooldown_suppresses_second_action_same_type():
    env = SafetyEnvelope(enabled=True, cooldown_seconds=5.0)
    env.evaluate(action=ACTION_STOP, fault_type='ANOMALY', now=1.0)
    result = env.evaluate(action=ACTION_STOP, fault_type='ANOMALY', now=2.0)
    assert result.status == 'SUPPRESSED_COOLDOWN'
    assert result.publish is False


def test_cooldown_allows_different_fault_type():
    env = SafetyEnvelope(enabled=True, cooldown_seconds=5.0)
    env.evaluate(action=ACTION_STOP, fault_type='ANOMALY', now=1.0)
    result = env.evaluate(action=ACTION_STOP, fault_type='LOG_PATTERN', now=2.0)
    assert result.status == 'ACCEPTED'


def test_cooldown_releases_after_window():
    env = SafetyEnvelope(enabled=True, cooldown_seconds=5.0)
    env.evaluate(action=ACTION_STOP, fault_type='ANOMALY', now=1.0)
    result = env.evaluate(action=ACTION_STOP, fault_type='ANOMALY', now=7.0)
    assert result.status == 'ACCEPTED'


def test_allowlist_rejects_unknown_action():
    env = SafetyEnvelope(enabled=True, cooldown_seconds=5.0)
    result = env.evaluate(action='SELF_DESTRUCT', fault_type='ANOMALY', now=1.0)
    assert result.status == 'SUPPRESSED_ALLOWLIST'
    assert result.publish is False


def test_log_only_is_never_published_but_is_accepted():
    env = SafetyEnvelope(enabled=True, cooldown_seconds=5.0)
    result = env.evaluate(action=ACTION_LOG_ONLY, fault_type='CRASH', now=1.0)
    assert result.status == 'ACCEPTED'
    assert result.publish is False    # LOG_ONLY never actuates


def test_resume_is_exempt_from_cooldown():
    """Regression: a RESUME must clear a STOP even inside the STOP's cooldown
    window. Previously RESUME shared the ANOMALY cooldown bucket, so a safety
    stop could suppress its own release and hold the robot too long."""
    env = SafetyEnvelope(enabled=True, cooldown_seconds=5.0)
    env.evaluate(action=ACTION_STOP, fault_type='ANOMALY', now=1.0)
    result = env.evaluate(action=ACTION_RESUME, fault_type='ANOMALY', now=2.0)
    assert result.status == 'ACCEPTED'
    assert result.publish is True


def test_resume_does_not_establish_a_cooldown():
    """A RESUME must not write a cooldown bucket that would block a later STOP."""
    env = SafetyEnvelope(enabled=True, cooldown_seconds=5.0)
    env.evaluate(action=ACTION_RESUME, fault_type='ANOMALY', now=1.0)
    result = env.evaluate(action=ACTION_STOP, fault_type='ANOMALY', now=2.0)
    assert result.status == 'ACCEPTED'


# --- RecoveryNode (lifecycle + hint handling) -------------------------------

def _hint(action: str, rule: str) -> RecoveryHint:
    h = RecoveryHint()
    h.fault_id = 'test-fault'
    h.suggested_action = action
    h.confidence = 0.9
    h.reasoning = 'test'
    h.rule_matched = rule
    return h


def _active_node(enabled: bool = True, cooldown: float = 5.0) -> RecoveryNode:
    node = RecoveryNode()
    node.set_parameters([
        Parameter('enabled', Parameter.Type.BOOL, enabled),
        Parameter('cooldown_seconds', Parameter.Type.DOUBLE, cooldown),
    ])
    node.trigger_configure()
    node.trigger_activate()
    return node


def test_node_stop_sets_hold_then_resume_clears_it_within_cooldown():
    """End-to-end through the node: a RESUME issued microseconds after a STOP,
    well inside the cooldown, must still clear the hold."""
    node = _active_node()
    try:
        node._on_hint(_hint(ACTION_STOP, 'R1'))
        assert node._current_action == ACTION_STOP
        node._on_hint(_hint(ACTION_RESUME, 'R2'))
        assert node._current_action is None
    finally:
        node.destroy_node()


def test_node_deactivate_drops_in_progress_hold():
    """A deactivate while holding STOP must clear _current_action so a later
    re-activate cannot resurrect a stale stop."""
    node = _active_node()
    try:
        node._on_hint(_hint(ACTION_STOP, 'R1'))
        assert node._current_action == ACTION_STOP
        node.trigger_deactivate()
        assert node._current_action is None
    finally:
        node.destroy_node()


def test_node_cleanup_releases_publishers():
    """on_cleanup must run without error and drop the publishers."""
    node = RecoveryNode()
    try:
        node.trigger_configure()
        assert node._pub_cmd is not None
        node.trigger_cleanup()
        assert node._pub_cmd is None
        assert node._pub_audit is None
    finally:
        node.destroy_node()


def test_node_publish_tick_emits_zero_twist_only_during_stop():
    node = _active_node()
    published = []
    node._pub_cmd.publish = lambda msg: published.append(msg)
    try:
        node._on_publish_tick()                       # idle: nothing published
        assert published == []
        node._on_hint(_hint(ACTION_STOP, 'R1'))
        node._on_publish_tick()                       # holding: zero-twist
        assert len(published) == 1
        assert published[0].linear.x == 0.0
        assert published[0].angular.z == 0.0
    finally:
        node.destroy_node()
