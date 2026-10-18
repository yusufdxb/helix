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


def test_operator_can_disable_automatic_resume():
    env = SafetyEnvelope(
        enabled=True,
        cooldown_seconds=5.0,
        allowed_actions={ACTION_STOP, ACTION_LOG_ONLY},
    )
    result = env.evaluate(action=ACTION_RESUME, fault_type='ANOMALY', now=1.0)
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
        assert node._pub_hold is not None
        assert node._pub_cmd is None          # legacy /helix/cmd_vel off by default
        node.trigger_cleanup()
        assert node._pub_hold is None
        assert node._pub_audit is None
    finally:
        node.destroy_node()


def test_node_publish_tick_emits_zero_twist_only_during_stop():
    """Legacy twist_mux path, opt-in only."""
    node = RecoveryNode()
    node.set_parameters([
        Parameter('enabled', Parameter.Type.BOOL, True),
        Parameter('publish_legacy_cmd_vel', Parameter.Type.BOOL, True),
    ])
    node.trigger_configure()
    node.trigger_activate()
    node._pub_hold.publish = lambda msg: None
    published = []
    node._pub_cmd.publish = lambda msg: published.append(msg)
    try:
        node._on_publish_tick()                       # idle: nothing published
        assert published == []
        node._on_hint(_hint(ACTION_STOP, 'R1'))
        node._on_publish_tick()                       # holding: zero-twist
        # one immediate publish on STOP acceptance, one from the tick
        assert len(published) == 2
        assert all(m.linear.x == 0.0 and m.angular.z == 0.0 for m in published)
    finally:
        node.destroy_node()


# --- hold channel (arbiter contract) -----------------------------------------

def _capture_holds(node):
    holds = []
    node._pub_hold.publish = lambda msg: holds.append(msg)
    return holds


def test_hold_state_published_every_tick_even_when_idle():
    node = _active_node()
    holds = _capture_holds(node)
    try:
        for _ in range(3):
            node._on_publish_tick()
        assert [h.hold for h in holds] == [False, False, False]
        seqs = [h.seq for h in holds]
        assert seqs == sorted(seqs) and len(set(seqs)) == 3
        assert len({h.epoch for h in holds}) == 1
    finally:
        node.destroy_node()


def test_stop_publishes_hold_immediately_with_fault_id():
    node = _active_node()
    holds = _capture_holds(node)
    try:
        node._on_hint(_hint(ACTION_STOP, 'R1'))
        assert holds and holds[-1].hold is True
        assert holds[-1].fault_id == 'test-fault'
        assert holds[-1].asserted_stamp > 0.0
    finally:
        node.destroy_node()


def test_resume_releases_hold_and_never_emits_velocity():
    node = RecoveryNode()
    node.set_parameters([
        Parameter('enabled', Parameter.Type.BOOL, True),
        Parameter('publish_legacy_cmd_vel', Parameter.Type.BOOL, True),
    ])
    node.trigger_configure()
    node.trigger_activate()
    holds = _capture_holds(node)
    twists = []
    node._pub_cmd.publish = lambda m: twists.append(m)
    try:
        node._on_hint(_hint(ACTION_STOP, 'R1'))
        n_twist = len(twists)
        node._on_hint(_hint(ACTION_RESUME, 'R2'))
        assert holds[-1].hold is False and holds[-1].fault_id == ''
        node._on_publish_tick()
        assert len(twists) == n_twist          # RESUME produced no Twist at all
    finally:
        node.destroy_node()


def test_stop_after_resume_inside_cooldown_is_accepted():
    """Regression (safety defect found 2026-09-17): diagnosis releases after
    3 s but cooldown is 5 s, so a fault recurring between those instants was
    SUPPRESSED_COOLDOWN and the robot kept moving with a live fault."""
    node = _active_node(cooldown=5.0)
    holds = _capture_holds(node)
    audits = []
    node._pub_audit.publish = lambda m: audits.append(m)
    try:
        node._on_hint(_hint(ACTION_STOP, 'R1'))
        node._on_hint(_hint(ACTION_RESUME, 'R2'))
        node._on_hint(_hint(ACTION_STOP, 'R1'))      # well inside 5 s
        assert audits[-1].status == 'ACCEPTED'
        assert node._current_action == ACTION_STOP and holds[-1].hold is True
    finally:
        node.destroy_node()


def test_repeated_stop_while_holding_still_cooled_down():
    node = _active_node(cooldown=5.0)
    audits = []
    node._pub_audit.publish = lambda m: audits.append(m)
    try:
        node._on_hint(_hint(ACTION_STOP, 'R1'))
        node._on_hint(_hint(ACTION_STOP, 'R1'))
        assert audits[-1].status == 'SUPPRESSED_COOLDOWN'
        assert node._current_action == ACTION_STOP     # hold unaffected
    finally:
        node.destroy_node()


def test_envelope_stop_not_holding_bypasses_cooldown():
    env = SafetyEnvelope(enabled=True, cooldown_seconds=5.0)
    env.evaluate(ACTION_STOP, 'ANOMALY', 1.0, holding=False)
    assert env.evaluate(ACTION_STOP, 'ANOMALY', 2.0, holding=False).status == 'ACCEPTED'
    assert env.evaluate(ACTION_STOP, 'ANOMALY', 2.5, holding=True).status == 'SUPPRESSED_COOLDOWN'


def test_disabled_recovery_never_asserts_hold():
    node = _active_node(enabled=False)
    holds = _capture_holds(node)
    try:
        node._on_hint(_hint(ACTION_STOP, 'R1'))
        node._on_publish_tick()
        assert all(h.hold is False for h in holds)
    finally:
        node.destroy_node()


def test_deactivate_asserts_hold_before_going_silent():
    node = _active_node()
    holds = _capture_holds(node)
    try:
        node.trigger_deactivate()
        assert holds and holds[-1].hold is True
        assert holds[-1].reason == 'RECOVERY_DEACTIVATING'
    finally:
        node.destroy_node()


def test_accepted_stop_publishes_hold_before_audit():
    """The hold reaches the arbiter before the audit record and its log line."""
    node = _active_node()
    order = []
    node._pub_hold.publish = lambda m: order.append(('hold', m.hold))
    node._pub_audit.publish = lambda m: order.append(('audit', m.status))
    try:
        node._on_hint(_hint(ACTION_STOP, 'R1'))
        assert order[:2] == [('hold', True), ('audit', 'ACCEPTED')]
    finally:
        node.destroy_node()


def test_audit_keeps_the_decision_stamp():
    node = _active_node()
    holds = _capture_holds(node)
    audits = []
    node._pub_audit.publish = lambda m: audits.append(m)
    try:
        node._on_hint(_hint(ACTION_STOP, 'R1'))
        assert audits[-1].timestamp <= holds[-1].stamp
        assert audits[-1].timestamp <= holds[-1].asserted_stamp
    finally:
        node.destroy_node()


def test_suppressed_decision_is_still_audited():
    node = _active_node(enabled=False)
    holds = _capture_holds(node)
    audits = []
    node._pub_audit.publish = lambda m: audits.append(m)
    try:
        node._on_hint(_hint(ACTION_STOP, 'R1'))
        assert [a.status for a in audits] == ['SUPPRESSED_DISABLED']
        assert not holds
    finally:
        node.destroy_node()


def test_audit_survives_a_failure_while_applying():
    node = _active_node()
    audits = []
    node._pub_audit.publish = lambda m: audits.append(m)

    def broken(*_):
        raise RuntimeError('publish failed')

    node._pub_hold.publish = broken
    try:
        try:
            node._on_hint(_hint(ACTION_STOP, 'R1'))
        except RuntimeError:
            pass
        assert [a.status for a in audits] == ['ACCEPTED']
    finally:
        node.destroy_node()
