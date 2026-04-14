"""Tests for the safety envelope — pure, no ROS 2 spin."""
from helix_recovery.recovery_node import SafetyEnvelope, ACTION_STOP, ACTION_RESUME, ACTION_LOG_ONLY


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
