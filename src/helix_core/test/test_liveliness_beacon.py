"""Liveliness beacon tests.

Mirrors test_heartbeat.py's fake-node style so the QoS-based mechanism gets
the same unit-level scrutiny as the existing heartbeat emitter, without
needing a live rclpy/DDS session for the plumbing (start/stop/idempotency,
QoS shape, per-node topic naming). The DDS-level detection itself (does a
lease actually expire and fire a callback) is proven separately in
test_liveliness_integration.py against real rclpy + Cyclone DDS.
"""
import pytest
from helix_core.liveliness_beacon import (
    DEFAULT_ASSERT_PERIOD_SEC,
    DEFAULT_LEASE_DURATION_SEC,
    LIVELINESS_TOPIC_PREFIX,
    LivelinessBeacon,
    build_liveliness_qos,
    liveliness_topic_for,
)
from rclpy.qos import LivelinessPolicy


class FakeTimer:
    def __init__(self, period, callback):
        self.period = period
        self.callback = callback


class FakePublisher:
    def __init__(self):
        self.assert_calls = 0
        self.published = []

    def assert_liveliness(self):
        self.assert_calls += 1

    def publish(self, msg):
        self.published.append(msg)


class FakeNode:
    """Records what a lifecycle node would have created, without ROS running."""

    def __init__(self, name='helix_fake_node'):
        self._name = name
        self.publisher = None
        self.topic = None
        self.qos = None
        self.timers = []
        self.destroyed = []

    def get_name(self):
        return self._name

    def create_publisher(self, msg_type, topic, qos_profile):
        self.topic = topic
        self.qos = qos_profile
        self.publisher = FakePublisher()
        return self.publisher

    def create_timer(self, period, callback):
        timer = FakeTimer(period, callback)
        self.timers.append(timer)
        return timer

    def destroy_timer(self, timer):
        self.destroyed.append(timer)


@pytest.fixture
def node():
    return FakeNode()


def test_topic_is_per_node_not_shared(node):
    LivelinessBeacon(node)
    assert node.topic == liveliness_topic_for('helix_fake_node')
    assert node.topic == f'{LIVELINESS_TOPIC_PREFIX}helix_fake_node'


def test_qos_uses_manual_by_topic_liveliness(node):
    LivelinessBeacon(node)
    assert node.qos.liveliness == LivelinessPolicy.MANUAL_BY_TOPIC


def test_qos_lease_duration_matches_constructor_argument(node):
    LivelinessBeacon(node, lease_duration_sec=0.75)
    assert node.qos.liveliness_lease_duration.nanoseconds == int(0.75 * 1e9)


def test_build_liveliness_qos_is_reused_by_beacon_and_would_match_a_watchdog():
    """Both sides of the mechanism must build QoS the same way, or LIVELINESS
    compatibility rules (subscriber's requested lease must be offered by the
    publisher) can silently reject the pairing."""
    qos_a = build_liveliness_qos(0.5)
    qos_b = build_liveliness_qos(0.5)
    assert qos_a.liveliness == qos_b.liveliness
    assert qos_a.liveliness_lease_duration.nanoseconds == qos_b.liveliness_lease_duration.nanoseconds


def test_no_timer_runs_until_started(node):
    LivelinessBeacon(node)
    assert node.timers == []


def test_start_asserts_liveliness_without_publishing_payload(node):
    beacon = LivelinessBeacon(node)
    beacon.start()
    node.timers[0].callback()
    assert node.publisher.assert_calls == 1
    assert node.publisher.published == []


def test_start_is_idempotent(node):
    beacon = LivelinessBeacon(node)
    beacon.start()
    beacon.start()
    assert len(node.timers) == 1


def test_stop_destroys_the_timer_and_allows_restart(node):
    beacon = LivelinessBeacon(node)
    beacon.start()
    first = node.timers[0]
    beacon.stop()
    assert node.destroyed == [first]
    beacon.start()
    assert len(node.timers) == 2


def test_stop_before_start_is_harmless(node):
    LivelinessBeacon(node).stop()
    assert node.destroyed == []


def test_assert_period_faster_than_lease_duration_by_default():
    """The Cyclone DDS gotcha: lease must be comfortably larger than the
    assert cadence or normal jitter reads as a false death. Default lease is
    3x the default assert period."""
    assert DEFAULT_LEASE_DURATION_SEC == pytest.approx(3 * DEFAULT_ASSERT_PERIOD_SEC)
    assert DEFAULT_ASSERT_PERIOD_SEC < DEFAULT_LEASE_DURATION_SEC
