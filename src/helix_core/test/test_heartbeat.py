"""Heartbeat emitter tests.

The emitter is what makes a node death observable: HeartbeatMonitor only
reports CRASH for a node it has already seen alive on /helix/heartbeat.
"""
import pytest
from helix_core.heartbeat import DEFAULT_PERIOD_SEC, HEARTBEAT_TOPIC, Heartbeat


class FakeTimer:
    def __init__(self, period, callback):
        self.period = period
        self.callback = callback


class FakePublisher:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class FakeNode:
    """Records what a lifecycle node would have created, without ROS running."""

    def __init__(self, name='helix_fake_node'):
        self._name = name
        self.publisher = None
        self.timers = []
        self.destroyed = []

    def get_name(self):
        return self._name

    def create_publisher(self, msg_type, topic, depth):
        self.topic = topic
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


def test_publisher_is_created_on_the_monitored_topic(node):
    Heartbeat(node)
    assert node.topic == HEARTBEAT_TOPIC
    assert HEARTBEAT_TOPIC == '/helix/heartbeat'


def test_no_timer_runs_until_started(node):
    Heartbeat(node)
    assert node.timers == []


def test_start_publishes_the_owning_node_name(node):
    heartbeat = Heartbeat(node)
    heartbeat.start()
    node.timers[0].callback()
    assert [msg.data for msg in node.publisher.published] == ['helix_fake_node']


def test_start_is_idempotent(node):
    heartbeat = Heartbeat(node)
    heartbeat.start()
    heartbeat.start()
    assert len(node.timers) == 1


def test_stop_destroys_the_timer_and_allows_restart(node):
    heartbeat = Heartbeat(node)
    heartbeat.start()
    first = node.timers[0]
    heartbeat.stop()
    assert node.destroyed == [first]
    heartbeat.start()
    assert len(node.timers) == 2


def test_stop_before_start_is_harmless(node):
    Heartbeat(node).stop()
    assert node.destroyed == []


def test_period_beats_faster_than_the_monitor_timeout(node):
    """0.3 s timeout x 3 misses must leave room for dropped beats."""
    heartbeat = Heartbeat(node)
    heartbeat.start()
    assert node.timers[0].period == DEFAULT_PERIOD_SEC
    assert DEFAULT_PERIOD_SEC < 0.3
