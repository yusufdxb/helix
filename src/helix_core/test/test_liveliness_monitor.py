"""
Unit tests for LivelinessMonitor's own logic: parameter-gated subscription
creation and the liveliness-changed -> FaultEvent decision, exercised
against a real (but disabled-by-default) LifecycleNode instance.

rclpy is initialized once for the whole test/ directory by conftest.py.

The actual DDS-level claim (a lease violation really fires
RCL_SUBSCRIPTION_LIVELINESS_CHANGED and this monitor really detects it) is
proven end-to-end in test_liveliness_integration.py; this file only checks
the decision logic once an event (real or fake) arrives.
"""

import pytest
from helix_core.liveliness_monitor import LivelinessMonitor
from rclpy.parameter import Parameter


class FakeLivelinessEvent:
    """Duck-types rclpy.qos_event.QoSLivelinessChangedInfo (a pybind struct)
    for pure decision-logic tests that don't need a live DDS session."""

    def __init__(self, alive_count=0, not_alive_count=0,
                 alive_count_change=0, not_alive_count_change=0):
        self.alive_count = alive_count
        self.not_alive_count = not_alive_count
        self.alive_count_change = alive_count_change
        self.not_alive_count_change = not_alive_count_change


@pytest.fixture
def disabled_monitor():
    """Default parameters: enabled=False. Matches the required default of
    not changing current CRASH-detection behavior."""
    node = LivelinessMonitor()
    node.trigger_configure()
    yield node
    node.trigger_cleanup()
    node.destroy_node()


def test_disabled_by_default_creates_no_subscriptions(disabled_monitor):
    assert disabled_monitor.get_parameter('enabled').value is False
    assert disabled_monitor._subs == []


def test_enabled_false_still_creates_the_fault_publisher(disabled_monitor):
    """Even disabled, the fault publisher exists so re-enabling later (e.g.
    a future parameter update) doesn't need a full reconfigure."""
    assert disabled_monitor._fault_pub is not None


def test_liveliness_changed_not_alive_emits_one_crash_fault():
    node = LivelinessMonitor()
    node.trigger_configure()
    try:
        received = []
        node._fault_pub.publish = lambda msg: received.append(msg)

        event = FakeLivelinessEvent(alive_count=0, not_alive_count=1,
                                     not_alive_count_change=1)
        node._on_liveliness_changed('helix_fake_crashed_node', event)

        assert len(received) == 1
        assert received[0].node_name == 'helix_fake_crashed_node'
        assert received[0].fault_type == 'CRASH'
        assert received[0].severity == 3
        assert 'dds_liveliness' in received[0].context_values
    finally:
        node.trigger_cleanup()
        node.destroy_node()


def test_liveliness_changed_suppresses_repeat_crash_until_alive_again():
    """Same contract as HeartbeatMonitor's _crashed_nodes set: don't flood
    /helix/faults with one fault per residual not-alive report."""
    node = LivelinessMonitor()
    node.trigger_configure()
    try:
        received = []
        node._fault_pub.publish = lambda msg: received.append(msg)

        event = FakeLivelinessEvent(not_alive_count=1, not_alive_count_change=1)
        node._on_liveliness_changed('helix_fake_node', event)
        node._on_liveliness_changed('helix_fake_node', event)
        assert len(received) == 1

        alive_again = FakeLivelinessEvent(alive_count=1, alive_count_change=1)
        node._on_liveliness_changed('helix_fake_node', alive_again)
        node._on_liveliness_changed('helix_fake_node', event)
        assert len(received) == 2
    finally:
        node.trigger_cleanup()
        node.destroy_node()


def test_enabled_true_with_monitored_nodes_creates_one_subscription_each():
    node = LivelinessMonitor()
    node.set_parameters([
        Parameter('enabled', Parameter.Type.BOOL, True),
        Parameter('monitored_nodes', Parameter.Type.STRING_ARRAY,
                   ['helix_context_buffer', 'helix_diagnosis_node']),
    ])
    node.trigger_configure()
    try:
        assert len(node._subs) == 2
        assert set(node._crashed.keys()) == {'helix_context_buffer', 'helix_diagnosis_node'}
    finally:
        node.trigger_cleanup()
        node.destroy_node()
