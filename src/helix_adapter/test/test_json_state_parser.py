"""Integration tests for JsonStateParser lifecycle node."""
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float64MultiArray, String


@pytest.fixture
def node():
    from helix_adapter.json_state_parser import JsonStateParser

    n = JsonStateParser()
    n.trigger_configure()
    n.trigger_activate()
    yield n
    n.trigger_deactivate()
    n.trigger_cleanup()
    n.destroy_node()


def _spin_until(executor, predicate, timeout_sec: float) -> bool:
    """Spin until predicate() is true or the deadline passes."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if predicate():
            return True
        executor.spin_once(timeout_sec=0.02)
    return predicate()


def _collect_metrics(node, seconds: float = 0.5):
    received: list = []
    helper = rclpy.create_node("test_json_helper")
    gnss_pub = helper.create_publisher(String, "/gnss", 10)
    mstate_pub = helper.create_publisher(String, "/multiplestate", 10)
    helper.create_subscription(
        Float64MultiArray, "/helix/metrics",
        lambda m: received.append((m.layout.dim[0].label, m.data[0])),
        10,
    )
    exec_ = SingleThreadedExecutor()
    exec_.add_node(node)
    exec_.add_node(helper)

    # Wait for both publishers to be discovered by JsonStateParser before
    # publishing; otherwise the first message races discovery and is dropped.
    _spin_until(
        exec_,
        lambda: (gnss_pub.get_subscription_count() >= 1
                 and mstate_pub.get_subscription_count() >= 1),
        timeout_sec=2.0,
    )

    # Republish across the collection window so a single dropped message
    # doesn't make the test fail; JsonStateParser keeps the latest payload
    # and re-emits on its publish timer.
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        gnss_pub.publish(String(
            data='{"satellite_total": 13, "satellite_inuse": 8, "hdop": 1.2}'))
        mstate_pub.publish(String(
            data='{"volume": 7, "brightness": 3, "obstaclesAvoidSwitch": true, '
                 '"uwbSwitch": false}'))
        for _ in range(5):
            exec_.spin_once(timeout_sec=0.02)

    helper.destroy_node()
    exec_.shutdown()
    return received


def test_parses_gnss_and_multiplestate(node):
    received = _collect_metrics(node, seconds=1.0)
    labels = {label: value for label, value in received}

    assert labels.get("gnss/satellite_total") == 13.0
    assert labels.get("gnss/satellite_inuse") == 8.0
    assert labels.get("gnss/hdop") == 1.2
    assert labels.get("go2_state/volume") == 7.0
    assert labels.get("go2_state/brightness") == 3.0
    assert labels.get("go2_state/obstaclesAvoidSwitch") == 1.0
    assert labels.get("go2_state/uwbSwitch") == 0.0


def test_invalid_json_is_silently_dropped(node):
    """A bad JSON string should not raise or propagate."""
    received: list = []
    helper = rclpy.create_node("test_json_bad_helper")
    gnss_pub = helper.create_publisher(String, "/gnss", 10)
    helper.create_subscription(
        Float64MultiArray, "/helix/metrics",
        lambda m: received.append(m.layout.dim[0].label),
        10,
    )
    exec_ = SingleThreadedExecutor()
    exec_.add_node(node)
    exec_.add_node(helper)

    _spin_until(
        exec_, lambda: gnss_pub.get_subscription_count() >= 1, timeout_sec=2.0
    )
    gnss_pub.publish(String(data='{not json'))
    gnss_pub.publish(String(data='null'))
    gnss_pub.publish(String(data='[1, 2, 3]'))

    start = time.monotonic()
    while time.monotonic() - start < 0.6:
        exec_.spin_once(timeout_sec=0.02)

    helper.destroy_node()
    exec_.shutdown()
    assert not any("gnss/" in label for label in received)


def test_invalid_source_spec_fails_configure():
    """Bad source spec leaves node in unconfigured state."""
    from helix_adapter.json_state_parser import JsonStateParser
    from rclpy.lifecycle import TransitionCallbackReturn

    n = JsonStateParser()
    n.set_parameters([
        rclpy.parameter.Parameter(
            "sources", rclpy.Parameter.Type.STRING_ARRAY,
            ["wrong_format_no_pipes"],
        ),
    ])
    result = n.trigger_configure()
    assert result == TransitionCallbackReturn.FAILURE
    n.destroy_node()
