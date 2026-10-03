"""Sink node timing: the deadman trips at the input deadline, not on the next poll.

In-process, dry_run (nothing is sent to a robot). Inputs go straight to the
node's callback; the trace publisher is captured.
"""
import json
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist  # noqa: E402
from helix_arbiter.go2_sport_sink import Go2SportSink  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402


@pytest.fixture
def sink():
    rclpy.init()
    node = Go2SportSink()
    traces = []
    node._pub_trace.publish = lambda m: traces.append(json.loads(m.data))
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    yield node, ex, traces
    ex.remove_node(node)
    node.destroy_node()
    rclpy.shutdown()


def _cmd(lx):
    t = Twist()
    t.linear.x = lx
    return t


def test_deadman_trips_at_the_input_deadline(sink):
    node, ex, traces = sink
    timeout = node.logic.input_timeout_sec
    lateness = []
    for phase in (0.0, 0.011, 0.023, 0.037, 0.049):
        end = time.monotonic() + 0.2 + phase
        while time.monotonic() < end:           # 50 Hz input, then silence
            node._on_cmd(_cmd(0.1))
            last_input = time.monotonic()
            ex.spin_once(timeout_sec=0.02)
        n = len(traces)
        deadline = last_input + 1.0
        while time.monotonic() < deadline and not any(
                t['reason'] == 'DEADMAN' for t in traces[n:]):
            ex.spin_once(timeout_sec=0.001)
        trips = [t for t in traces[n:] if t['reason'] == 'DEADMAN']
        assert trips, 'no DEADMAN StopMove after silence'
        lateness.append(trips[0]['t_mono'] - last_input - timeout)
    # Polling alone (50 ms) is late by up to 50 ms; the deadline timer is not.
    assert all(-0.001 <= x < 0.02 for x in lateness), lateness


def test_deadman_still_trips_before_any_input(sink):
    node, ex, traces = sink
    end = time.monotonic() + 0.3
    while time.monotonic() < end:
        ex.spin_once(timeout_sec=0.01)
    assert traces and traces[0]['reason'] == 'DEADMAN'
    assert traces[0]['api_id'] == 1003 and traces[0]['sent_to_robot'] is False
