"""Tests for ContextBuffer — ring behavior and snapshot retrieval."""
from helix_diagnosis.context_buffer import RosoutRing


def test_ring_bounded_to_capacity():
    r = RosoutRing(capacity=3)
    for i in range(10):
        r.append(f'line_{i}')
    snap = r.snapshot()
    assert len(snap) == 3
    assert snap == ['line_7', 'line_8', 'line_9']


def test_ring_empty_by_default():
    r = RosoutRing(capacity=5)
    assert r.snapshot() == []


def test_ring_under_capacity():
    r = RosoutRing(capacity=5)
    r.append('a')
    r.append('b')
    assert r.snapshot() == ['a', 'b']


# --- DiagnosticStatus.level normalisation -----------------------------------
# rclpy delivers the ROS `byte` field as a one-byte bytes object. int() on it
# raises ValueError, which killed ContextBuffer inside its own /helix/node_health
# callback about a second after activation in every run.

def test_level_accepts_the_bytes_form_rclpy_delivers():
    from helix_diagnosis.context_buffer import diagnostic_level
    assert diagnostic_level(b'\x00') == 0
    assert diagnostic_level(b'\x02') == 2


def test_level_accepts_a_plain_int():
    from helix_diagnosis.context_buffer import diagnostic_level
    assert diagnostic_level(2) == 2


def test_health_snapshot_survives_a_byte_level():
    import json

    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    from helix_diagnosis.context_buffer import ContextBuffer

    node = ContextBuffer()          # rclpy context comes from the session fixture
    try:
        msg = DiagnosticArray()
        status = DiagnosticStatus()
        status.name = 'helix_log_parser'
        status.level = DiagnosticStatus.ERROR
        status.message = 'no heartbeat'
        msg.status = [status]
        node._on_health(msg)        # used to raise ValueError and kill the node
        parsed = json.loads(node._latest_health)
        assert parsed['status'] == [
            {'name': 'helix_log_parser', 'level': 2, 'message': 'no heartbeat'}
        ]
    finally:
        node.destroy_node()
