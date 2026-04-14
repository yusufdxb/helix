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
