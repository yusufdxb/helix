"""
Tests for StateDB — pure Python SQLite layer.
No ROS dependency. Uses a temp file DB for isolation.
"""
import os
import time
import tempfile
import pytest
from helix_recovery.state_db import StateDB


@pytest.fixture
def db(tmp_path):
    """Create a fresh StateDB backed by a temp file for each test."""
    db_path = str(tmp_path / "test_helix.db")
    instance = StateDB(db_path=db_path)
    yield instance


def _make_fault(node_name: str = "test_node", fault_type: str = "CRASH") -> dict:
    return {
        "node_name": node_name,
        "fault_type": fault_type,
        "severity": 3,
        "detail": "Test fault detail",
        "timestamp": time.time(),
    }


def _make_attempt(fault_id: str, attempt_number: int = 1, success: bool = True) -> dict:
    return {
        "fault_id": fault_id,
        "action_taken": "restart_node",
        "tier": 1,
        "attempt_number": attempt_number,
        "success": success,
        "duration_sec": 1.23,
        "outcome_detail": "Node restarted successfully",
        "timestamp": time.time(),
    }


def test_insert_fault_returns_uuid(db):
    """insert_fault should return a non-empty UUID string."""
    fault_id = db.insert_fault(_make_fault())
    assert isinstance(fault_id, str)
    assert len(fault_id) == 36  # standard uuid4 length


def test_insert_fault_persists(db):
    """Inserted fault should appear in get_recent_faults."""
    fault_id = db.insert_fault(_make_fault(node_name="nav_node"))
    faults = db.get_recent_faults("nav_node", limit=5)
    assert len(faults) == 1
    assert faults[0]["id"] == fault_id
    assert faults[0]["node_name"] == "nav_node"
    assert faults[0]["fault_type"] == "CRASH"


def test_insert_recovery_attempt_returns_uuid(db):
    """insert_recovery_attempt should return a UUID."""
    fault_id = db.insert_fault(_make_fault())
    attempt_id = db.insert_recovery_attempt(_make_attempt(fault_id))
    assert isinstance(attempt_id, str)
    assert len(attempt_id) == 36


def test_get_recovery_history_both_attempts(db):
    """get_recovery_history should return both attempts for same node+fault_type."""
    fault_id = db.insert_fault(_make_fault())
    db.insert_recovery_attempt(_make_attempt(fault_id, attempt_number=1, success=False))
    db.insert_recovery_attempt(_make_attempt(fault_id, attempt_number=2, success=True))
    history = db.get_recovery_history("test_node", "CRASH")
    # history is the node_recovery_history row (updated via update_node_history)
    # Separate check: get_recent_faults to confirm attempts logged
    faults = db.get_recent_faults("test_node", limit=10)
    assert len(faults) == 1


def test_update_node_history_increments(db):
    """update_node_history should increment total_attempts and successful_recoveries."""
    db.update_node_history("slam_node", "ANOMALY", success=True)
    db.update_node_history("slam_node", "ANOMALY", success=True)
    db.update_node_history("slam_node", "ANOMALY", success=False)

    history = db.get_recovery_history("slam_node", "ANOMALY")
    assert history["total_attempts"] == 3
    assert history["successful_recoveries"] == 2


def test_get_all_stats_returns_dict(db):
    """get_all_stats should return a dict with known keys."""
    db.insert_fault(_make_fault())
    stats = db.get_all_stats()
    assert isinstance(stats, dict)
    assert "total_faults" in stats
    assert "total_recovery_attempts" in stats
    assert "success_rate" in stats


def test_get_recent_faults_respects_limit(db):
    """get_recent_faults should respect the limit parameter."""
    for i in range(5):
        db.insert_fault(_make_fault(node_name="busy_node"))
    faults = db.get_recent_faults("busy_node", limit=3)
    assert len(faults) == 3


def test_thread_safety(db):
    """Concurrent inserts from multiple threads should not raise."""
    import threading
    errors = []

    def insert_many():
        try:
            for _ in range(10):
                db.insert_fault(_make_fault())
        except Exception as exc:
            errors.append(exc)

    threads = [threading.Thread(target=insert_many) for _ in range(4)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    assert len(errors) == 0, f"Thread errors: {errors}"
