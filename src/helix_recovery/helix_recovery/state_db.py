"""
StateDB — HELIX Phase 2 persistence layer.

Pure Python SQLite database. No ROS dependency.
Thread-safe via threading.Lock().
Default path: ~/.helix/helix_state.db
"""
import os
import sqlite3
import threading
import time
import uuid
from typing import Any, Dict, List

# ── Constants ────────────────────────────────────────────────────────────────
DEFAULT_DB_DIR: str = os.path.expanduser("~/.helix")
DEFAULT_DB_PATH: str = os.path.join(DEFAULT_DB_DIR, "helix_state.db")

_CREATE_FAULT_EVENTS: str = """
CREATE TABLE IF NOT EXISTS fault_events (
    id TEXT PRIMARY KEY,
    node_name TEXT,
    fault_type TEXT,
    severity INTEGER,
    detail TEXT,
    timestamp REAL,
    created_at REAL
)
"""

_CREATE_RECOVERY_ATTEMPTS: str = """
CREATE TABLE IF NOT EXISTS recovery_attempts (
    id TEXT PRIMARY KEY,
    fault_id TEXT,
    action_taken TEXT,
    tier INTEGER,
    attempt_number INTEGER,
    success INTEGER,
    duration_sec REAL,
    outcome_detail TEXT,
    timestamp REAL,
    FOREIGN KEY (fault_id) REFERENCES fault_events(id)
)
"""

_CREATE_NODE_HISTORY: str = """
CREATE TABLE IF NOT EXISTS node_recovery_history (
    node_name TEXT,
    fault_type TEXT,
    total_attempts INTEGER DEFAULT 0,
    successful_recoveries INTEGER DEFAULT 0,
    last_fault_timestamp REAL,
    PRIMARY KEY (node_name, fault_type)
)
"""


class StateDB:
    """Thread-safe SQLite persistence for HELIX fault and recovery data."""

    def __init__(self, db_path: str = DEFAULT_DB_PATH) -> None:
        """
        Initialize the database, creating tables if they don't exist.

        Args:
            db_path: Absolute path to the SQLite file. Directory is created
                     automatically if it doesn't exist.
        """
        self._db_path = db_path
        self._lock = threading.Lock()
        os.makedirs(os.path.dirname(db_path), exist_ok=True)
        self._init_schema()

    def _init_schema(self) -> None:
        """Create all tables if they don't already exist."""
        with self._connect() as conn:
            conn.execute(_CREATE_FAULT_EVENTS)
            conn.execute(_CREATE_RECOVERY_ATTEMPTS)
            conn.execute(_CREATE_NODE_HISTORY)

    def _connect(self) -> sqlite3.Connection:
        """Return a new SQLite connection with WAL mode for concurrency."""
        conn = sqlite3.connect(self._db_path)
        conn.row_factory = sqlite3.Row
        conn.execute("PRAGMA journal_mode=WAL")
        return conn

    # ── Public API ───────────────────────────────────────────────────────────

    def insert_fault(self, fault_event_dict: Dict[str, Any]) -> str:
        """
        Insert a fault event record.

        Args:
            fault_event_dict: Keys: node_name, fault_type, severity, detail, timestamp

        Returns:
            UUID string for the inserted record.
        """
        fault_id = str(uuid.uuid4())
        now = time.time()
        with self._lock:
            with self._connect() as conn:
                conn.execute(
                    "INSERT INTO fault_events "
                    "(id, node_name, fault_type, severity, detail, timestamp, created_at) "
                    "VALUES (?, ?, ?, ?, ?, ?, ?)",
                    (
                        fault_id,
                        fault_event_dict.get("node_name", ""),
                        fault_event_dict.get("fault_type", ""),
                        fault_event_dict.get("severity", 0),
                        fault_event_dict.get("detail", ""),
                        fault_event_dict.get("timestamp", now),
                        now,
                    ),
                )
        return fault_id

    def insert_recovery_attempt(self, attempt_dict: Dict[str, Any]) -> str:
        """
        Insert a recovery attempt record.

        Args:
            attempt_dict: Keys: fault_id, action_taken, tier, attempt_number,
                          success, duration_sec, outcome_detail, timestamp

        Returns:
            UUID string for the inserted record.
        """
        attempt_id = str(uuid.uuid4())
        now = time.time()
        with self._lock:
            with self._connect() as conn:
                conn.execute(
                    "INSERT INTO recovery_attempts "
                    "(id, fault_id, action_taken, tier, attempt_number, "
                    "success, duration_sec, outcome_detail, timestamp) "
                    "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                    (
                        attempt_id,
                        attempt_dict.get("fault_id", ""),
                        attempt_dict.get("action_taken", ""),
                        attempt_dict.get("tier", 1),
                        attempt_dict.get("attempt_number", 1),
                        1 if attempt_dict.get("success", False) else 0,
                        attempt_dict.get("duration_sec", 0.0),
                        attempt_dict.get("outcome_detail", ""),
                        attempt_dict.get("timestamp", now),
                    ),
                )
        return attempt_id

    def get_recovery_history(
        self, node_name: str, fault_type: str
    ) -> Dict[str, Any]:
        """
        Return the node_recovery_history row for (node_name, fault_type).

        Returns an empty dict with zeroed fields if no history exists yet.
        """
        with self._lock:
            with self._connect() as conn:
                row = conn.execute(
                    "SELECT * FROM node_recovery_history "
                    "WHERE node_name = ? AND fault_type = ?",
                    (node_name, fault_type),
                ).fetchone()
        if row is None:
            return {
                "node_name": node_name,
                "fault_type": fault_type,
                "total_attempts": 0,
                "successful_recoveries": 0,
                "last_fault_timestamp": 0.0,
            }
        return dict(row)

    def get_recent_faults(
        self, node_name: str, limit: int = 10
    ) -> List[Dict[str, Any]]:
        """
        Return the most recent fault events for a node, newest first.

        Args:
            node_name: Node to filter by.
            limit: Maximum number of rows to return.
        """
        with self._lock:
            with self._connect() as conn:
                rows = conn.execute(
                    "SELECT * FROM fault_events WHERE node_name = ? "
                    "ORDER BY created_at DESC LIMIT ?",
                    (node_name, limit),
                ).fetchall()
        return [dict(r) for r in rows]

    def update_node_history(
        self, node_name: str, fault_type: str, success: bool
    ) -> None:
        """
        Upsert node_recovery_history, incrementing attempt/success counters.

        Args:
            node_name: Name of the node that was recovered.
            fault_type: Fault type that triggered recovery.
            success: Whether this recovery attempt succeeded.
        """
        now = time.time()
        with self._lock:
            with self._connect() as conn:
                conn.execute(
                    """
                    INSERT INTO node_recovery_history
                        (node_name, fault_type, total_attempts,
                         successful_recoveries, last_fault_timestamp)
                    VALUES (?, ?, 1, ?, ?)
                    ON CONFLICT(node_name, fault_type) DO UPDATE SET
                        total_attempts = total_attempts + 1,
                        successful_recoveries = successful_recoveries + ?,
                        last_fault_timestamp = ?
                    """,
                    (
                        node_name,
                        fault_type,
                        1 if success else 0,
                        now,
                        1 if success else 0,
                        now,
                    ),
                )

    def get_all_stats(self) -> Dict[str, Any]:
        """
        Return aggregate statistics across all faults and recovery attempts.

        Returns a dict with keys: total_faults, total_recovery_attempts,
        success_rate, by_fault_type (dict of fault_type → count).
        """
        with self._lock:
            with self._connect() as conn:
                total_faults = conn.execute(
                    "SELECT COUNT(*) FROM fault_events"
                ).fetchone()[0]
                total_attempts = conn.execute(
                    "SELECT COUNT(*) FROM recovery_attempts"
                ).fetchone()[0]
                successes = conn.execute(
                    "SELECT COUNT(*) FROM recovery_attempts WHERE success = 1"
                ).fetchone()[0]
                by_type_rows = conn.execute(
                    "SELECT fault_type, COUNT(*) as cnt FROM fault_events GROUP BY fault_type"
                ).fetchall()

        success_rate = (successes / total_attempts) if total_attempts > 0 else 0.0
        by_fault_type = {r["fault_type"]: r["cnt"] for r in by_type_rows}

        return {
            "total_faults": total_faults,
            "total_recovery_attempts": total_attempts,
            "success_rate": round(success_rate, 3),
            "by_fault_type": by_fault_type,
        }
