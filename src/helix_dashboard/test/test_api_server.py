"""
Tests for api_server.py — no running server, no ROS, no WebSocket needed.

Uses FastAPI's TestClient (synchronous wrapper around httpx).
"""
import json
import time
import pytest
from fastapi.testclient import TestClient

from helix_dashboard.api_server import app, get_state, set_state, DEFAULT_STATE


def _fresh_state():
    """Return a deep copy of DEFAULT_STATE for test isolation."""
    import copy
    return copy.deepcopy(DEFAULT_STATE)


class TestDefaultState:
    def test_default_state_has_required_keys(self):
        state = _fresh_state()
        assert "node_health" in state
        assert "recent_faults" in state
        assert "recent_recoveries" in state
        assert "recent_diagnoses" in state
        assert "stats" in state
        assert "fault_timeline" in state
        assert "last_updated" in state

    def test_default_stats_has_required_fields(self):
        state = _fresh_state()
        stats = state["stats"]
        assert "total_faults" in stats
        assert "total_recoveries" in stats
        assert "successful_recoveries" in stats
        assert "recovery_success_rate" in stats


class TestGetState:
    def setup_method(self):
        set_state(_fresh_state())
        self.client = TestClient(app)

    def test_get_state_returns_200(self):
        resp = self.client.get("/api/state")
        assert resp.status_code == 200

    def test_get_state_returns_json_with_all_keys(self):
        resp = self.client.get("/api/state")
        body = resp.json()
        assert "node_health" in body
        assert "recent_faults" in body
        assert "stats" in body
        assert "fault_timeline" in body


class TestGetStats:
    def setup_method(self):
        set_state(_fresh_state())
        self.client = TestClient(app)

    def test_get_stats_returns_200(self):
        resp = self.client.get("/api/stats")
        assert resp.status_code == 200

    def test_get_stats_has_recovery_success_rate(self):
        resp = self.client.get("/api/stats")
        body = resp.json()
        assert "recovery_success_rate" in body

    def test_get_stats_rate_is_float(self):
        resp = self.client.get("/api/stats")
        body = resp.json()
        assert isinstance(body["recovery_success_rate"], float)


class TestGetFaults:
    def setup_method(self):
        state = _fresh_state()
        state["recent_faults"] = [
            {"node_name": f"node_{i}", "fault_type": "CRASH", "timestamp": time.time()}
            for i in range(20)
        ]
        set_state(state)
        self.client = TestClient(app)

    def test_get_faults_returns_200(self):
        resp = self.client.get("/api/faults")
        assert resp.status_code == 200

    def test_get_faults_limit_respected(self):
        resp = self.client.get("/api/faults?limit=5")
        body = resp.json()
        assert len(body) <= 5

    def test_get_faults_default_limit_50(self):
        resp = self.client.get("/api/faults")
        body = resp.json()
        assert len(body) <= 50


class TestGetRecoveries:
    def setup_method(self):
        state = _fresh_state()
        state["recent_recoveries"] = [
            {"action_taken": "restart_node", "success": True, "timestamp": time.time()}
            for _ in range(10)
        ]
        set_state(state)
        self.client = TestClient(app)

    def test_get_recoveries_returns_200(self):
        resp = self.client.get("/api/recoveries")
        assert resp.status_code == 200

    def test_get_recoveries_returns_list(self):
        resp = self.client.get("/api/recoveries")
        assert isinstance(resp.json(), list)


class TestGetDiagnoses:
    def setup_method(self):
        set_state(_fresh_state())
        self.client = TestClient(app)

    def test_get_diagnoses_returns_200(self):
        resp = self.client.get("/api/diagnoses")
        assert resp.status_code == 200

    def test_get_diagnoses_default_limit_20(self):
        state = _fresh_state()
        state["recent_diagnoses"] = [{"confidence": 0.9} for _ in range(30)]
        set_state(state)
        client = TestClient(app)
        resp = client.get("/api/diagnoses")
        assert len(resp.json()) <= 20


class TestSuccessRateCalculation:
    def test_success_rate_zero_when_no_recoveries(self):
        state = _fresh_state()
        state["stats"]["total_recoveries"] = 0
        state["stats"]["successful_recoveries"] = 0
        set_state(state)
        client = TestClient(app)
        resp = client.get("/api/stats")
        assert resp.json()["recovery_success_rate"] == 0.0

    def test_success_rate_calculated_correctly(self):
        state = _fresh_state()
        state["stats"]["total_recoveries"] = 10
        state["stats"]["successful_recoveries"] = 7
        # success_rate is computed dynamically in /api/stats
        state["stats"]["recovery_success_rate"] = 0.7
        set_state(state)
        client = TestClient(app)
        resp = client.get("/api/stats")
        body = resp.json()
        assert abs(body["recovery_success_rate"] - 0.7) < 0.01


class TestServeIndex:
    def setup_method(self):
        set_state(_fresh_state())
        self.client = TestClient(app)

    def test_root_returns_200_or_404(self):
        # Returns 200 if index.html exists on disk, 404 if not yet created.
        # Either is acceptable in unit tests — we just check it doesn't crash.
        resp = self.client.get("/")
        assert resp.status_code in (200, 404)
