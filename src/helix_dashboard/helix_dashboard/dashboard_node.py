"""
dashboard_node.py — HELIX Phase 4 ROS 2 bridge node.

Plain rclpy.Node (not lifecycle — dashboard runs always).
Named 'helix_dashboard_node'.

Subscribes to all HELIX topics and maintains an in-memory state dict.
On each incoming message, updates state and broadcasts the new state
to all connected WebSocket clients via api_server.broadcast().

The asyncio event loop for WebSocket handling is started here and
passed to api_server before uvicorn is started.
"""
import asyncio
import json
import threading
import time
from pathlib import Path
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node

from helix_msgs.msg import FaultEvent, LLMDiagnosis, RecoveryAction

import helix_dashboard.api_server as api_server

# Max items kept in each in-memory list
MAX_FAULTS = 50
MAX_RECOVERIES = 50
MAX_DIAGNOSES = 50
MAX_TIMELINE = 200


def msg_to_dict(msg: Any) -> Dict[str, Any]:
    """
    Convert a ROS 2 message to a plain dict for JSON serialisation.

    Handles: string, int, float, bool, list[str], bytes.
    Uses time.time() for the 'received_at' timestamp so the dashboard
    always has a wall-clock time even if the message timestamp is 0.
    """
    result: Dict[str, Any] = {"received_at": time.time()}
    for field in msg.get_fields_and_field_types().keys():
        val = getattr(msg, field, None)
        if val is None:
            result[field] = None
        elif isinstance(val, (str, int, float, bool)):
            result[field] = val
        elif isinstance(val, (list, tuple)):
            result[field] = list(val)
        elif isinstance(val, bytes):
            result[field] = val.decode(errors="replace")
        else:
            # Fallback: str() for nested message types we don't need to parse
            result[field] = str(val)
    return result


class DashboardNode(Node):
    """
    ROS 2 plain node that bridges HELIX topics to the FastAPI dashboard.

    State is a single dict protected by threading.Lock(). On each update
    the full state dict is serialised to JSON and broadcast to WebSocket
    clients via asyncio.run_coroutine_threadsafe.
    """

    def __init__(self, ws_loop: asyncio.AbstractEventLoop) -> None:
        super().__init__("helix_dashboard_node")

        self.declare_parameter("port", 8080)
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("max_fault_history", MAX_FAULTS)
        self.declare_parameter("max_recovery_history", MAX_RECOVERIES)

        self._ws_loop = ws_loop
        self._lock = threading.Lock()

        self._state: Dict[str, Any] = {
            "node_health": {},
            "recent_faults": [],
            "recent_recoveries": [],
            "recent_diagnoses": [],
            "stats": {
                "total_faults": 0,
                "total_recoveries": 0,
                "successful_recoveries": 0,
                "llm_calls": 0,
                "avg_inference_time": 0.0,
                "recovery_success_rate": 0.0,
            },
            "fault_timeline": [],
            "last_updated": 0.0,
        }
        api_server.set_state(self._state)

        # Subscriptions
        self.create_subscription(
            FaultEvent, "/helix/faults", self._on_fault, 50
        )
        self.create_subscription(
            RecoveryAction, "/helix/recovery_actions", self._on_recovery, 50
        )
        self.create_subscription(
            LLMDiagnosis, "/helix/llm_diagnoses", self._on_diagnosis, 50
        )

        # Optional: DiagnosticArray for node health (may not be published)
        try:
            from diagnostic_msgs.msg import DiagnosticArray
            self.create_subscription(
                DiagnosticArray, "/helix/node_health",
                self._on_node_health, 20
            )
        except ImportError:
            self.get_logger().warn("diagnostic_msgs not available — node health disabled")

        self.get_logger().info("DashboardNode initialised — subscribed to all HELIX topics")

    # ── ROS callbacks ──────────────────────────────────────────────────────────

    def _on_fault(self, msg: FaultEvent) -> None:
        fault_dict = msg_to_dict(msg)
        with self._lock:
            # Prepend (newest first)
            self._state["recent_faults"].insert(0, fault_dict)
            if len(self._state["recent_faults"]) > MAX_FAULTS:
                self._state["recent_faults"].pop()

            # Timeline entry
            self._state["fault_timeline"].append({
                "timestamp": fault_dict.get("timestamp", time.time()),
                "fault_type": msg.fault_type,
                "node_name": msg.node_name,
            })
            if len(self._state["fault_timeline"]) > MAX_TIMELINE:
                self._state["fault_timeline"].pop(0)

            self._state["stats"]["total_faults"] += 1
            self._state["last_updated"] = time.time()
            api_server.set_state(self._state)

        self._broadcast_state()

    def _on_recovery(self, msg: RecoveryAction) -> None:
        rec_dict = msg_to_dict(msg)
        with self._lock:
            self._state["recent_recoveries"].insert(0, rec_dict)
            if len(self._state["recent_recoveries"]) > MAX_RECOVERIES:
                self._state["recent_recoveries"].pop()

            stats = self._state["stats"]
            stats["total_recoveries"] += 1
            if msg.success:
                stats["successful_recoveries"] += 1
            total = stats["total_recoveries"]
            success = stats["successful_recoveries"]
            stats["recovery_success_rate"] = (success / total) if total > 0 else 0.0
            self._state["last_updated"] = time.time()
            api_server.set_state(self._state)

        self._broadcast_state()

    def _on_diagnosis(self, msg: LLMDiagnosis) -> None:
        diag_dict = msg_to_dict(msg)
        with self._lock:
            self._state["recent_diagnoses"].insert(0, diag_dict)
            if len(self._state["recent_diagnoses"]) > MAX_DIAGNOSES:
                self._state["recent_diagnoses"].pop()

            stats = self._state["stats"]
            stats["llm_calls"] += 1
            # Update running average inference time using duration if available
            dur = diag_dict.get("duration_sec", 0.0)
            if dur and dur > 0:
                prev_avg = stats.get("avg_inference_time", 0.0)
                n = stats["llm_calls"]
                stats["avg_inference_time"] = prev_avg + (dur - prev_avg) / n
            self._state["last_updated"] = time.time()
            api_server.set_state(self._state)

        self._broadcast_state()

    def _on_node_health(self, msg: Any) -> None:
        with self._lock:
            for status in msg.status:
                level_map = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}
                self._state["node_health"][status.name] = level_map.get(
                    status.level, "STALE"
                )
            self._state["last_updated"] = time.time()
            api_server.set_state(self._state)

        self._broadcast_state()

    # ── Broadcast ─────────────────────────────────────────────────────────────

    def _broadcast_state(self) -> None:
        """Push current state to all WebSocket clients (non-blocking)."""
        try:
            payload = json.dumps(self._state)
            asyncio.run_coroutine_threadsafe(
                api_server.broadcast(payload), self._ws_loop
            )
        except Exception as exc:
            self.get_logger().warn(f"Broadcast failed: {exc}")


def main(args=None) -> None:
    """Entry point for helix_dashboard_node console script."""
    rclpy.init(args=args)

    # Start the asyncio event loop for WebSocket handling in a daemon thread
    ws_loop = asyncio.new_event_loop()
    ws_thread = threading.Thread(
        target=ws_loop.run_forever, daemon=True, name="helix-ws-loop"
    )
    ws_thread.start()

    # Wire the loop into api_server before starting uvicorn
    api_server.set_ws_loop(ws_loop)

    # Locate the static directory (works both from source and installed)
    try:
        from ament_index_python.packages import get_package_share_directory
        static_dir = Path(get_package_share_directory("helix_dashboard")) / "static"
    except Exception:
        # Fallback: relative to this file (useful for development)
        static_dir = Path(__file__).parent / "static"
    api_server.set_static_dir(static_dir)

    # Create the node
    node = DashboardNode(ws_loop=ws_loop)

    # Start uvicorn in a daemon thread
    host = node.get_parameter("host").value
    port = node.get_parameter("port").value
    api_server.start_server(host=host, port=port)

    node.get_logger().info(
        f"HELIX Dashboard running at http://{host}:{port}"
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        ws_loop.call_soon_threadsafe(ws_loop.stop)
        rclpy.shutdown()
