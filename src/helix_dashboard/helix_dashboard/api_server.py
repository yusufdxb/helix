"""
api_server.py — HELIX Phase 4 FastAPI backend.

Endpoints:
  GET  /              → serves index.html
  GET  /api/state     → full in-memory state dict
  GET  /api/stats     → stats sub-dict
  GET  /api/faults    → recent_faults (limit param)
  GET  /api/recoveries → recent_recoveries (limit param)
  GET  /api/diagnoses → recent_diagnoses (limit param)
  GET  /api/db/stats  → historical stats from SQLite via StateDB
  WS   /ws            → real-time state broadcast

State is owned by dashboard_node.py and mutated via set_state().
api_server.py reads it read-only (no lock needed — Python GIL + dict read is safe
for the snapshot use case; correctness > perfect consistency here).
"""
import asyncio
import json
import os
import time
import threading
from pathlib import Path
from typing import Any, Dict, Optional, Set

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse

# ── State (set by dashboard_node at startup) ──────────────────────────────────

DEFAULT_STATE: Dict[str, Any] = {
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

_state: Dict[str, Any] = DEFAULT_STATE.copy()
_connected_clients: Set[WebSocket] = set()
_ws_loop: Optional[asyncio.AbstractEventLoop] = None

# Path to the static directory — set at startup by dashboard_node
_static_dir: Optional[Path] = None


def get_state() -> Dict[str, Any]:
    """Return the current state dict (snapshot)."""
    return _state


def set_state(new_state: Dict[str, Any]) -> None:
    """Replace the current state (called by dashboard_node on each update)."""
    global _state
    _state = new_state


def set_ws_loop(loop: asyncio.AbstractEventLoop) -> None:
    """Register the asyncio event loop used by the WebSocket handler."""
    global _ws_loop
    _ws_loop = loop


def set_static_dir(path: Path) -> None:
    """Set the directory containing index.html."""
    global _static_dir
    _static_dir = path


def get_connected_clients() -> Set[WebSocket]:
    return _connected_clients


# ── FastAPI app ───────────────────────────────────────────────────────────────

app = FastAPI(title="HELIX Dashboard API", version="0.1.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


# ── REST endpoints ────────────────────────────────────────────────────────────

@app.get("/api/state")
async def api_state() -> JSONResponse:
    return JSONResponse(_state)


@app.get("/api/stats")
async def api_stats() -> JSONResponse:
    stats = dict(_state.get("stats", {}))
    # Recompute success rate dynamically
    total = stats.get("total_recoveries", 0)
    success = stats.get("successful_recoveries", 0)
    stats["recovery_success_rate"] = (success / total) if total > 0 else 0.0
    return JSONResponse(stats)


@app.get("/api/faults")
async def api_faults(limit: int = 50) -> JSONResponse:
    faults = _state.get("recent_faults", [])
    return JSONResponse(faults[:limit])


@app.get("/api/recoveries")
async def api_recoveries(limit: int = 50) -> JSONResponse:
    recoveries = _state.get("recent_recoveries", [])
    return JSONResponse(recoveries[:limit])


@app.get("/api/diagnoses")
async def api_diagnoses(limit: int = 20) -> JSONResponse:
    diagnoses = _state.get("recent_diagnoses", [])
    return JSONResponse(diagnoses[:limit])


@app.get("/api/db/stats")
async def api_db_stats() -> JSONResponse:
    """Return historical stats from SQLite via StateDB."""
    try:
        from helix_recovery.state_db import StateDB, DEFAULT_DB_PATH
        db = StateDB(db_path=DEFAULT_DB_PATH)
        return JSONResponse(db.get_all_stats())
    except Exception as exc:
        return JSONResponse({"error": str(exc), "available": False}, status_code=503)


@app.get("/")
async def serve_index():
    if _static_dir is None:
        return JSONResponse({"error": "static dir not configured"}, status_code=404)
    index = _static_dir / "index.html"
    if not index.exists():
        return JSONResponse({"error": "index.html not found"}, status_code=404)
    return FileResponse(str(index))


# ── WebSocket ─────────────────────────────────────────────────────────────────

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket) -> None:
    await ws.accept()
    _connected_clients.add(ws)
    try:
        # Send full current state on connect
        await ws.send_text(json.dumps(_state))
        # Keep connection alive; actual updates pushed by dashboard_node via broadcast()
        while True:
            try:
                # Wait for client ping or disconnect (20s timeout)
                await asyncio.wait_for(ws.receive_text(), timeout=20.0)
            except asyncio.TimeoutError:
                # Send ping to keep alive
                await ws.send_text(json.dumps({"ping": True}))
    except WebSocketDisconnect:
        pass
    finally:
        _connected_clients.discard(ws)


async def broadcast(message: str) -> None:
    """Send message to all connected WebSocket clients."""
    dead: Set[WebSocket] = set()
    for ws in list(_connected_clients):
        try:
            await ws.send_text(message)
        except Exception:
            dead.add(ws)
    _connected_clients.difference_update(dead)


# ── Server startup ────────────────────────────────────────────────────────────

def start_server(host: str = "0.0.0.0", port: int = 8080) -> None:
    """
    Start uvicorn in a daemon thread.

    Called by dashboard_node.py after the asyncio WS loop is set.
    """
    def _run() -> None:
        uvicorn.run(app, host=host, port=port, log_level="warning")

    t = threading.Thread(target=_run, daemon=True, name="helix-uvicorn")
    t.start()
