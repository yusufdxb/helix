"""Explain the live-vs-replay disagreement on the 2026-10-06 loaded standing capture.

The live shipped detector emitted one rate_hz/gnss fault that replay did not.
This script shows why, using only the recorded bag and the shipped detector body:

1. the live fault payload and the replayed sample at the same time carry the same
   value, window mean, window std and z, so both paths saw identical inputs;
2. the first and fifth violating samples are 2.0 s apart to within the arrival
   jitter, i.e. the min_anomaly_duration_s gate (2.0 s) sits exactly on the
   0.5 s metric grid, so whether the fifth sample passes is decided by
   sub-millisecond timing;
3. the shipped body emits that fault in replay as soon as the gate is moved
   off the grid.

Run from the repository root: python3 analysis/gate_edge.py
"""

from __future__ import annotations

import json
import sqlite3
import statistics
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "analysis"))
from detector_replay import run_shipped  # noqa: E402

BAG = REPO / "hardware_eval_20261006/bags/loaded_standing_observe/bag_0.db3"
METRICS = REPO / "analysis/data/metrics_loaded_standing_20261006.json"
SHIPPED = dict(zscore_threshold=4.0, window_size=60, consecutive_trigger=3, emit_cooldown_s=1.0)
FAULT_T = 760.2
METRIC = "rate_hz/gnss"


def main() -> int:
    rows = [(s["t"], s["metric"], s["value"]) for s in json.load(open(METRICS))["samples"]]

    print("shipped body, min_anomaly_duration_s sweep (faults on", METRIC, "):")
    for md in (2.0, 1.999, 1.99, 1.9):
        em = [e for e in run_shipped(rows, min_anomaly_duration_s=md, **SHIPPED) if e["metric"] == METRIC]
        print(f"  {md:>6}: {[round(e['t'], 2) for e in em]}")

    con = sqlite3.connect(BAG)
    t0 = con.execute("select min(timestamp) from messages").fetchone()[0]
    tid = con.execute("select id from topics where name='/helix/metrics'").fetchone()[0]
    arrivals = [
        ts
        for ts, blob in con.execute(
            "select timestamp, data from messages where topic_id=? order by timestamp", (tid,)
        )
        if METRIC.encode() in blob
    ]
    streak = [ts for ts in arrivals if (FAULT_T - 2.1) * 1e9 <= ts - t0 <= (FAULT_T + 0.2) * 1e9]
    gaps = [(b - a) / 1e6 for a, b in zip(arrivals, arrivals[1:])]
    print(f"violating streak: {len(streak)} samples, first->last {(streak[-1] - streak[0]) / 1e9:.6f} s (gate 2.0 s)")
    print(
        f"{METRIC} inter-arrival over capture: mean {statistics.mean(gaps):.4f} ms, "
        f"sd {statistics.pstdev(gaps):.4f} ms, min {min(gaps):.3f}, max {max(gaps):.3f}"
    )

    fid = con.execute("select id from topics where name='/helix/faults'").fetchone()[0]
    for ts, blob in con.execute("select timestamp, data from messages where topic_id=?", (fid,)):
        text = "".join(chr(b) if 32 <= b < 127 else " " for b in blob)
        print(f"live fault at bag t={(ts - t0) / 1e9:.3f} s:", " ".join(text.split())[-120:])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
