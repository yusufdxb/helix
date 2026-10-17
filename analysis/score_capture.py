#!/usr/bin/env python3
"""Score a real GO2 HELIX capture against the shipped and proposed detector settings.

Reuses the analysis modules in this directory; no detector logic is reimplemented.
Usage: python3 analysis/score_capture.py [BAG_DIR] [OUT_JSON]
Defaults: the 2026-10-06 loaded-standing capture and
analysis/results/loaded_standing_20261006.json. Also writes
analysis/data/metrics_loaded_standing_20261006.json (extract_telemetry convention).
"""
import json
import math
import sqlite3
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO / "scripts"))
from detector_replay import STOP_METRIC_PREFIX, run_shipped, stop_episodes  # noqa: E402
from extract_telemetry import extract_recorded_metrics  # noqa: E402
from idle_margin import idle_scores, max_run  # noqa: E402
from validate_closed_loop_bag import helix_typestore  # noqa: E402

BAG = REPO / "hardware_eval_20261006" / "bags" / "loaded_standing_observe"
OUT_JSON = HERE / "results" / "loaded_standing_20261006.json"
METRICS_JSON = HERE / "data" / "metrics_loaded_standing_20261006.json"
if len(sys.argv) > 1:
    BAG = Path(sys.argv[1]).resolve()
if len(sys.argv) > 2:
    OUT_JSON = Path(sys.argv[2]).resolve()
CONFIGS = {
    "shipped": dict(zscore_threshold=4.0, window_size=60, consecutive_trigger=3, min_anomaly_duration_s=2.0, emit_cooldown_s=1.0),
    "proposed": dict(zscore_threshold=3.5, window_size=120, consecutive_trigger=3, min_anomaly_duration_s=2.0, emit_cooldown_s=1.0),
}

payload = extract_recorded_metrics(BAG, METRICS_JSON)
dur = payload["duration_s"]
series = {}
for s in payload["samples"]:
    v = math.nan if s["value"] is None else s["value"]
    series.setdefault(s["metric"], []).append((s["t"], v))
nan_counts = {m: sum(1 for _, v in x if math.isnan(v)) for m, x in series.items()}
samples = sorted((s["t"], s["metric"], math.nan if s["value"] is None else s["value"]) for s in payload["samples"])
streams = {"capture": series}

# live faults
ts = helix_typestore()
live = []
con = sqlite3.connect(str(BAG / "bag_0.db3"))
tid, tt = con.execute("select id,type from topics where name='/helix/faults'").fetchone()
t0 = con.execute("select min(timestamp) from messages where topic_id=(select id from topics where name='/helix/metrics')").fetchone()[0]
for stamp, blob in con.execute("select timestamp,data from messages where topic_id=?", (tid,)):
    m = ts.deserialize_cdr(bytes(blob), tt)
    live.append({"t_rel": (stamp - t0) / 1e9, "node": m.node_name, "type": m.fault_type, "detail": m.detail})

res = {"duration_s": dur, "n_samples": payload["n_samples"], "metrics": sorted(series),
       "samples_per_metric": {m: len(x) for m, x in series.items()}, "nan_samples_per_metric": nan_counts,
       "utlidar_robot_odom_present": "rate_hz/utlidar_robot_odom" in series,
       "live_faults": live, "configs": {}}
for name, p in CONFIGS.items():
    faults = run_shipped(samples, **p)
    flood = run_shipped(samples, **{**p, "emit_cooldown_s": 0.0})
    eps = stop_episodes(faults)
    n_stop = sum(1 for f in faults if f["metric"].startswith(STOP_METRIC_PREFIX))
    scores = idle_scores(streams, "zscore", p["window_size"])
    runs = {m: max_run(v, p["zscore_threshold"]) for m, v in scores.items()}
    res["configs"][name] = {
        "params": p, "false_faults": len(faults), "false_faults_per_hour": len(faults) * 3600 / dur,
        "false_faults_no_cooldown": len(flood),
        "faults_by_metric": {m: sum(1 for f in faults if f["metric"] == m) for m in {f["metric"] for f in faults}},
        "stop_producing_faults": n_stop, "stop_producing_per_hour": n_stop * 3600 / dur,
        "stop_episodes": len(eps), "stop_episodes_per_hour": len(eps) * 3600 / dur,
        "longest_above_threshold_run_per_metric": runs,
        "longest_run_overall": max(runs.values()) if runs else 0, "needed_to_emit": 5,
        "max_score_per_metric": {m: max(v) for m, v in scores.items() if v}}
OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
OUT_JSON.write_text(json.dumps(res, indent=1))
sh = res["configs"]["shipped"]
print(f"duration {dur:.1f}s samples {res['n_samples']} metrics {len(series)}")
print("metrics:", res["metrics"])
print("nan:", {k: v for k, v in nan_counts.items() if v})
print("rate_hz/utlidar_robot_odom present:", res["utlidar_robot_odom_present"])
print(f"LIVE faults {len(live)} vs REPLAY shipped faults {sh['false_faults']} -> {'MATCH' if len(live)==sh['false_faults'] else 'MISMATCH'}")
for f in live:
    print("  live:", f)
for n, c in res["configs"].items():
    print(n, "faults", c["false_faults"], f"({c['false_faults_per_hour']:.2f}/h)", "nocooldown", c["false_faults_no_cooldown"],
          "stop-faults", c["stop_producing_faults"], "stop-episodes", c["stop_episodes"], "longest run", c["longest_run_overall"], "of 5")
    print("  per-metric run:", c["longest_above_threshold_run_per_metric"])
    print("  by metric:", c["faults_by_metric"])
