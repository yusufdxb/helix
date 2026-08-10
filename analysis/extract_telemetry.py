#!/usr/bin/env python3
"""Extract real recorded telemetry from HELIX hardware bags into flat npz/json.

Two independent real sources are produced:

  A) RECORDED-METRICS  (analysis/data/metrics_<session>.json)
     The literal /helix/metrics stream that the shipped anomaly_detector
     subscribed to on real GO2 hardware. This is the detector's true input,
     recorded, not reconstructed.

  B) RECONSTRUCTED-RATE (analysis/data/recon_extended_5min.json)
     The extended_5min bag did not record /helix/metrics, but it did record
     the raw arrival timestamps of exactly the topics whose rate_hz metrics
     produced Session 8's false alarms (/utlidar/imu, /gnss, /multiplestate,
     /utlidar/cloud, /utlidar/robot_pose). We replay helix_adapter's
     RateWindow (window_sec=5.0) plus the 0.5 s publish timer over those real
     timestamps to regenerate the metric stream the detector would have seen.

     ASSUMPTION: rosbag2 stores the recorder process's receive timestamp.
     RateWindow uses the monitor node's callback-dispatch time. These are
     different processes on the same host, so the reconstruction inherits the
     recorder's scheduling jitter rather than the monitor's. Both are
     receiver-side arrival times on the same machine and the same DDS
     transport, so the reconstruction is a close proxy, not an identity.

Read-only with respect to the repo: writes only under analysis/data/.
"""
from __future__ import annotations

import json
import sqlite3
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DATA = Path(__file__).resolve().parent / "data"

# Reconstruction constants mirror helix_adapter/topic_rate_monitor.py defaults.
RATE_WINDOW_SEC = 5.0
PUBLISH_PERIOD_SEC = 0.5

RECON_TOPICS = [
    "/utlidar/imu",
    "/utlidar/robot_pose",
    "/utlidar/cloud",
    "/gnss",
    "/multiplestate",
]


def _metric_name(topic: str) -> str:
    """Mirror topic_rate_monitor._publish naming."""
    return "rate_hz/" + topic.replace("/", "_").lstrip("_")


def extract_recorded_metrics(bag_dir: Path, out: Path) -> dict:
    """Pull the recorded /helix/metrics stream out of a rosbag2 sqlite bag."""
    from rosbags.typesys import Stores, get_typestore

    ts_store = get_typestore(Stores.ROS2_HUMBLE)
    samples: list[tuple[int, str, float]] = []
    for db in sorted(bag_dir.glob("*.db3")):
        con = sqlite3.connect(str(db))
        row = con.execute(
            "SELECT id, type FROM topics WHERE name='/helix/metrics'"
        ).fetchone()
        if row is None:
            con.close()
            continue
        tid, ttype = row
        for stamp, blob in con.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (tid,),
        ):
            msg = ts_store.deserialize_cdr(bytes(blob), ttype)
            if not len(msg.layout.dim):
                continue
            label = str(msg.layout.dim[0].label)
            for value in msg.data:
                samples.append((int(stamp), label, float(value)))
        con.close()

    samples.sort(key=lambda s: s[0])
    if not samples:
        raise SystemExit(f"no /helix/metrics in {bag_dir}")
    t0 = samples[0][0]
    payload = {
        "source": str(bag_dir.relative_to(REPO)),
        "kind": "RECORDED-METRICS",
        "duration_s": round((samples[-1][0] - t0) / 1e9, 3),
        "n_samples": len(samples),
        "samples": [
            {"t": round((s[0] - t0) / 1e9, 6), "metric": s[1], "value": s[2]}
            for s in samples
        ],
    }
    out.write_text(json.dumps(payload))
    return payload


def _arrival_times(bag_dir: Path, topic: str) -> list[float]:
    """Receive timestamps (seconds, bag-relative) for one topic."""
    stamps: list[int] = []
    for db in sorted(bag_dir.glob("*.db3")):
        con = sqlite3.connect(str(db))
        row = con.execute("SELECT id FROM topics WHERE name=?", (topic,)).fetchone()
        if row is not None:
            stamps.extend(
                r[0]
                for r in con.execute(
                    "SELECT timestamp FROM messages WHERE topic_id=?", (row[0],)
                )
            )
        con.close()
    stamps.sort()
    return stamps


def reconstruct_rate_metrics(bag_dir: Path, out: Path) -> dict:
    """Replay RateWindow + the 0.5 s publish timer over real arrival times."""
    import math

    per_topic = {t: _arrival_times(bag_dir, t) for t in RECON_TOPICS}
    per_topic = {t: v for t, v in per_topic.items() if v}
    if not per_topic:
        raise SystemExit(f"no reconstruction topics in {bag_dir}")

    all_ns = sorted(x for v in per_topic.values() for x in v)
    t0, t1 = all_ns[0], all_ns[-1]
    dur = (t1 - t0) / 1e9

    # Publish grid, same cadence as the shipped node's timer.
    n_pub = int(dur / PUBLISH_PERIOD_SEC)
    samples = []
    for topic, stamps in per_topic.items():
        rel = [(s - t0) / 1e9 for s in stamps]
        name = _metric_name(topic)
        idx = 0
        lo = 0
        for k in range(1, n_pub + 1):
            now = k * PUBLISH_PERIOD_SEC
            cutoff = now - RATE_WINDOW_SEC
            while idx < len(rel) and rel[idx] <= now:
                idx += 1
            # RateWindow._evict drops strictly-older-than-cutoff entries.
            while lo < idx and rel[lo] < cutoff:
                lo += 1
            n = idx - lo
            if n == 0:
                value = math.nan
            elif n == 1:
                value = 0.0
            else:
                span = rel[idx - 1] - rel[lo]
                value = 0.0 if span < 1e-6 else (n - 1) / span
            samples.append((now, name, value))

    samples.sort(key=lambda s: (s[0], s[1]))
    payload = {
        "source": str(bag_dir.relative_to(REPO)),
        "kind": "RECONSTRUCTED-RATE",
        "rate_window_sec": RATE_WINDOW_SEC,
        "publish_period_sec": PUBLISH_PERIOD_SEC,
        "duration_s": round(dur, 3),
        "n_samples": len(samples),
        "topic_msg_counts": {t: len(v) for t, v in per_topic.items()},
        "samples": [
            {
                "t": round(s[0], 6),
                "metric": s[1],
                "value": (None if s[2] != s[2] else s[2]),
            }
            for s in samples
        ],
    }
    out.write_text(json.dumps(payload))
    return payload


SESSION8_BAG = Path.home() / "workspace" / "helix-demo-build" / "bag"


def reconstruct_session8_sportmodestate(out: Path) -> dict:
    """Rebuild a rate_hz metric stream from Session 8's own raw arrival times.

    The Session 8 bag recorded /sportmodestate (130k messages, ~296 Hz) but not
    /helix/metrics, so the detector's actual Session 8 input is unrecoverable.
    Replaying RateWindow over /sportmodestate's real arrival timestamps gives a
    metric stream carrying Session 8's own DDS jitter, on the same host, in the
    same run that produced the 30 false alarms.

    ASSUMPTION: /sportmodestate was not one of the six monitored topics, so this
    stream is a same-run noise sample, not a replay of a metric the detector
    actually consumed. It is used as an extra real false-alarm baseline and is
    excluded from the STOP_AND_HOLD escalation count (R1 matches only
    rate_hz/utlidar*).
    """
    import math

    db = SESSION8_BAG / "post_fix_demo_0.db3"
    con = sqlite3.connect(str(db))
    tid = con.execute(
        "SELECT id FROM topics WHERE name='/sportmodestate'"
    ).fetchone()[0]
    stamps = [
        r[0]
        for r in con.execute(
            "SELECT timestamp FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)
        )
    ]
    con.close()
    t0 = stamps[0]
    rel = [(s - t0) / 1e9 for s in stamps]
    dur = rel[-1]

    samples = []
    idx = lo = 0
    for k in range(1, int(dur / PUBLISH_PERIOD_SEC) + 1):
        now = k * PUBLISH_PERIOD_SEC
        cutoff = now - RATE_WINDOW_SEC
        while idx < len(rel) and rel[idx] <= now:
            idx += 1
        while lo < idx and rel[lo] < cutoff:
            lo += 1
        n = idx - lo
        if n == 0:
            value = math.nan
        elif n == 1:
            value = 0.0
        else:
            span = rel[idx - 1] - rel[lo]
            value = 0.0 if span < 1e-6 else (n - 1) / span
        samples.append((now, "rate_hz/sportmodestate", value))

    payload = {
        "source": "helix-demo-build/bag/post_fix_demo_0.db3 (Session 8)",
        "kind": "RECONSTRUCTED-RATE",
        "rate_window_sec": RATE_WINDOW_SEC,
        "publish_period_sec": PUBLISH_PERIOD_SEC,
        "duration_s": round(dur, 3),
        "n_samples": len(samples),
        "topic_msg_counts": {"/sportmodestate": len(stamps)},
        "samples": [
            {"t": round(s[0], 6), "metric": s[1],
             "value": (None if s[2] != s[2] else s[2])}
            for s in samples
        ],
    }
    out.write_text(json.dumps(payload))
    return payload


def main() -> int:
    DATA.mkdir(parents=True, exist_ok=True)
    bags = REPO / "hardware_eval_20260406" / "bags"

    for session in ("helix_adapter_session", "helix_faults_session"):
        p = extract_recorded_metrics(bags / session, DATA / f"metrics_{session}.json")
        print(
            f"[RECORDED-METRICS] {session}: {p['n_samples']} samples over "
            f"{p['duration_s']}s"
        )

    p = reconstruct_rate_metrics(
        bags / "extended_5min", DATA / "recon_extended_5min.json"
    )
    print(
        f"[RECONSTRUCTED-RATE] extended_5min: {p['n_samples']} samples over "
        f"{p['duration_s']}s from {p['topic_msg_counts']}"
    )

    if (SESSION8_BAG / "post_fix_demo_0.db3").exists():
        p = reconstruct_session8_sportmodestate(DATA / "recon_session8_sportmodestate.json")
        print(
            f"[RECONSTRUCTED-RATE] session8_sportmodestate: {p['n_samples']} samples "
            f"over {p['duration_s']}s"
        )
    else:
        print("[skip] Session 8 bag not present")
    return 0


if __name__ == "__main__":
    sys.exit(main())
