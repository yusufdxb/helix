#!/usr/bin/env python3
"""Held-out evaluation of the HELIX anomaly detector (native AnomalyCore).

Every number this script produces comes from helix_detector_eval, which runs
the same AnomalyCore the C++ node runs. Nothing here re-implements detection.

Data
  Real GO2 /helix/metrics streams (the detector's literal input), split by
  capture date so configuration choice never sees the test data:

    DEV       April 2026 captures (analysis/data/*.json): 901 s, robot idle,
              plus every capture passed with --dev
    HELD-OUT  every capture passed with --heldout

A capture that has been used to judge a configuration is DEV from then on.
History (docs/DETECTOR_EVAL.md): round 1 selected on April only and was
scored on 2026-10-06 standing and 2026-10-17 lying; the second exposed false
alarms, so round 2 selects on April + both and is scored on a new capture.

False alarms (REAL): faults the detector raises on an unmodified stream.
Detection (SYNTHETIC ONSET ON A REAL STREAM): from an onset, the real samples
of one stop-relevant rate metric are transformed the way the shipped rate
estimator ((n-1)/span over a 5 s window, 0.5 s publish grid) would report
the fault, keeping the real noise:

    collapse_<p>  arrival rate drops to p % at onset; the estimate blends
                  linearly to p % over the 5 s window
    death         no arrivals after onset. The legacy monitor keeps a near
                  normal estimate until the window empties (5 s), reports
                  0.0 once, then NaN; with stale_after_s S the native monitor
                  reports NaN from onset + S.
    drift_<p>     arrival rate declines linearly to p % over 60 s

Recovery (SYNTHETIC): the same faults lasting 20 s, then the real stream
resumes; measures clear latency and false clears (a clear record while the
fault is still present).

Usage
  analysis/detector_eval.py --eval-bin <helix_detector_eval> \\
      [--heldout <bag_dir_or_json> ...] --out analysis/results/detector_eval.json
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
import sqlite3
import statistics
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

HERE = Path(__file__).resolve().parent
DATA = HERE / "data"
STOP_PREFIX = "rate_hz/utlidar"
WINDOW_S = 5.0          # topic_rate_monitor window_sec (shipped)
PERIOD_S = 0.5          # topic_rate_monitor publish_period_sec (shipped)
WARMUP_S = 70.0         # real samples before an onset (fills a 120-sample window)
HORIZON_S = 30.0        # a fault not caught within 30 s is a miss
SPACING_S = 20.0        # onsets per metric are this far apart
RECOVER_AFTER_S = 20.0  # recovery runs: fault duration

Sample = Tuple[float, str, float]   # (t, metric, value) value may be NaN

DEV_SOURCES = [
    ("apr06_extended_5min", DATA / "recon_extended_5min.json"),
    ("apr06_adapter_session", DATA / "metrics_helix_adapter_session.json"),
    ("apr23_session8_sportmodestate", DATA / "recon_session8_sportmodestate.json"),
]

FAULT_MODELS = ["collapse_50", "collapse_75", "death", "death_s1", "drift_70"]
# Slow drift needs long captures: 30 % over 300 s, scored over 330 s. Reported
# separately; DEV captures are too short to contain one, so it is not a
# selection criterion.
SLOW_MODELS = ["slowdrift_70"]
HORIZONS = {"slowdrift_70": 330.0}


# -- loading ----------------------------------------------------------------

def load_json(path: Path) -> List[Sample]:
    d = json.loads(path.read_text())
    out = []
    for s in d["samples"]:
        v = s["value"]
        out.append((float(s["t"]), s["metric"], math.nan if v is None else float(v)))
    return sorted(out, key=lambda x: x[0])


def load_bag(path: Path) -> List[Sample]:
    """/helix/metrics from a rosbag2 sqlite3 directory (needs ROS sourced)."""
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import Float64MultiArray
    db = next(path.glob("*.db3")) if path.is_dir() else path
    con = sqlite3.connect(str(db))
    tid = con.execute("SELECT id FROM topics WHERE name='/helix/metrics'").fetchone()
    if tid is None:
        raise RuntimeError(f"{db}: no /helix/metrics topic")
    rows = con.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (tid[0],)).fetchall()
    con.close()
    out = []
    t0 = rows[0][0] if rows else 0
    for ts, blob in rows:
        m = deserialize_message(bytes(blob), Float64MultiArray)
        if not m.layout.dim or not m.layout.dim[0].label:
            continue
        for v in m.data:
            out.append(((ts - t0) / 1e9, m.layout.dim[0].label, float(v)))
    return out


def load(path: Path) -> List[Sample]:
    return load_json(path) if path.suffix == ".json" else load_bag(path)


def by_metric(samples: Iterable[Sample]) -> Dict[str, List[Tuple[float, float]]]:
    out: Dict[str, List[Tuple[float, float]]] = {}
    for t, m, v in samples:
        out.setdefault(m, []).append((t, v))
    return out


def stop_metrics(streams: Dict[str, List[Tuple[float, float]]]) -> List[str]:
    """Stop-relevant rate metrics that are live (mostly finite) in this capture."""
    out = []
    for m, s in sorted(streams.items()):
        if not m.startswith(STOP_PREFIX):
            continue
        finite = [v for _, v in s if math.isfinite(v)]
        if len(finite) > 0.9 * len(s) and statistics.fmean(finite) > 1.0:
            out.append(m)
    return out


# -- detector ---------------------------------------------------------------

@dataclass(frozen=True)
class Cfg:
    name: str
    zscore_threshold: float = 4.0
    window_size: int = 60
    consecutive_trigger: int = 3
    min_anomaly_duration_s: float = 2.0
    emit_cooldown_s: float = 1.0
    freeze_baseline: int = 0
    std_floor_rel: float = 0.0
    duration_tolerance_s: float = 0.0
    clear_samples: int = 4
    clear_zscore: float = 0.0
    anchor_tolerance_rel: float = 0.0

    def argv(self) -> List[str]:
        keys = ("zscore_threshold", "window_size", "consecutive_trigger",
                "min_anomaly_duration_s", "emit_cooldown_s", "freeze_baseline",
                "std_floor_rel", "duration_tolerance_s", "clear_samples", "clear_zscore",
                "anchor_tolerance_rel")
        return [f"{k}={getattr(self, k)}" for k in keys]


def run_detector(eval_bin: str, cfg: Cfg, samples: Iterable[Sample]) -> List[dict]:
    lines = "".join(
        f"{t:.6f},{m},{'nan' if not math.isfinite(v) else repr(v)}\n" for t, m, v in samples)
    p = subprocess.run([eval_bin, *cfg.argv()], input=lines, capture_output=True,
                       text=True, check=True)
    return [json.loads(x) for x in p.stdout.splitlines() if x.strip()]


# -- fault synthesis --------------------------------------------------------

def _transform(model: str, d: float, v: float) -> float:
    """Value at time d >= 0 after onset, given the real value v."""
    if model.startswith("collapse_"):
        p = int(model.split("_")[1]) / 100.0
        return v * (1.0 - (1.0 - p) * min(d, WINDOW_S) / WINDOW_S)
    if model.startswith("drift_"):
        p = int(model.split("_")[1]) / 100.0
        return v * (1.0 - (1.0 - p) * min(d, 60.0) / 60.0)
    if model.startswith("slowdrift_"):
        p = int(model.split("_")[1]) / 100.0
        return v * (1.0 - (1.0 - p) * min(d, 300.0) / 300.0)
    if model == "death":
        # Legacy RateWindow: old arrivals keep (n-1)/span near nominal until
        # the window empties; one 0.0 sample (n == 1), then NaN.
        if d < WINDOW_S - PERIOD_S:
            return v
        if d < WINDOW_S:
            return 0.0
        return math.nan
    if model == "death_s1":
        return v if d < 1.0 else math.nan
    raise ValueError(model)


def inject(stream: List[Tuple[float, float]], onset: float, model: str,
           until: Optional[float] = None) -> List[Tuple[float, float]]:
    out = []
    for t, v in stream:
        if t < onset or (until is not None and t >= until) or not math.isfinite(v):
            out.append((t, v))
        else:
            out.append((t, _transform(model, t - onset, v)))
    return out


def injections(streams, metrics: List[str], horizon: float = HORIZON_S) -> List[Tuple[str, float]]:
    out = []
    for m in metrics:
        s = streams[m]
        t_first, t_last = s[0][0], s[-1][0]
        onset = t_first + WARMUP_S
        while onset + horizon <= t_last:
            out.append((m, onset))
            onset += SPACING_S
    return out


# -- scoring ----------------------------------------------------------------

def false_alarms(eval_bin: str, cfg: Cfg, samples: List[Sample], duration_s: float) -> dict:
    ev = run_detector(eval_bin, cfg, samples)
    faults = [e for e in ev if e["kind"] == "fault"]
    stop = [e for e in faults if e["metric"].startswith(STOP_PREFIX)]
    # Episodes: faults on the same metric within 10 s are one episode.
    def episodes(fs):
        n, last = 0, {}
        for e in sorted(fs, key=lambda e: e["t"]):
            if e["metric"] not in last or e["t"] - last[e["metric"]] > 10.0:
                n += 1
            last[e["metric"]] = e["t"]
        return n
    hours = duration_s / 3600.0
    return {
        "duration_s": round(duration_s, 1),
        "stop_fault_episodes": episodes(stop),
        "stop_fault_episodes_per_h": round(episodes(stop) / hours, 2),
        "all_fault_episodes": episodes(faults),
        "all_fault_episodes_per_h": round(episodes(faults) / hours, 2),
        "by_metric": {m: sum(1 for e in faults if e["metric"] == m)
                      for m in sorted({e["metric"] for e in faults})},
    }


def detection(eval_bin: str, cfg: Cfg, streams, metrics, model: str,
              recover: bool = False) -> dict:
    """Batch every injection as its own renamed metric in one detector run."""
    horizon = HORIZONS.get(model, HORIZON_S)
    batch: List[Sample] = []
    meta = {}
    for k, (m, onset) in enumerate(injections(streams, metrics, horizon)):
        name = f"{m}#{k}"
        until = onset + RECOVER_AFTER_S if recover else None
        lo, hi = onset - WARMUP_S, onset + (RECOVER_AFTER_S + 30.0 if recover else horizon)
        seg = [(t, v) for t, v in inject(streams[m], onset, model, until) if lo <= t <= hi]
        batch.extend((t, name, v) for t, v in seg)
        meta[name] = (onset, until)
    ev = run_detector(eval_bin, cfg, batch)
    first_fault: Dict[str, float] = {}
    clears: Dict[str, List[float]] = {}
    for e in ev:
        if e["kind"] == "fault":
            first_fault.setdefault(e["metric"], e["t"])
        else:
            clears.setdefault(e["metric"], []).append(e["t"])
    lat, n_det, false_clear, clear_lat, pre_onset = [], 0, 0, [], 0
    for name, (onset, until) in meta.items():
        ff = first_fault.get(name)
        if ff is not None and ff < onset:
            pre_onset += 1     # a real-data fault inside the warmup: not a detection
            ff = None
        if ff is not None and ff - onset <= horizon:
            n_det += 1
            lat.append(ff - onset)
            end = until if until is not None else onset + horizon
            if any(ff < c < end for c in clears.get(name, [])):
                false_clear += 1
            if until is not None:
                after = [c - until for c in clears.get(name, []) if c >= until]
                if after:
                    clear_lat.append(after[0])
    n = len(meta)
    out = {
        "n": n, "detected": n_det, "tpr": round(n_det / n, 4) if n else None,
        "latency_median_s": round(statistics.median(lat), 3) if lat else None,
        "latency_p90_s": round(sorted(lat)[int(0.9 * (len(lat) - 1))], 3) if lat else None,
        "false_clears_during_fault": false_clear,
        "faults_before_onset": pre_onset,
    }
    if recover:
        out["cleared"] = len(clear_lat)
        out["clear_latency_median_s"] = round(statistics.median(clear_lat), 3) if clear_lat else None
    return out


def evaluate(eval_bin: str, cfg: Cfg, sources: List[Tuple[str, List[Sample]]]) -> dict:
    res = {"config": cfg.__dict__, "false_alarms": {}, "detection": {}, "recovery": {}}
    pooled: Dict[str, List[dict]] = {}
    for name, samples in sources:
        duration = max(t for t, _, _ in samples) - min(t for t, _, _ in samples)
        res["false_alarms"][name] = false_alarms(eval_bin, cfg, samples, duration)
        streams = by_metric(samples)
        mets = stop_metrics(streams)
        for model in FAULT_MODELS + SLOW_MODELS:
            d = detection(eval_bin, cfg, streams, mets, model)
            if d["n"]:
                pooled.setdefault(model, []).append(d)
        for model in ("collapse_50", "death_s1"):
            d = detection(eval_bin, cfg, streams, mets, model, recover=True)
            pooled.setdefault("recover_" + model, []).append(d)
    for model, ds in pooled.items():
        n = sum(d["n"] for d in ds)
        det = sum(d["detected"] for d in ds)
        agg = {"n": n, "detected": det, "tpr": round(det / n, 4) if n else None,
               "false_clears_during_fault": sum(d["false_clears_during_fault"] for d in ds),
               "faults_before_onset": sum(d["faults_before_onset"] for d in ds),
               "latency_median_s_by_source": [d["latency_median_s"] for d in ds]}
        if model.startswith("recover_"):
            agg["cleared"] = sum(d.get("cleared", 0) for d in ds)
            agg["clear_latency_median_s_by_source"] = [d.get("clear_latency_median_s") for d in ds]
            res["recovery"][model[len("recover_"):]] = agg
        else:
            res["detection"][model] = agg
    tot = sum(v["duration_s"] for v in res["false_alarms"].values())
    eps = sum(v["stop_fault_episodes"] for v in res["false_alarms"].values())
    res["stop_false_alarms_per_h"] = round(eps / (tot / 3600.0), 2)
    res["stop_false_alarm_episodes"] = eps
    res["idle_hours"] = round(tot / 3600.0, 4)
    return res


# -- configurations ---------------------------------------------------------

def candidate_grid() -> List[Cfg]:
    out = [
        Cfg("shipped_z4_w60"),
        Cfg("proposed_z3.5_w120", zscore_threshold=3.5, window_size=120),
    ]
    for z, w, floor, dur, anchor in itertools.product(
            (4.0, 5.0, 6.0), (60, 120), (0.01, 0.02, 0.05), (1.0, 2.0), (0.0, 0.1, 0.2)):
        out.append(Cfg(f"freeze_z{z}_w{w}_f{floor}_d{dur}_a{anchor}", zscore_threshold=z,
                       window_size=w, freeze_baseline=1, std_floor_rel=floor,
                       min_anomaly_duration_s=dur, duration_tolerance_s=0.25,
                       anchor_tolerance_rel=anchor))
    return out


def select(results: List[dict]) -> Optional[dict]:
    """Pick on DEV only.

    Hard constraints: zero stop-metric false alarms and zero false clears
    (a clear while the fault is still present would release a hold early).
    Then highest mean TPR over the fault models, then lowest mean latency,
    then fewest log-only false alarms.
    """
    def clean(r):
        if r["stop_false_alarms_per_h"] > 0.0:
            return False
        fc = sum(r["detection"][m]["false_clears_during_fault"]
                 for m in FAULT_MODELS + SLOW_MODELS if m in r["detection"])
        fc += sum(v["false_clears_during_fault"] for v in r["recovery"].values())
        return fc == 0
    def score(r):
        # Slow drift counts once DEV has captures long enough to score it.
        models = [m for m in FAULT_MODELS + SLOW_MODELS
                  if m in r["detection"] and r["detection"][m]["n"]]
        tprs = [r["detection"][m]["tpr"] or 0.0 for m in models]
        lats = [x for m in FAULT_MODELS for x in r["detection"][m]["latency_median_s_by_source"]
                if x is not None]
        all_fa = sum(v["all_fault_episodes"] for v in r["false_alarms"].values())
        return (round(statistics.fmean(tprs), 4),
                -(statistics.fmean(lats) if lats else 99.0), -all_fa)
    ok = [r for r in results if clean(r)]
    return max(ok, key=score) if ok else None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--eval-bin", required=True)
    ap.add_argument("--dev", action="append", default=[],
                    help="extra DEV capture: bag dir or JSON; name=path to label it")
    ap.add_argument("--heldout", action="append", default=[],
                    help="bag dir or JSON; name=path to label it")
    ap.add_argument("--out", default=str(HERE / "results" / "detector_eval.json"))
    args = ap.parse_args()

    def labelled(spec: str):
        name, _, path = spec.partition("=")
        if not path:
            name, path = Path(spec).name, spec
        return name, load(Path(path))

    dev = [(n, load(p)) for n, p in DEV_SOURCES] + [labelled(d) for d in args.dev]
    held = [labelled(h) for h in args.heldout]

    grid = candidate_grid()
    dev_results = [evaluate(args.eval_bin, c, dev) for c in grid]
    chosen = select(dev_results)
    report = {
        "protocol": "configs scored on DEV only; the chosen config and the two "
                    "baselines are then scored once on HELD-OUT",
        "dev_sources": [n for n, _ in dev],
        "heldout_sources": [n for n, _ in held],
        "dev": {r["config"]["name"]: r for r in dev_results},
        "chosen_on_dev": chosen["config"]["name"] if chosen else None,
        "heldout": {},
    }
    if held:
        names = {"shipped_z4_w60", "proposed_z3.5_w120"}
        if chosen:
            base = chosen["config"]["name"]
            names.add(base)
            # The chosen config with the anchored band, to measure what the
            # band adds on slow drift (DEV cannot score that).
            stem = base.rsplit("_a", 1)[0]
            names.update({stem + "_a0.1", stem + "_a0.2"})
        for c in grid:
            if c.name in names:
                report["heldout"][c.name] = evaluate(args.eval_bin, c, held)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(report, indent=1))
    def line(tag, r):
        det = r["detection"]
        print(f"{tag:<44} FA/h={r['stop_false_alarms_per_h']:<6} " + " ".join(
            f"{m}={det[m]['detected']}/{det[m]['n']}" for m in FAULT_MODELS + SLOW_MODELS
            if m in det))
    print("DEV")
    for r in dev_results[:2]:
        line(r["config"]["name"], r)
    if chosen:
        line("chosen: " + chosen["config"]["name"], chosen)
    for name, r in report["heldout"].items():
        line("HELD-OUT " + name, r)


if __name__ == "__main__":
    main()
