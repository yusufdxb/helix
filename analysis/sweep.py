#!/usr/bin/env python3
"""Sweep HELIX detector parameters: measured false alarms vs detection latency.

FALSE-ALARM AXIS  = REAL. Recorded/reconstructed telemetry from a GO2 standing
                    idle. Every fault counted here is a fault the detector
                    would have raised on a healthy robot.

LATENCY AXIS      = SYNTHETIC ONSET ON A REAL BASELINE. The detector is warmed
                    up on the real idle stream, then from an onset time the
                    real values are replaced by a fault model (topic death,
                    step rate drop, ramp rate drop). Latency is the delay from
                    onset to the first emitted fault on that metric. These
                    numbers are labelled SYNTHETIC everywhere they appear.

Writes analysis/results/sweep.json.
"""
from __future__ import annotations

import copy
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Dict, List, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))

from detector_replay import (  # noqa: E402
    STOP_METRIC_PREFIX,
    DetectorConfig,
    ReplayDetector,
)

HERE = Path(__file__).resolve().parent
DATA = HERE / "data"
RESULTS = HERE / "results"

# Idle baselines. helix_faults_session is deliberately EXCLUDED: it is the
# fault-injection capture, so counting its faults as false alarms would be
# wrong. Only captures of a healthy, standing robot belong on this axis.
BASELINES = [
    ("extended_5min", "recon_extended_5min.json", "RECONSTRUCTED-RATE"),
    ("adapter_session", "metrics_helix_adapter_session.json", "RECORDED-METRICS"),
    ("session8_sportmodestate", "recon_session8_sportmodestate.json", "RECONSTRUCTED-RATE"),
]

# Fault injection (SYNTHETIC).
WARMUP_S = 60.0          # real samples before any onset, so the window is full
ONSET_SPACING_S = 30.0
HORIZON_S = 30.0         # a fault not caught within 30 s counts as a miss
RAMP_S = 30.0

# Fault magnitudes are expressed in units of the metric's OWN measured idle
# sigma, because that is the unit the z-score detector actually works in. A
# "10 percent rate drop" means nothing to the detector; "5 sigma" does.
# step_drop_50 is kept because a 50 percent rate collapse is the operationally
# recognisable failure a reader will ask about.
FAULT_MODELS = [
    "stale",
    "step_sigma_2",
    "step_sigma_3",
    "step_sigma_5",
    "step_sigma_10",
    "step_drop_50",
    "ramp_sigma_5",
]

# Sweep grid.
STATISTICS = ["zscore", "mad", "ewma"]
THRESHOLDS = [2.5, 3.0, 3.5, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 16.0]
CONSECUTIVE = [1, 3, 5, 8]
MIN_DURATION = [0.0, 2.0, 5.0, 10.0]
WINDOWS = [20, 30, 60, 120]


def load_streams() -> Dict[str, Dict[str, List[Tuple[float, float]]]]:
    """source -> metric -> [(t, value)] with NaN restored for stale samples."""
    out: Dict[str, Dict[str, List[Tuple[float, float]]]] = {}
    for name, fname, _kind in BASELINES:
        path = DATA / fname
        if not path.exists():
            print(f"[warn] missing {path}, skipping {name}")
            continue
        payload = json.loads(path.read_text())
        per_metric: Dict[str, List[Tuple[float, float]]] = {}
        for s in payload["samples"]:
            value = math.nan if s["value"] is None else float(s["value"])
            per_metric.setdefault(s["metric"], []).append((float(s["t"]), value))
        for series in per_metric.values():
            series.sort()
        out[name] = per_metric
    return out


def exposure_seconds(streams) -> Tuple[float, float]:
    """(total idle seconds, idle seconds on sources carrying utlidar metrics)."""
    total = 0.0
    stop_relevant = 0.0
    for _src, metrics in streams.items():
        span = max(series[-1][0] - series[0][0] for series in metrics.values())
        total += span
        if any(m.startswith(STOP_METRIC_PREFIX) for m in metrics):
            stop_relevant += span
    return total, stop_relevant


# ── false-alarm axis (REAL) ──────────────────────────────────────────────────


def false_alarms(streams, config: DetectorConfig) -> dict:
    """Count faults the config would raise on real idle data."""
    total = 0
    stop_cmds = 0
    per_metric: Dict[str, int] = {}
    per_source: Dict[str, int] = {}
    for _src, metrics in streams.items():
        per_source.setdefault(_src, 0)
        for metric, series in metrics.items():
            det = ReplayDetector(config=config)
            for t, v in series:
                det.process(t, metric, v)
            n = len(det.emitted)
            if n:
                per_metric[metric] = per_metric.get(metric, 0) + n
            total += n
            per_source[_src] += n
            if metric.startswith(STOP_METRIC_PREFIX):
                stop_cmds += n
    return {
        "faults": total,
        "stop_cmds": stop_cmds,
        "per_metric": per_metric,
        "per_source": per_source,
    }


# ── latency axis (SYNTHETIC onset on REAL baseline) ──────────────────────────


def _inject(value: float, dt: float, model: str, sigma: float) -> float:
    """Return the faulted value ``dt`` seconds after onset.

    ``sigma`` is the metric's measured idle standard deviation over the real
    baseline series, so sigma-scaled faults are comparable across metrics that
    run at 1 Hz and at 250 Hz.
    """
    if model == "stale":
        return math.nan
    if model.startswith("step_sigma_"):
        k = float(model.rsplit("_", 1)[1])
        return value - k * sigma
    if model.startswith("ramp_sigma_"):
        k = float(model.rsplit("_", 1)[1])
        return value - k * sigma * min(1.0, dt / RAMP_S)
    if model.startswith("step_drop_"):
        frac = int(model.rsplit("_", 1)[1]) / 100.0
        return value * (1.0 - frac)
    raise ValueError(model)


def latency_trials(streams, config: DetectorConfig) -> dict:
    """Warm up on real data, then inject. Returns per-model latency stats."""
    per_model: Dict[str, List[float]] = {m: [] for m in FAULT_MODELS}
    per_model_trials: Dict[str, int] = {m: 0 for m in FAULT_MODELS}
    misses: Dict[str, Dict[str, int]] = {m: {} for m in FAULT_MODELS}

    for _src, metrics in streams.items():
        for metric, series in metrics.items():
            if not metric.startswith("rate_hz/"):
                continue
            finite = [v for _t, v in series if not math.isnan(v)]
            sigma = statistics.pstdev(finite) if len(finite) > 1 else 0.0
            t_end = series[-1][0]
            onsets = []
            t = WARMUP_S
            while t + HORIZON_S <= t_end:
                onsets.append(t)
                t += ONSET_SPACING_S
            if not onsets:
                continue

            # One warm pass over the real stream, snapshotting detector state
            # at every onset. Each injection then continues from its snapshot,
            # so every trial sees a full, real warmup without re-running it.
            det = ReplayDetector(config=config)
            snaps: Dict[float, ReplayDetector] = {}
            next_onset = 0
            for t, v in series:
                while next_onset < len(onsets) and t > onsets[next_onset]:
                    snaps[onsets[next_onset]] = copy.deepcopy(det)
                    next_onset += 1
                det.process(t, metric, v)
            while next_onset < len(onsets):
                snaps[onsets[next_onset]] = copy.deepcopy(det)
                next_onset += 1

            for onset in onsets:
                tail = [(t, v) for t, v in series if onset < t <= onset + HORIZON_S]
                if not tail:
                    continue
                for model in FAULT_MODELS:
                    trial = copy.deepcopy(snaps[onset])
                    trial.emitted.clear()
                    per_model_trials[model] += 1
                    hit = None
                    for t, v in tail:
                        trial.process(t, metric, _inject(v, t - onset, model, sigma))
                        if trial.emitted:
                            hit = trial.emitted[0]["t"] - onset
                            break
                    if hit is not None:
                        per_model[model].append(hit)
                    else:
                        misses[model][metric] = misses[model].get(metric, 0) + 1

    out = {}
    for model in FAULT_MODELS:
        lats = per_model[model]
        n = per_model_trials[model]
        out[model] = {
            "trials": n,
            "detected": len(lats),
            "detect_rate": (len(lats) / n) if n else None,
            "median_latency_s": (statistics.median(lats) if lats else None),
            "p90_latency_s": (
                sorted(lats)[min(len(lats) - 1, int(0.9 * len(lats)))] if lats else None
            ),
            "max_latency_s": (max(lats) if lats else None),
            "missed_by_metric": misses[model],
        }
    return out


# ── driver ───────────────────────────────────────────────────────────────────


def grid() -> List[DetectorConfig]:
    seen = set()
    configs = []
    for stat in STATISTICS:
        for thr in THRESHOLDS:
            for k in CONSECUTIVE:
                for dur in MIN_DURATION:
                    cfg = DetectorConfig(stat, thr, k, 60, dur)
                    if cfg.label() not in seen:
                        seen.add(cfg.label())
                        configs.append(cfg)
            for w in WINDOWS:
                cfg = DetectorConfig(stat, thr, 3, w, 2.0)
                if cfg.label() not in seen:
                    seen.add(cfg.label())
                    configs.append(cfg)
    return configs


def main() -> int:
    RESULTS.mkdir(parents=True, exist_ok=True)
    streams = load_streams()
    total_s, stop_s = exposure_seconds(streams)
    print(f"real idle exposure: {total_s:.1f}s total, {stop_s:.1f}s on utlidar-bearing sources")

    configs = grid()
    print(f"sweeping {len(configs)} configs")
    rows = []
    for i, cfg in enumerate(configs, 1):
        fa = false_alarms(streams, cfg)
        lat = latency_trials(streams, cfg)
        rows.append(
            {
                "statistic": cfg.statistic,
                "threshold": cfg.threshold,
                "consecutive_trigger": cfg.consecutive_trigger,
                "window_size": cfg.window_size,
                "min_anomaly_duration_s": cfg.min_anomaly_duration_s,
                "label": cfg.label(),
                "real_false_faults": fa["faults"],
                "real_false_stops": fa["stop_cmds"],
                "real_false_faults_per_hour": fa["faults"] / (total_s / 3600.0),
                "real_false_stops_per_hour": fa["stop_cmds"] / (stop_s / 3600.0),
                "real_per_metric": fa["per_metric"],
                "real_per_source": fa["per_source"],
                "synthetic_latency": lat,
            }
        )
        if i % 25 == 0:
            print(f"  {i}/{len(configs)}")

    payload = {
        "provenance": {
            "false_alarm_axis": "REAL idle GO2 telemetry",
            "latency_axis": "SYNTHETIC fault onset injected into REAL idle baseline",
            "baselines": [
                {"name": n, "file": f, "kind": k} for n, f, k in BASELINES
            ],
            "excluded": "helix_faults_session (fault-injection capture, not idle)",
            "idle_exposure_s": round(total_s, 3),
            "idle_exposure_s_utlidar_sources": round(stop_s, 3),
            "warmup_s": WARMUP_S,
            "onset_spacing_s": ONSET_SPACING_S,
            "horizon_s": HORIZON_S,
            "fault_models": FAULT_MODELS,
        },
        "rows": rows,
    }
    (RESULTS / "sweep.json").write_text(json.dumps(payload, indent=1))
    print(f"wrote {RESULTS / 'sweep.json'}  ({len(rows)} rows)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
