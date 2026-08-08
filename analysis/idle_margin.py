#!/usr/bin/env python3
"""How close does real idle noise actually get to firing? (REAL data only.)

A false-alarm count of zero over 15 minutes is a weak statement: with that much
exposure the 95 percent Poisson upper bound is still about 12 events per hour.
The stronger, exposure-independent measurement is the MARGIN: the distribution
of the detector's own score on healthy idle telemetry, and the longest run of
consecutive above-threshold samples that idle noise ever produced.

A threshold is safe when it sits well above the idle score distribution AND
when idle noise never strings together ``consecutive_trigger`` violations.

Writes analysis/results/idle_margin.json.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import Dict, List

sys.path.insert(0, str(Path(__file__).resolve().parent))

from detector_replay import STATISTICS  # noqa: E402
from sweep import load_streams  # noqa: E402

RESULTS = Path(__file__).resolve().parent / "results"

WINDOWS = [20, 30, 60, 120, 180]
STATS = ["zscore", "mad", "ewma"]
THRESHOLDS = [2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0, 8.0]


def _ewma_state_update(state: dict, value: float, window_size: int) -> None:
    alpha = 2.0 / (window_size + 1.0)
    if "mu" not in state:
        state["mu"] = value
        state["var"] = 0.0
        state["n"] = 1
        return
    delta = value - state["mu"]
    state["mu"] += alpha * delta
    state["var"] = (1.0 - alpha) * (state["var"] + alpha * delta * delta)
    state["n"] += 1
    if state["n"] < 3:
        state["var"] = 0.0


def idle_scores(streams, statistic: str, window_size: int):
    """Per-metric list of scores the detector computes on healthy idle data."""
    from collections import deque

    out: Dict[str, List[float]] = {}
    for _src, metrics in streams.items():
        for metric, series in metrics.items():
            if not metric.startswith("rate_hz/"):
                continue
            window: deque = deque(maxlen=window_size)
            state: dict = {}
            scores: List[float] = []
            for _t, value in series:
                if math.isnan(value):
                    continue
                if len(window) >= 2:
                    res = STATISTICS[statistic](list(window), value, state)
                    if res is not None:
                        scores.append(res[0])
                window.append(value)
                _ewma_state_update(state, value, window_size)
            out.setdefault(metric, []).extend(scores)
    return out


def max_run(scores: List[float], threshold: float) -> int:
    best = run = 0
    for s in scores:
        run = run + 1 if s > threshold else 0
        best = max(best, run)
    return best


def main() -> int:
    RESULTS.mkdir(parents=True, exist_ok=True)
    streams = load_streams()
    report = []
    for statistic in STATS:
        for window in WINDOWS:
            per_metric = idle_scores(streams, statistic, window)
            pooled = [s for v in per_metric.values() for s in v]
            if not pooled:
                continue
            pooled_sorted = sorted(pooled)

            def q(p: float) -> float:
                return pooled_sorted[min(len(pooled_sorted) - 1, int(p * len(pooled_sorted)))]

            entry = {
                "statistic": statistic,
                "window_size": window,
                "n_scores": len(pooled),
                "p50": q(0.50),
                "p99": q(0.99),
                "p999": q(0.999),
                "max": pooled_sorted[-1],
                "max_per_metric": {k: max(v) for k, v in per_metric.items() if v},
                "max_consecutive_run": {
                    str(t): max(max_run(v, t) for v in per_metric.values())
                    for t in THRESHOLDS
                },
            }
            report.append(entry)
            print(
                f"{statistic:7s} W={window:3d}  n={len(pooled):5d}  p99={entry['p99']:6.2f} "
                f"p99.9={entry['p999']:6.2f}  max={entry['max']:7.2f}  "
                f"maxrun@3.0={entry['max_consecutive_run']['3.0']}  "
                f"maxrun@4.0={entry['max_consecutive_run']['4.0']}"
            )

    (RESULTS / "idle_margin.json").write_text(json.dumps(report, indent=1))
    print(f"wrote {RESULTS / 'idle_margin.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
