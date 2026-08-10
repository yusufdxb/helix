#!/usr/bin/env python3
"""Publication figures for the HELIX detector operating-point study.

Writes analysis/results/helix_operating_point.{png,svg} and
analysis/results/helix_selfmasking.{png,svg}.
"""
from __future__ import annotations

import json
import math
import statistics
import sys
from collections import deque
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))

from detector_replay import STATISTICS  # noqa: E402
from sweep import load_streams  # noqa: E402

RESULTS = Path(__file__).resolve().parent / "results"

CREAM = "#faf7f2"
INK = "#1b1b1b"
GRID = "#d8d2c7"
RUST = "#b4451f"
TEAL = "#1f6f6b"
OCHRE = "#c08a2e"
SLATE = "#5a6672"
STAT_COLOR = {"zscore": TEAL, "mad": RUST, "ewma": OCHRE}

SHIPPED = "zscore/thr=4/k=3/W=60/dur=2"
SESSION8 = "zscore/thr=3/k=3/W=60/dur=0"
RECOMMENDED = "zscore/thr=3.5/k=3/W=120/dur=2"

plt.rcParams.update(
    {
        "figure.facecolor": CREAM,
        "axes.facecolor": CREAM,
        "savefig.facecolor": CREAM,
        "axes.edgecolor": SLATE,
        "axes.labelcolor": INK,
        "text.color": INK,
        "xtick.color": SLATE,
        "ytick.color": SLATE,
        "font.size": 9,
        "axes.titlesize": 10,
        "axes.grid": True,
        "grid.color": GRID,
        "grid.linewidth": 0.6,
        "legend.frameon": False,
    }
)


def _z_trace(series, onset, window_size, sigma, k_sigma=10.0, horizon=15.0):
    """Score trajectory after a synthetic k_sigma step drop, one window size."""
    window: deque = deque(maxlen=window_size)
    ts, zs = [], []
    for t, v in series:
        if math.isnan(v):
            continue
        value = v - k_sigma * sigma if t > onset else v
        if len(window) >= 2:
            res = STATISTICS["zscore"](list(window), value, {})
            if res is not None and onset - 2.0 <= t <= onset + horizon:
                ts.append(t - onset)
                zs.append(res[0])
        window.append(value)
    return ts, zs


def figure_operating_point(sweep, margin):
    rows = sweep["rows"]
    by_label = {r["label"]: r for r in rows}
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.4))

    # (a) tradeoff scatter -----------------------------------------------------
    ax = axes[0]
    for stat in ("zscore", "mad", "ewma"):
        pts = [
            (r["real_false_faults_per_hour"],
             r["synthetic_latency"]["step_sigma_5"]["detect_rate"])
            for r in rows
            if r["statistic"] == stat
        ]
        ax.scatter(
            [p[0] + 0.6 for p in pts],
            [p[1] for p in pts],
            s=14,
            alpha=0.45,
            color=STAT_COLOR[stat],
            edgecolors="none",
            label=stat,
        )
    for label, name, colour, dx, dy in [
        (SHIPPED, "shipped 4.0 / W=60", INK, 6, 14),
        (SESSION8, "Session 8  3.0 / W=60", SLATE, -95, -18),
        (RECOMMENDED, "recommended 3.5 / W=120", RUST, 8, -22),
    ]:
        r = by_label[label]
        x = r["real_false_faults_per_hour"] + 0.6
        y = r["synthetic_latency"]["step_sigma_5"]["detect_rate"]
        ax.scatter([x], [y], s=95, facecolors="none", edgecolors=colour, linewidths=1.9, zorder=5)
        ax.annotate(
            name,
            (x, y),
            textcoords="offset points",
            xytext=(dx, dy),
            fontsize=8,
            color=colour,
            weight="bold",
        )
    ax.set_xscale("log")
    ax.set_xlabel("false faults per hour on REAL idle data\n(+0.6 offset so zero is plottable, log axis)")
    ax.set_ylabel("SYNTHETIC detection rate,\nsustained 5-sigma rate step")
    ax.set_title("(a) 570 configurations: what each threshold buys", loc="left")
    ax.set_ylim(-0.04, 1.04)
    ax.legend(loc="lower right", fontsize=8, title="statistic", title_fontsize=8)

    # (b) detection by fault model --------------------------------------------
    ax = axes[1]
    models = ["stale", "step_sigma_3", "step_sigma_5", "step_sigma_10", "step_drop_50", "ramp_sigma_5"]
    pretty = ["topic\ndeath", "3 sigma\nstep", "5 sigma\nstep", "10 sigma\nstep", "50% rate\ncollapse", "5 sigma\nramp 30 s"]
    width = 0.38
    xs = list(range(len(models)))
    for offset, label, colour, name in [
        (-width / 2, SHIPPED, SLATE, "shipped: z>4.0, W=60"),
        (width / 2, RECOMMENDED, RUST, "recommended: z>3.5, W=120"),
    ]:
        r = by_label[label]
        vals = [r["synthetic_latency"][m]["detect_rate"] for m in models]
        bars = ax.bar([x + offset for x in xs], vals, width, color=colour, label=name)
        lift = 0.13 if offset < 0 else 0.03
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, v + lift, f"{v:.2f}",
                    ha="center", fontsize=7, color=colour)
    ax.set_xticks(xs)
    ax.set_xticklabels(pretty, fontsize=7.5)
    ax.set_ylim(0, 1.45)
    ax.set_ylabel("SYNTHETIC detection rate (62 injections each)")
    ax.set_title("(b) What each config actually catches", loc="left")
    ax.legend(loc="upper right", fontsize=8)

    # (c) idle margin ----------------------------------------------------------
    ax = axes[2]
    for entry in margin:
        if entry["statistic"] != "zscore" or entry["window_size"] not in (60, 120):
            continue
        colour = SLATE if entry["window_size"] == 60 else RUST
        runs = entry["max_consecutive_run"]
        thr = sorted(float(t) for t in runs)
        ax.plot(
            thr,
            [runs[f"{t:g}"] if f"{t:g}" in runs else runs[str(t)] for t in thr],
            marker="o",
            color=colour,
            label=f"window_size = {entry['window_size']}",
        )
    ax.axhline(3, color=INK, linestyle="--", linewidth=1.0)
    ax.text(4.6, 3.2, "consecutive_trigger = 3", ha="left", fontsize=7.5, color=INK)
    ax.axhline(5, color=OCHRE, linestyle=":", linewidth=1.2)
    ax.text(4.6, 5.35, "samples needed for min_anomaly_duration_s = 2.0", ha="left",
            fontsize=7.5, color="#8a6416")
    ax.set_xlabel("threshold")
    ax.set_ylabel("longest run of consecutive above-threshold\nsamples seen on REAL idle data")
    ax.set_title("(c) How close idle noise gets to firing", loc="left")
    ax.legend(loc="upper right", fontsize=8)

    fig.suptitle(
        "HELIX anomaly detector: measured operating point. "
        "False-alarm axis is REAL idle GO2 telemetry (901.5 s); detection axis is SYNTHETIC onsets on that real baseline.",
        fontsize=10, y=1.0, x=0.005, ha="left",
    )
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(RESULTS / "helix_operating_point.png", dpi=220)
    fig.savefig(RESULTS / "helix_operating_point.svg")
    plt.close(fig)


def figure_selfmasking(streams):
    """Why the shipped config cannot see a sustained fault: the window eats it."""
    metrics = streams["extended_5min"]
    series = metrics["rate_hz/utlidar_imu"]
    finite = [v for _t, v in series if not math.isnan(v)]
    sigma = statistics.pstdev(finite)
    onset = 150.0

    fig, ax = plt.subplots(figsize=(7.6, 4.2))
    for window_size, colour, style in [(60, SLATE, "-"), (120, RUST, "-")]:
        ts, zs = _z_trace(series, onset, window_size, sigma)
        ax.plot(ts, zs, style, color=colour, linewidth=1.8, label=f"window_size = {window_size}")
    ax.axhline(4.0, color=SLATE, linestyle="--", linewidth=1.1)
    ax.text(14.6, 4.15, "shipped threshold 4.0", ha="right", fontsize=8, color=SLATE)
    ax.axhline(3.5, color=RUST, linestyle="--", linewidth=1.1)
    ax.text(14.6, 3.05, "recommended threshold 3.5", ha="right", fontsize=8, color=RUST)
    ax.axvspan(0, 2.0, color=OCHRE, alpha=0.18, lw=0)
    ax.text(2.4, 11.8, "min_anomaly_duration_s = 2.0\nno fault may be emitted inside this band",
            ha="left", fontsize=8, color="#8a6416")
    ax.axvline(0, color=INK, linewidth=1.0)
    ax.set_xlabel("seconds after a SYNTHETIC sustained 10-sigma rate step\n"
                  "injected into REAL idle /utlidar/imu telemetry")
    ax.set_ylabel("detector z-score")
    ax.set_title(
        "Self-masking: a sustained fault contaminates its own baseline\n"
        "before the duration gate opens", loc="left")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_xlim(-2, 15)
    fig.tight_layout()
    fig.savefig(RESULTS / "helix_selfmasking.png", dpi=220)
    fig.savefig(RESULTS / "helix_selfmasking.svg")
    plt.close(fig)


def main() -> int:
    sweep = json.loads((RESULTS / "sweep.json").read_text())
    margin = json.loads((RESULTS / "idle_margin.json").read_text())
    figure_operating_point(sweep, margin)
    figure_selfmasking(load_streams())
    print(f"wrote figures to {RESULTS}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
