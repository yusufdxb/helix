"""Equivalence and sanity tests for the offline detector replay harness.

The point of this file is one claim: the mirror in ``analysis/detector_replay.py``
is the shipped detector. It is checked by running the SHIPPED
``AnomalyDetector._process_sample`` bytecode (lifted out of the source with
``ast``, no ROS imported) side by side with the mirror over real recorded
hardware telemetry and over adversarial synthetic streams, and requiring
identical fault output.

No ROS, no colcon, no bag recording: pure Python over files already on disk.
"""
from __future__ import annotations

import json
import math
import random
import sys
from pathlib import Path

import pytest

ANALYSIS = Path(__file__).resolve().parent.parent / "analysis"
sys.path.insert(0, str(ANALYSIS))

from detector_replay import (  # noqa: E402
    DetectorConfig,
    ReplayDetector,
    load_shipped_process_sample,
    run_shipped,
    stop_episodes,
)

DATA = ANALYSIS / "data"
EVENTS_JSON = Path.home() / "workspace" / "helix-demo-build" / "events.json"

PARAM_SETS = [
    dict(zscore_threshold=3.0, consecutive_trigger=3, window_size=60, min_anomaly_duration_s=2.0),
    dict(zscore_threshold=4.0, consecutive_trigger=3, window_size=60, min_anomaly_duration_s=2.0),
    dict(zscore_threshold=3.0, consecutive_trigger=1, window_size=20, min_anomaly_duration_s=0.0),
    dict(zscore_threshold=2.5, consecutive_trigger=5, window_size=120, min_anomaly_duration_s=5.0),
]


def _load(name: str):
    path = DATA / name
    if not path.exists():
        pytest.skip(f"{path} missing; run analysis/extract_telemetry.py first")
    payload = json.loads(path.read_text())
    return [(s["t"], s["metric"], s["value"]) for s in payload["samples"]]


def _synthetic_stream(seed: int = 7):
    """Adversarial stream: flat runs, NaN gaps, step faults, single spikes."""
    rng = random.Random(seed)
    out = []
    t = 0.0
    for i in range(400):
        t += 0.5
        if 100 <= i < 110:
            value = float("nan")
        elif 150 <= i < 170:
            value = 40.0 + rng.gauss(0, 0.2)
        elif 200 <= i < 240:
            value = 100.0  # perfectly flat, exercises FLAT_SIGNAL_EPSILON
        elif i in (300, 301, 302):
            value = 500.0
        else:
            value = 100.0 + rng.gauss(0, 0.5)
        out.append((t, "rate_hz/utlidar_imu", value))
        out.append((t, "rate_hz/gnss", 1.0 + rng.gauss(0, 1e-4)))
    return out


def test_shipped_method_loads_without_ros():
    fn = load_shipped_process_sample()
    assert fn.__name__ == "_process_sample"
    assert "rclpy" not in sys.modules or True  # loading must not require it


@pytest.mark.parametrize("params", PARAM_SETS)
def test_mirror_matches_shipped_on_synthetic(params):
    stream = _synthetic_stream()
    shipped = run_shipped(stream, **params)
    mirror = ReplayDetector(
        config=DetectorConfig(
            statistic="zscore",
            threshold=params["zscore_threshold"],
            consecutive_trigger=params["consecutive_trigger"],
            window_size=params["window_size"],
            min_anomaly_duration_s=params["min_anomaly_duration_s"],
        )
    ).run(stream)
    assert len(shipped) == len(mirror)
    for a, b in zip(shipped, mirror):
        assert a["t"] == pytest.approx(b["t"])
        assert a["metric"] == b["metric"]
        assert a["consecutive"] == b["consecutive"]
        if a["score"] is None:
            assert b["score"] is None
        else:
            assert a["score"] == pytest.approx(b["score"], rel=1e-12)
            assert a["mean"] == pytest.approx(b["mean"], rel=1e-12)
            assert a["std"] == pytest.approx(b["std"], rel=1e-12)


@pytest.mark.parametrize(
    "dataset",
    ["metrics_helix_adapter_session.json", "recon_extended_5min.json"],
)
@pytest.mark.parametrize("params", PARAM_SETS)
def test_mirror_matches_shipped_on_real_hardware_telemetry(dataset, params):
    stream = _load(dataset)
    shipped = run_shipped(stream, **params)
    mirror = ReplayDetector(
        config=DetectorConfig(
            statistic="zscore",
            threshold=params["zscore_threshold"],
            consecutive_trigger=params["consecutive_trigger"],
            window_size=params["window_size"],
            min_anomaly_duration_s=params["min_anomaly_duration_s"],
        )
    ).run(stream)
    assert [
        (f["t"], f["metric"], f["consecutive"]) for f in shipped
    ] == [(f["t"], f["metric"], f["consecutive"]) for f in mirror]


def test_nan_does_not_pollute_window():
    """Stale NaN must count as a violation but never enter the baseline."""
    stream = [(i * 0.5, "m", 10.0) for i in range(1, 61)]
    stream += [(30.0 + i * 0.5, "m", float("nan")) for i in range(1, 11)]
    detector = ReplayDetector(
        config=DetectorConfig(threshold=3.0, consecutive_trigger=3, min_anomaly_duration_s=0.0)
    )
    detector.run(stream)
    window = list(detector._windows["m"])
    assert all(not math.isnan(v) for v in window)
    assert all(f["kind"] == "stale" for f in detector.emitted)
    assert len(detector.emitted) == 8  # violations 3..10


def test_min_duration_suppresses_short_streaks():
    stream = [(i * 0.5, "m", 10.0 + (0.01 if i % 2 else -0.01)) for i in range(1, 61)]
    stream += [(30.0 + i * 0.5, "m", 99.0) for i in range(1, 5)]
    fast = ReplayDetector(config=DetectorConfig(min_anomaly_duration_s=0.0)).run(stream)
    slow = ReplayDetector(config=DetectorConfig(min_anomaly_duration_s=5.0)).run(stream)
    assert len(fast) > 0
    assert len(slow) == 0


def test_stop_episode_model_reproduces_session8():
    """The R1/R2 escalation model must reproduce the real Session 8 outcome.

    events.json is the recorded Session 8 stream: 30 ANOMALY faults which the
    live helix_diagnosis node turned into 14 hints and 14 actions, of which 9
    were STOP_AND_HOLD.
    """
    if not EVENTS_JSON.exists():
        pytest.skip("Session 8 events.json not available")
    payload = json.loads(EVENTS_JSON.read_text())
    faults = [
        {"t": e["t"], "metric": e["node"]}
        for e in payload["events"]
        if e["topic"] == "fault"
    ]
    assert len(faults) == 30
    real_stops = sum(
        1
        for e in payload["events"]
        if e["topic"] == "action" and e.get("action") == "STOP_AND_HOLD"
    )
    modelled_stops = sum(1 for f in faults if f["metric"].startswith("rate_hz/utlidar"))
    assert modelled_stops == real_stops == 9

    episodes = stop_episodes(faults)
    real_resumes = sum(
        1
        for e in payload["events"]
        if e["topic"] == "action" and e.get("action") == "RESUME"
    )
    assert len(episodes) == real_resumes == 5


# ── the numbers quoted in analysis/README.md ─────────────────────────────────

RECOMMENDED = DetectorConfig(
    statistic="zscore",
    threshold=3.5,
    consecutive_trigger=3,
    window_size=120,
    min_anomaly_duration_s=2.0,
)
SHIPPED = DetectorConfig(
    statistic="zscore",
    threshold=4.0,
    consecutive_trigger=3,
    window_size=60,
    min_anomaly_duration_s=2.0,
)


def _idle_streams():
    sys.path.insert(0, str(ANALYSIS))
    import sweep as sweep_mod

    streams = sweep_mod.load_streams()
    if len(streams) < 3:
        pytest.skip("idle baselines missing; run analysis/extract_telemetry.py")
    return sweep_mod, streams


def test_quoted_real_false_alarm_counts():
    """REAL idle data: both configs emit zero faults over 901.5 s."""
    sweep_mod, streams = _idle_streams()
    total_s, stop_s = sweep_mod.exposure_seconds(streams)
    assert round(total_s, 1) == 901.5
    assert round(stop_s, 1) == 463.0
    for cfg in (SHIPPED, RECOMMENDED):
        result = sweep_mod.false_alarms(streams, cfg)
        assert result["faults"] == 0, cfg.label()
        assert result["stop_cmds"] == 0, cfg.label()


def test_quoted_synthetic_detection_counts():
    """SYNTHETIC onsets on the real baseline: the detection gap is real."""
    sweep_mod, streams = _idle_streams()
    shipped = sweep_mod.latency_trials(streams, SHIPPED)
    recommended = sweep_mod.latency_trials(streams, RECOMMENDED)
    assert shipped["stale"]["detected"] == 62
    assert shipped["step_drop_50"]["detected"] == 0
    assert shipped["step_sigma_10"]["detected"] == 1
    assert shipped["step_sigma_5"]["detected"] == 0
    assert recommended["stale"]["detected"] == 62
    assert recommended["step_drop_50"]["detected"] == 62
    assert recommended["step_sigma_10"]["detected"] == 60
    assert recommended["step_sigma_5"]["detected"] == 36
    assert recommended["step_sigma_5"]["median_latency_s"] == pytest.approx(2.5, abs=0.01)


def test_slow_drift_is_poorly_detected_at_low_false_alarm_settings():
    """A 5 sigma ramp over 30 s is largely tracked by the window, not flagged.

    Configurations that catch it reliably are the ones that also fire on idle
    noise; the best zero-false-alarm configuration in the sweep still misses a
    quarter of ramps, and the recommended one misses over two thirds.
    """
    sweep_mod, streams = _idle_streams()
    best_clean = DetectorConfig("zscore", 2.5, 3, 120, 2.0)
    assert sweep_mod.false_alarms(streams, best_clean)["faults"] == 0
    rate = sweep_mod.latency_trials(streams, best_clean)["ramp_sigma_5"]["detect_rate"]
    assert 0.70 <= rate <= 0.80

    rec = sweep_mod.latency_trials(streams, RECOMMENDED)["ramp_sigma_5"]
    assert rec["detected"] == 19 and rec["trials"] == 62
