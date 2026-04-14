#!/usr/bin/env python3
"""
HELIX Realistic Anomaly Detection Benchmark
=============================================
Evaluates the Z-score anomaly detector against non-trivial scenarios
where perfect separation is NOT expected.

Scenarios:
  1. Non-Gaussian (Laplace) noise -- heavier tails challenge Z-score assumptions
  2. Gradual drift -- slow ramp that may evade threshold detectors
  3. Transient spikes -- validates consecutive_trigger rejection
  4. Marginal anomalies -- near-threshold signals in realistic sensor noise

Run:
  python3 scripts/bench_realistic_anomalies.py

Outputs:
  results/realistic_anomaly_results.json
"""

from __future__ import annotations

import json
import math
import os
import random
import time
from collections import deque
from typing import Dict, List

# ---------------------------------------------------------------------------
# _AnomalyCore -- copied from benchmark_helix.py
# ---------------------------------------------------------------------------

FLAT_SIGNAL_EPSILON = 1e-6


class _AnomalyCore:
    """Pure-Python port of AnomalyDetector._process_sample for benchmarking."""

    def __init__(self, zscore_threshold: float, consecutive_trigger: int, window_size: int):
        self.zscore_threshold = zscore_threshold
        self.consecutive_trigger = consecutive_trigger
        self.window_size = window_size
        self._windows: Dict[str, deque] = {}
        self._consecutive: Dict[str, int] = {}
        self.fault_count = 0
        self.fault_timestamps: List[float] = []

    def reset(self) -> None:
        self._windows.clear()
        self._consecutive.clear()
        self.fault_count = 0
        self.fault_timestamps.clear()

    def process_sample(self, metric_name: str, value: float) -> bool:
        if metric_name not in self._windows:
            self._windows[metric_name] = deque(maxlen=self.window_size)
            self._consecutive[metric_name] = 0
        window = self._windows[metric_name]

        fault_emitted = False
        if len(window) >= 2:
            samples = list(window)
            mean = sum(samples) / len(samples)
            variance = sum((s - mean) ** 2 for s in samples) / len(samples)
            std = math.sqrt(variance)

            if std >= FLAT_SIGNAL_EPSILON:
                zscore = abs((value - mean) / std)
                if zscore > self.zscore_threshold:
                    self._consecutive[metric_name] += 1
                    if self._consecutive[metric_name] >= self.consecutive_trigger:
                        self.fault_count += 1
                        self.fault_timestamps.append(time.perf_counter())
                        fault_emitted = True
                else:
                    self._consecutive[metric_name] = 0

        window.append(value)
        return fault_emitted


# ---------------------------------------------------------------------------
# Random helpers (stdlib only -- no numpy dependency)
# ---------------------------------------------------------------------------

def laplace_sample(rng: random.Random, loc: float, scale: float) -> float:
    """Draw a single sample from Laplace(loc, scale)."""
    u = rng.random() - 0.5
    return loc - scale * math.copysign(1, u) * math.log(1 - 2 * abs(u))


def gauss_sample(rng: random.Random, mu: float, sigma: float) -> float:
    return rng.gauss(mu, sigma)


# ---------------------------------------------------------------------------
# Scenario 1: Non-Gaussian (Laplace) noise
# ---------------------------------------------------------------------------

def scenario_laplace(
    seed: int = 42,
    n_trials: int = 200,
    window_size: int = 60,
    consecutive_trigger: int = 3,
    baseline_loc: float = 10.0,
    baseline_scale: float = 1.0,
    spike_Ks: List[float] = None,
    zscore_thresholds: List[float] = None,
) -> Dict:
    if spike_Ks is None:
        spike_Ks = [3.0, 5.0, 8.0, 12.0]
    if zscore_thresholds is None:
        zscore_thresholds = [2.0, 3.0, 4.0]

    rng = random.Random(seed)
    results = []

    for threshold in zscore_thresholds:
        for K in spike_Ks:
            # --- Positive trials: inject consecutive spike at mean + K*scale ---
            tp = 0
            for _ in range(n_trials):
                det = _AnomalyCore(threshold, consecutive_trigger, window_size)
                for _ in range(window_size):
                    det.process_sample("m", laplace_sample(rng, baseline_loc, baseline_scale))
                fc_before = det.fault_count
                spike_val = baseline_loc + K * baseline_scale
                for _ in range(consecutive_trigger):
                    det.process_sample("m", spike_val)
                if det.fault_count > fc_before:
                    tp += 1

            # --- Negative trials: continue Laplace draws ---
            fp = 0
            for _ in range(n_trials):
                det = _AnomalyCore(threshold, consecutive_trigger, window_size)
                for _ in range(window_size):
                    det.process_sample("m", laplace_sample(rng, baseline_loc, baseline_scale))
                fc_before = det.fault_count
                for _ in range(consecutive_trigger):
                    det.process_sample("m", laplace_sample(rng, baseline_loc, baseline_scale))
                if det.fault_count > fc_before:
                    fp += 1

            tpr = tp / n_trials
            fpr = fp / n_trials
            results.append({
                "zscore_threshold": threshold,
                "spike_K": K,
                "tpr": round(tpr, 4),
                "fpr": round(fpr, 4),
                "true_positives": tp,
                "false_positives": fp,
                "n_trials": n_trials,
            })

    return {
        "scenario": "laplace_noise",
        "description": (
            f"Baseline: {window_size} Laplace(loc={baseline_loc}, scale={baseline_scale}) samples. "
            f"Positive: {consecutive_trigger} consecutive samples at mean + K*scale. "
            f"Negative: continue Laplace draws."
        ),
        "results": results,
    }


# ---------------------------------------------------------------------------
# Scenario 2: Gradual drift
# ---------------------------------------------------------------------------

def scenario_drift(
    seed: int = 42,
    n_trials: int = 200,
    window_size: int = 60,
    consecutive_trigger: int = 3,
    zscore_threshold: float = 3.0,
    baseline_mu: float = 10.0,
    baseline_sigma: float = 0.5,
    drift_rates: List[float] = None,
    drift_length: int = 20,
) -> Dict:
    if drift_rates is None:
        drift_rates = [0.1, 0.2, 0.5, 1.0, 2.0]

    rng = random.Random(seed)
    results = []

    for R in drift_rates:
        detections = 0
        samples_to_detect: List[int] = []

        for _ in range(n_trials):
            det = _AnomalyCore(zscore_threshold, consecutive_trigger, window_size)
            for _ in range(window_size):
                det.process_sample("m", gauss_sample(rng, baseline_mu, baseline_sigma))
            fc_before = det.fault_count
            detected_at = None
            for i in range(drift_length):
                val = baseline_mu + R * (i + 1) + gauss_sample(rng, 0, baseline_sigma)
                fired = det.process_sample("m", val)
                if fired and detected_at is None:
                    detected_at = i + 1
            if det.fault_count > fc_before:
                detections += 1
                samples_to_detect.append(detected_at)

        det_rate = detections / n_trials
        mean_s2d = round(sum(samples_to_detect) / len(samples_to_detect), 1) if samples_to_detect else None
        results.append({
            "drift_rate": R,
            "detection_rate": round(det_rate, 4),
            "detections": detections,
            "mean_samples_to_detect": mean_s2d,
            "n_trials": n_trials,
        })

    # Negative trials (no drift)
    neg_fp = 0
    for _ in range(n_trials):
        det = _AnomalyCore(zscore_threshold, consecutive_trigger, window_size)
        for _ in range(window_size):
            det.process_sample("m", gauss_sample(rng, baseline_mu, baseline_sigma))
        fc_before = det.fault_count
        for _ in range(drift_length):
            det.process_sample("m", gauss_sample(rng, baseline_mu, baseline_sigma))
        if det.fault_count > fc_before:
            neg_fp += 1

    return {
        "scenario": "gradual_drift",
        "description": (
            f"Baseline: {window_size} N({baseline_mu}, {baseline_sigma}) samples. "
            f"Drift: value = {baseline_mu} + R*i + noise for {drift_length} samples. "
            f"Z-threshold={zscore_threshold}, consecutive={consecutive_trigger}."
        ),
        "drift_results": results,
        "negative_trials": {
            "false_positive_rate": round(neg_fp / n_trials, 4),
            "false_positives": neg_fp,
            "n_trials": n_trials,
        },
    }


# ---------------------------------------------------------------------------
# Scenario 3: Transient spikes (should NOT trigger)
# ---------------------------------------------------------------------------

def scenario_transient_spikes(
    seed: int = 42,
    n_trials: int = 200,
    window_size: int = 60,
    consecutive_trigger: int = 3,
    zscore_threshold: float = 3.0,
    baseline_mu: float = 10.0,
    baseline_sigma: float = 0.5,
    spike_sigma_mult: float = 5.0,
    post_spike_samples: int = 10,
) -> Dict:
    rng = random.Random(seed)

    spike_val = baseline_mu + spike_sigma_mult * baseline_sigma

    # --- Single spike ---
    single_alarms = 0
    for _ in range(n_trials):
        det = _AnomalyCore(zscore_threshold, consecutive_trigger, window_size)
        for _ in range(window_size):
            det.process_sample("m", gauss_sample(rng, baseline_mu, baseline_sigma))
        fc_before = det.fault_count
        # 1 spike
        det.process_sample("m", spike_val)
        # Return to normal
        for _ in range(post_spike_samples):
            det.process_sample("m", gauss_sample(rng, baseline_mu, baseline_sigma))
        if det.fault_count > fc_before:
            single_alarms += 1

    # --- Double spike ---
    double_alarms = 0
    for _ in range(n_trials):
        det = _AnomalyCore(zscore_threshold, consecutive_trigger, window_size)
        for _ in range(window_size):
            det.process_sample("m", gauss_sample(rng, baseline_mu, baseline_sigma))
        fc_before = det.fault_count
        # 2 consecutive spikes
        det.process_sample("m", spike_val)
        det.process_sample("m", spike_val)
        # Return to normal
        for _ in range(post_spike_samples):
            det.process_sample("m", gauss_sample(rng, baseline_mu, baseline_sigma))
        if det.fault_count > fc_before:
            double_alarms += 1

    # --- Triple spike (control -- SHOULD trigger) ---
    triple_alarms = 0
    for _ in range(n_trials):
        det = _AnomalyCore(zscore_threshold, consecutive_trigger, window_size)
        for _ in range(window_size):
            det.process_sample("m", gauss_sample(rng, baseline_mu, baseline_sigma))
        fc_before = det.fault_count
        for _ in range(consecutive_trigger):
            det.process_sample("m", spike_val)
        if det.fault_count > fc_before:
            triple_alarms += 1

    return {
        "scenario": "transient_spikes",
        "description": (
            f"Baseline: {window_size} N({baseline_mu}, {baseline_sigma}). "
            f"Spike at {spike_sigma_mult}-sigma above mean. "
            f"consecutive_trigger={consecutive_trigger}. "
            f"Single and double spikes should NOT fire; triple should."
        ),
        "single_spike": {
            "false_alarm_rate": round(single_alarms / n_trials, 4),
            "alarms": single_alarms,
            "n_trials": n_trials,
            "expected": "0% (spike count < consecutive_trigger)",
        },
        "double_spike": {
            "false_alarm_rate": round(double_alarms / n_trials, 4),
            "alarms": double_alarms,
            "n_trials": n_trials,
            "expected": "0% (spike count < consecutive_trigger)",
        },
        "triple_spike_control": {
            "true_positive_rate": round(triple_alarms / n_trials, 4),
            "detections": triple_alarms,
            "n_trials": n_trials,
            "expected": "~100% (spike count == consecutive_trigger at 5-sigma)",
        },
    }


# ---------------------------------------------------------------------------
# Scenario 4: Marginal anomalies in realistic sensor noise
# ---------------------------------------------------------------------------

def scenario_marginal(
    seed: int = 42,
    n_trials: int = 200,
    window_size: int = 60,
    consecutive_trigger: int = 3,
    baseline_mu: float = 10.0,
    baseline_sigma: float = 1.0,
    anomaly_sigmas: List[float] = None,
    zscore_thresholds: List[float] = None,
) -> Dict:
    if anomaly_sigmas is None:
        anomaly_sigmas = [3.5, 4.0, 5.0]
    if zscore_thresholds is None:
        zscore_thresholds = [2.0, 3.0, 3.5, 4.0]

    rng = random.Random(seed)
    results = []

    for threshold in zscore_thresholds:
        # --- Positive: inject 3 consecutive samples at escalating sigma multiples ---
        tp = 0
        for _ in range(n_trials):
            det = _AnomalyCore(threshold, consecutive_trigger, window_size)
            for _ in range(window_size):
                det.process_sample("m", gauss_sample(rng, baseline_mu, baseline_sigma))
            fc_before = det.fault_count
            for sigma_mult in anomaly_sigmas:
                det.process_sample("m", baseline_mu + sigma_mult * baseline_sigma)
            if det.fault_count > fc_before:
                tp += 1

        # --- Negative: continue normal draws ---
        fp = 0
        for _ in range(n_trials):
            det = _AnomalyCore(threshold, consecutive_trigger, window_size)
            for _ in range(window_size):
                det.process_sample("m", gauss_sample(rng, baseline_mu, baseline_sigma))
            fc_before = det.fault_count
            for _ in range(consecutive_trigger):
                det.process_sample("m", gauss_sample(rng, baseline_mu, baseline_sigma))
            if det.fault_count > fc_before:
                fp += 1

        tpr = tp / n_trials
        fpr = fp / n_trials
        results.append({
            "zscore_threshold": threshold,
            "tpr": round(tpr, 4),
            "fpr": round(fpr, 4),
            "true_positives": tp,
            "false_positives": fp,
            "n_trials": n_trials,
        })

    return {
        "scenario": "marginal_anomalies",
        "description": (
            f"Baseline: {window_size} N({baseline_mu}, {baseline_sigma}). "
            f"Positive: {consecutive_trigger} consecutive at "
            f"{anomaly_sigmas} sigma above mean. "
            f"These are near-threshold -- not trivially separable."
        ),
        "anomaly_sigmas": anomaly_sigmas,
        "results": results,
    }


# ---------------------------------------------------------------------------
# Output formatting
# ---------------------------------------------------------------------------

def print_scenario_1(data: Dict) -> None:
    print("\n" + "=" * 70)
    print("SCENARIO 1: Non-Gaussian (Laplace) Noise")
    print("=" * 70)
    print(data["description"])
    print()
    print(f"{'Z-thresh':>10}  {'Spike K':>8}  {'TPR':>8}  {'FPR':>8}")
    print("-" * 42)
    for r in data["results"]:
        print(f"{r['zscore_threshold']:>10.1f}  {r['spike_K']:>8.1f}  "
              f"{r['tpr']:>7.1%}  {r['fpr']:>7.1%}")


def print_scenario_2(data: Dict) -> None:
    print("\n" + "=" * 70)
    print("SCENARIO 2: Gradual Drift")
    print("=" * 70)
    print(data["description"])
    print()
    print(f"{'Drift R':>10}  {'Det.Rate':>10}  {'Mean S2D':>10}")
    print("-" * 36)
    for r in data["drift_results"]:
        s2d = f"{r['mean_samples_to_detect']}" if r["mean_samples_to_detect"] else "N/A"
        print(f"{r['drift_rate']:>10.1f}  {r['detection_rate']:>9.1%}  {s2d:>10}")
    neg = data["negative_trials"]
    print(f"\nNegative (no drift): FPR = {neg['false_positive_rate']:.1%} "
          f"({neg['false_positives']}/{neg['n_trials']})")


def print_scenario_3(data: Dict) -> None:
    print("\n" + "=" * 70)
    print("SCENARIO 3: Transient Spikes (consecutive_trigger validation)")
    print("=" * 70)
    print(data["description"])
    s = data["single_spike"]
    d = data["double_spike"]
    t = data["triple_spike_control"]
    print(f"\n  Single spike:  false alarm rate = {s['false_alarm_rate']:.1%}  "
          f"({s['alarms']}/{s['n_trials']})  -- expected: {s['expected']}")
    print(f"  Double spike:  false alarm rate = {d['false_alarm_rate']:.1%}  "
          f"({d['alarms']}/{d['n_trials']})  -- expected: {d['expected']}")
    print(f"  Triple spike:  TPR = {t['true_positive_rate']:.1%}  "
          f"({t['detections']}/{t['n_trials']})  -- expected: {t['expected']}")


def print_scenario_4(data: Dict) -> None:
    print("\n" + "=" * 70)
    print("SCENARIO 4: Marginal Anomalies in Realistic Sensor Noise")
    print("=" * 70)
    print(data["description"])
    print(f"\nAnomaly injection sigmas: {data['anomaly_sigmas']}")
    print(f"{'Z-thresh':>10}  {'TPR':>8}  {'FPR':>8}")
    print("-" * 30)
    for r in data["results"]:
        print(f"{r['zscore_threshold']:>10.1f}  {r['tpr']:>7.1%}  {r['fpr']:>7.1%}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    print("HELIX Realistic Anomaly Detection Benchmark")
    print("=" * 70)
    print("Seed: 42 | Trials per condition: 200")

    s1 = scenario_laplace()
    print_scenario_1(s1)

    s2 = scenario_drift()
    print_scenario_2(s2)

    s3 = scenario_transient_spikes()
    print_scenario_3(s3)

    s4 = scenario_marginal()
    print_scenario_4(s4)

    # -- Summary --
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)

    # Laplace FPR at Z=3.0
    laplace_z3 = [r for r in s1["results"] if r["zscore_threshold"] == 3.0]
    laplace_fpr = max(r["fpr"] for r in laplace_z3) if laplace_z3 else 0
    print(f"  Laplace noise worst FPR (Z=3.0):  {laplace_fpr:.1%}")

    # Drift: lowest rate detected
    drift_low = s2["drift_results"][0]
    print(f"  Drift R={drift_low['drift_rate']}: detection rate = "
          f"{drift_low['detection_rate']:.1%}")

    # Transient: should be 0%
    print(f"  Single-spike false alarm rate:     "
          f"{s3['single_spike']['false_alarm_rate']:.1%}")
    print(f"  Double-spike false alarm rate:     "
          f"{s3['double_spike']['false_alarm_rate']:.1%}")

    # Marginal TPR at Z=3.0
    marginal_z3 = [r for r in s4["results"] if r["zscore_threshold"] == 3.0]
    marginal_tpr = marginal_z3[0]["tpr"] if marginal_z3 else 0
    print(f"  Marginal anomaly TPR (Z=3.0):     {marginal_tpr:.1%}")

    # -- Save JSON --
    output = {
        "benchmark": "helix_realistic_anomaly_detection",
        "version": "1.0",
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "seed": 42,
        "n_trials_per_condition": 200,
        "detector_config": {
            "window_size": 60,
            "consecutive_trigger": 3,
        },
        "scenarios": {
            "1_laplace_noise": s1,
            "2_gradual_drift": s2,
            "3_transient_spikes": s3,
            "4_marginal_anomalies": s4,
        },
    }

    results_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results")
    os.makedirs(results_dir, exist_ok=True)
    out_path = os.path.join(results_dir, "realistic_anomaly_results.json")

    with open(out_path, "w") as f:
        json.dump(output, f, indent=2)
    print(f"\nWrote: {out_path}")


if __name__ == "__main__":
    main()
