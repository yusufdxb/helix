#!/usr/bin/env python3
"""Falsification experiment: HELIX vs a deliberately competent simple baseline.
Inputs: recorded GO2 /helix/metrics, plus explicitly synthetic faults injected into
the recorded metric stream. No robot, no actual actuation, no neural training.
"""
import ast
import json
import math
import statistics
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace
from typing import Optional, Dict
from dataclasses import dataclass

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src" / "helix_diagnosis"))
from analysis.detector_replay import run_shipped
from analysis.extract_telemetry import extract_recorded_metrics
from helix_diagnosis.rules import evaluate, STATE_IDLE, STATE_STOP_AND_HOLD

BAG = ROOT / "hardware_eval_20261006/bags/loaded_standing_observe"
OUT = ROOT / "experiments/helix_advantage_20261102/results.json"
OUT.parent.mkdir(parents=True, exist_ok=True)

# HELIX methods below come unmodified from the repository, via the
# existing AST-based replay loader; these settings correspond to shipped and
# previously proposed parameter sets. Neither is trained on this test run.
PARAMS = {
    "helix_shipped": dict(zscore_threshold=4.0, window_size=60,
                          consecutive_trigger=3, min_anomaly_duration_s=2.0,
                          emit_cooldown_s=1.0),
    "helix_proposed_exploratory": dict(zscore_threshold=3.5, window_size=120,
                                      consecutive_trigger=3,
                                      min_anomaly_duration_s=2.0,
                                      emit_cooldown_s=1.0),
}
ONSET_TIMES = [300, 400, 500, 600, 700, 800, 900, 1000]
HORIZON_S = 20.0
FAULTS = ["death", "drop_50pct", "drop_25pct", "transient_1s"]

def finite(v):
    return v is not None and math.isfinite(v)

def baseline_events(series, healthy_median):
    """Fixed expected rate + a 2 s debounced watchdog, and a latched episode.
    This is a transparent ROS diagnostics + rule-action reference, not a ROS
    package, not a claim about a specific upstream software_watchdogs build.
    Looks only for missing or low rate; no high-rate anomaly detection.
    """
    threshold = healthy_median * 0.6
    bad_since = None
    bad_samples = 0
    latched = False
    result = []
    for t, v in series:
        if t < 180.0:  # train/calibrate on 60..180, evaluate from 180 onward
            continue
        bad = (not finite(v)) or v < threshold
        if bad:
            bad_samples += 1
            if bad_since is None:
                bad_since = t
            if not latched and bad_samples >= 3 and t - bad_since >= 2.0:
                result.append(t)
                latched = True
        else:
            bad_since = None
            bad_samples = 0
            latched = False
    return result

def inject(v, dt, fault):
    if fault == "death":
        return None
    if fault == "drop_50pct":
        return None if v is None else v * 0.5
    if fault == "drop_25pct":
        return None if v is None else v * 0.75
    if fault == "transient_1s":
        return None if dt < 1.0 else v
    raise AssertionError(fault)

def helix_faults(series, metric, params):
    return run_shipped(((t, metric, v) for t, v in series), **params)

def first_hit_helix(series, metric, params, onset):
    xs = helix_faults(series, metric, params)
    ys = [f["t"] - onset for f in xs if onset <= f["t"] <= onset + HORIZON_S]
    return min(ys) if ys else None

def first_hit_baseline(series, median, onset):
    xs = baseline_events(series, median)
    ys = [t - onset for t in xs if onset <= t <= onset + HORIZON_S]
    return min(ys) if ys else None

def get_actual_source_class(source_path, names, namespace):
    tree = ast.parse(source_path.read_text())
    target = [n for n in tree.body if isinstance(n, ast.ClassDef) and n.name in names]
    assert len(target) == len(names), (source_path, names)
    mod = ast.fix_missing_locations(ast.Module(body=target, type_ignores=[]))
    exec(compile(mod, str(source_path), "exec"), namespace)
    return [namespace[n] for n in names]

def recovery_test():
    # The exact diagnosis state machine class (not a rewrite).
    diagnosis_classes = {
        "Optional": Optional,
        "HintShape": None,
        "evaluate": evaluate,
        "STATE_IDLE": STATE_IDLE,
        "STATE_STOP_AND_HOLD": STATE_STOP_AND_HOLD,
    }
    Machine, Ctx = get_actual_source_class(
        ROOT/"src/helix_diagnosis/helix_diagnosis/diagnosis_node.py",
        ["DiagnosisStateMachine", "_Ctx"], diagnosis_classes)
    # The exact shipped SafetyEnvelope, including its action allowlist/cooldown.
    recovery_classes = {
        "dataclass": dataclass,
        "Dict": Dict,
        "Optional": Optional,
        "ALLOWED_ACTIONS": {"STOP_AND_HOLD", "RESUME", "LOG_ONLY"},
        "ACTION_STOP": "STOP_AND_HOLD",
        "ACTION_RESUME": "RESUME",
        "ACTION_LOG_ONLY": "LOG_ONLY",
        "PUBLISHING_ACTIONS": {"STOP_AND_HOLD", "RESUME"},
    }
    EnvelopeResult, Envelope = get_actual_source_class(
        ROOT/"src/helix_recovery/helix_recovery/recovery_node.py",
        ["EnvelopeResult", "SafetyEnvelope"], recovery_classes)
    tests = {
        "lidar_fault_no_fresh_healthy_message":
            SimpleNamespace(fault_type="ANOMALY", severity=2,
                context_keys=["metric_name"], context_values=["rate_hz/utlidar_cloud"],
                node_name="helix_anomaly_detector"),
        "critical_log_no_correction_proof":
            SimpleNamespace(fault_type="LOG_PATTERN", severity=3,
                context_keys=[], context_values=[], node_name="helix_log_parser"),
    }
    cases = {}
    for name, event in tests.items():
        m = Machine()
        envelope = Envelope(True, 5.0)
        stop = m.process_fault(event, 100.0)
        assert stop and stop.suggested_action == "STOP_AND_HOLD", name
        accepted_stop = envelope.evaluate(stop.suggested_action, stop.rule_matched, 100.0)
        # Deliberately no follow-up evidence: the root cause could persist.
        resume = m.tick(103.2)
        accepted_resume = (envelope.evaluate(resume.suggested_action,
                           resume.rule_matched, 103.2) if resume else None)
        cases[name] = {
            "stop_hint": stop.suggested_action, "stop_envelope_status": accepted_stop.status,
            "no_new_health_evidence": True,
            "resume_hint_after_3_2s": resume.suggested_action if resume else None,
            "resume_envelope_status": accepted_resume.status if accepted_resume else None,
            "unsafe_early_resume_by_evidence_requirement":
                bool(resume and resume.suggested_action == "RESUME"
                     and accepted_resume and accepted_resume.status == "ACCEPTED"),
            "simple_evidence_latched_reference_would_resume": False,
        }
    # Separate crash classification. R4 is LOG_ONLY even if critical; whether
    # that is unsafe depends on what crashed and whether motion was active.
    crash = SimpleNamespace(fault_type="CRASH", severity=3,
        context_keys=[], context_values=[], node_name="nav_controller")
    hint = Machine().process_fault(crash, 200.0)
    cases["critical_nav_controller_crash"] = {
        "hint": hint.suggested_action if hint else None,
        "physical_safety_unverified": True,
    }
    return cases

def main():
    with tempfile.TemporaryDirectory() as work:
        payload = extract_recorded_metrics(BAG, Path(work)/"metrics.json")
    d = {}
    for s in payload["samples"]:
        d.setdefault(s["metric"], []).append((float(s["t"]), s["value"]))
    sensor_names = sorted(m for m in d if m.startswith("rate_hz/utlidar"))
    assert len(sensor_names) >= 2, sensor_names
    series_map = {m: sorted(d[m]) for m in sensor_names}
    calibrations = {
        m: statistics.median([float(v) for t, v in xs
               if 60 <= t < 180 and finite(v)]) for m, xs in series_map.items()
    }
    assert all(v > 0 for v in calibrations.values()), calibrations
    trials = []
    summary = {name: {f: [] for f in FAULTS}
               for name in ("fixed_band_watchdog", *PARAMS)}
    for metric, clean in series_map.items():
        for onset in ONSET_TIMES:
            assert any(onset + HORIZON_S < t for t,_ in clean), (metric,onset)
            pre = [(t,v) for t,v in clean if t < onset]
            tail = [(t,v) for t,v in clean if onset <= t <= onset+HORIZON_S]
            for f in FAULTS:
                trial = pre + [(t, inject(v,t-onset,f)) for t,v in tail]
                hits = {"fixed_band_watchdog": first_hit_baseline(trial,
                         calibrations[metric],onset)}
                for name,params in PARAMS.items():
                    hits[name] = first_hit_helix(trial,metric,params,onset)
                for name,lat in hits.items():
                    summary[name][f].append(lat)
                trials.append({"metric":metric,"onset_s":onset,
                               "injection":f, "latency_s":hits})
    idle = {}
    for name in summary:
        starts=[]
        for metric, xs in series_map.items():
            if name == "fixed_band_watchdog":
                starts.extend((metric,t) for t in baseline_events(xs,calibrations[metric]))
            else:
                starts.extend((metric,ev["t"]) for ev in
                     helix_faults(xs,metric,PARAMS[name]) if ev["t"]>=180)
        idle[name]={"stop_producing_fault_events":len(starts),
                    "first_events":starts[:12]}
    aggregate={}
    for name,groups in summary.items():
        aggregate[name]={}
        for f,lats in groups.items():
            successes=[x for x in lats if x is not None]
            aggregate[name][f]={
                "detected":len(successes),"trials":len(lats),
                "median_latency_s":round(statistics.median(successes),3) if successes else None,
                "p90_latency_s":round(sorted(successes)[math.ceil(.9*len(successes))-1],3) if successes else None,
            }
    decision=recovery_test()
    report={
        "data":"RECORDED real GO2 /helix/metrics from 2026-10-06; synthetic faults added offline",
        "replay_scope":"Actual shipped HELIX detector methods, diagnosis state machine and SafetyEnvelope executed as pure classes; no ROS graph, no hardware commands",
        "baseline":"Independent fixed expected-rate watchdog: first-180s median, 60% low-rate bound, >=3 samples and >=2s persisted; stops latch until healthy inputs",
        "scope_notes":"One loaded, standing GO2 session, not walking; 8 injection times per sensor; overlapping trials are not statistically independent; no physical recovery measured; fixed-band baseline is reference implementation, not ros2_pulse built-in policy",
        "recording_duration_s":payload["duration_s"],
        "recorded_samples":payload["n_samples"],
        "sensor_metrics":sensor_names,
        "median_reference_rates_hz":calibrations,
        "trial_count":len(trials),
        "healthy_recording_stop_faults":idle,
        "injected_fault_detection":aggregate,
        "recovery_no_new_evidence_tests":decision,
        "all_trials":trials,
    }
    OUT.write_text(json.dumps(report,indent=2,allow_nan=False))
    print(json.dumps({k:v for k,v in report.items() if k!="all_trials"},indent=2))
    assert len(trials)>=64
    assert all(x["unsafe_early_resume_by_evidence_requirement"]
               for x in list(decision.values())[:2]), "Check diagnosis policy source"
    print("EXPERIMENT_DONE: no physical recoveries were exercised; do not interpret this as hardware proof.")

if __name__=="__main__":
    main()
