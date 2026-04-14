#!/usr/bin/env python3
"""
bench_log_parser.py — Quantitative benchmark for the HELIX LogParser.

Standalone (no ROS 2 required). Loads rules from the YAML config, ports the
core matching + dedup logic, and measures:

  Part 1: Detection accuracy (precision, recall per rule and overall)
  Part 2: Throughput (messages/sec for compiled regex matching)
  Part 3: Dedup behavior validation

Results are printed and saved to results/log_parser_results.json.
"""

import json
import os
import random
import re
import sys
import time
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

# ---------------------------------------------------------------------------
# Path resolution
# ---------------------------------------------------------------------------
SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
RULES_PATH = REPO_ROOT / "src" / "helix_bringup" / "config" / "log_rules.yaml"
RESULTS_DIR = REPO_ROOT / "results"
RESULTS_FILE = RESULTS_DIR / "log_parser_results.json"

# ---------------------------------------------------------------------------
# Constants mirrored from log_parser.py
# ---------------------------------------------------------------------------
DEFAULT_DEDUP_WINDOW_SEC = 5.0


# ---------------------------------------------------------------------------
# Rule loader (ported from LogParser._load_rules)
# ---------------------------------------------------------------------------
def load_rules(path: Path) -> List[Dict[str, Any]]:
    """Load YAML rules and compile regex patterns (case-insensitive)."""
    with open(path) as f:
        data = yaml.safe_load(f)

    rules = data.get("rules", [])
    compiled: List[Dict[str, Any]] = []
    for rule in rules:
        required = {"id", "pattern", "fault_type", "severity", "detail_template"}
        if required - set(rule.keys()):
            continue
        rule["_compiled"] = re.compile(rule["pattern"], re.IGNORECASE)
        compiled.append(rule)
    return compiled


# ---------------------------------------------------------------------------
# Matcher (ported from LogParser._on_log, without ROS types)
# ---------------------------------------------------------------------------
def match_message(msg: str, rules: List[Dict[str, Any]]) -> Optional[str]:
    """Return the rule_id of the first matching rule, or None."""
    for rule in rules:
        if rule["_compiled"].search(msg):
            return rule["id"]
    return None


# ---------------------------------------------------------------------------
# Dedup simulator
# ---------------------------------------------------------------------------
class DedupSimulator:
    """Mirrors LogParser dedup logic: (rule_id, node_name) -> last_emitted_ts."""

    def __init__(self, window: float = DEFAULT_DEDUP_WINDOW_SEC):
        self.window = window
        self._cache: Dict[Tuple[str, str], float] = {}

    def should_emit(self, rule_id: str, node_name: str, ts: float) -> bool:
        """Return True if this fault should be emitted (not a duplicate).

        Mirrors LogParser._on_log dedup: cache defaults to 0.0, which works
        in production because time.time() >> 0.  We replicate that by using
        realistic epoch-scale timestamps in our test scenarios.
        """
        key = (rule_id, node_name)
        last = self._cache.get(key, 0.0)
        if ts - last < self.window:
            return False
        self._cache[key] = ts
        return True


# ===================================================================
# PART 1 — Detection accuracy
# ===================================================================

# (message, expected_rule_id_or_None, label)
TRUE_POSITIVES: List[Tuple[str, str, str]] = [
    ("Costmap2D failed to initialize properly", "nav2_costmap_fail", "TP"),
    ("SLAM diverged at frame 142", "slam_diverged", "TP"),
    ("Pose covariance exceeded threshold", "slam_diverged", "TP"),
    ("DDS discovery timeout after 30s", "dds_discovery_lost", "TP"),
    ("Remote participant lost connection", "dds_discovery_lost", "TP"),
    ("TF extrapolation into the future", "transform_timeout", "TP"),
    ("transform timeout waiting for base_link", "transform_timeout", "TP"),
    ("Emergency stop activated by operator", "hardware_estop", "TP"),
    ("E-STOP triggered by bumper sensor", "hardware_estop", "TP"),
]

TRUE_NEGATIVES: List[Tuple[str, Optional[str], str]] = [
    ("Navigation goal reached successfully", None, "TN"),
    ("SLAM initialization complete", None, "TN"),
    ("DDS discovery completed normally", None, "TN"),
    ("Transform lookup succeeded in 0.01s", None, "TN"),
    ("All systems nominal", None, "TN"),
    ("Battery level: 85%", None, "TN"),
    ("Motor temperature within normal range", None, "TN"),
    ("Costmap update cycle took 0.05s", None, "TN"),
    ("SLAM covariance within bounds", None, "TN"),
]

# Near-miss edge cases with expected behaviour documented
EDGE_CASES: List[Tuple[str, Optional[str], str, str]] = [
    # (message, expected, label, reason)
    (
        "costmap failed",
        None,
        "EDGE",
        'No "to initialize" — pattern is "Costmap.*failed to initialize"',
    ),
    (
        "slam has diverged slightly",
        "slam_diverged",
        "EDGE",
        'Case-insensitive, contains "SLAM.*diverged"',
    ),
    (
        "discovery was lost temporarily",
        None,
        "EDGE",
        'No "timeout" and no "participant.*lost"',
    ),
    (
        "estop was disengaged",
        "hardware_estop",
        "EDGE",
        'Contains "estop" literally',
    ),
]


def run_accuracy(rules: List[Dict[str, Any]]) -> dict:
    """Evaluate detection accuracy across TP/TN/edge-case corpus."""
    all_cases = []
    for msg, expected, label in TRUE_POSITIVES:
        all_cases.append((msg, expected, label, ""))
    for msg, expected, label in TRUE_NEGATIVES:
        all_cases.append((msg, expected, label, ""))
    for item in EDGE_CASES:
        all_cases.append(item)

    results_table: List[dict] = []
    # Per-rule counters
    tp_count: Dict[str, int] = defaultdict(int)
    fp_count: Dict[str, int] = defaultdict(int)
    fn_count: Dict[str, int] = defaultdict(int)
    total_correct = 0

    for msg, expected, label, note in all_cases:
        actual = match_message(msg, rules)
        correct = actual == expected
        if correct:
            total_correct += 1

        # Update per-rule stats
        if expected is not None and actual == expected:
            tp_count[expected] += 1
        if expected is not None and actual != expected:
            fn_count[expected] += 1
        if actual is not None and actual != expected:
            fp_count[actual] += 1

        results_table.append(
            {
                "message": msg,
                "expected": expected,
                "actual": actual,
                "correct": correct,
                "label": label,
                "note": note,
            }
        )

    # Compute per-rule precision and recall
    all_rule_ids = sorted({r["id"] for r in rules})
    per_rule: Dict[str, dict] = {}
    for rid in all_rule_ids:
        tp = tp_count[rid]
        fp = fp_count[rid]
        fn = fn_count[rid]
        precision = tp / (tp + fp) if (tp + fp) > 0 else 1.0
        recall = tp / (tp + fn) if (tp + fn) > 0 else 1.0
        per_rule[rid] = {
            "tp": tp, "fp": fp, "fn": fn,
            "precision": round(precision, 4),
            "recall": round(recall, 4),
        }

    overall_accuracy = round(total_correct / len(all_cases), 4) if all_cases else 0.0

    return {
        "total_cases": len(all_cases),
        "total_correct": total_correct,
        "overall_accuracy": overall_accuracy,
        "per_rule": per_rule,
        "details": results_table,
    }


# ===================================================================
# PART 2 — Throughput
# ===================================================================

SAMPLE_NON_MATCHING = [
    "Navigation goal reached successfully",
    "SLAM initialization complete",
    "DDS discovery completed normally",
    "Transform lookup succeeded in 0.01s",
    "All systems nominal",
    "Battery level: 85%",
    "Motor temperature within normal range",
    "Lidar scan received 1024 points",
    "IMU data within calibration range",
    "Wheel odometry updated",
    "Path planner found 12 waypoints",
    "Camera frame captured at 30fps",
    "Joint state published for 6 actuators",
    "Map update completed in 0.03s",
    "Obstacle cleared from local costmap",
]

SAMPLE_MATCHING = [
    "Costmap2D failed to initialize properly",
    "SLAM diverged at frame 142",
    "Pose covariance exceeded threshold",
    "DDS discovery timeout after 30s",
    "Remote participant lost connection",
    "TF extrapolation into the future",
    "transform timeout waiting for base_link",
    "Emergency stop activated by operator",
    "E-STOP triggered by bumper sensor",
]


def run_throughput(rules: List[Dict[str, Any]], n_messages: int = 10_000) -> dict:
    """Generate n_messages and time the regex matching."""
    # Build corpus: ~80% non-matching, ~20% matching
    rng = random.Random(42)
    corpus: List[str] = []
    for _ in range(n_messages):
        if rng.random() < 0.2:
            corpus.append(rng.choice(SAMPLE_MATCHING))
        else:
            corpus.append(rng.choice(SAMPLE_NON_MATCHING))

    # Warm-up
    for msg in corpus[:100]:
        match_message(msg, rules)

    # Timed run
    t0 = time.perf_counter()
    matches = 0
    for msg in corpus:
        if match_message(msg, rules) is not None:
            matches += 1
    elapsed = time.perf_counter() - t0

    msgs_per_sec = n_messages / elapsed if elapsed > 0 else float("inf")

    return {
        "total_messages": n_messages,
        "matching_messages": matches,
        "elapsed_sec": round(elapsed, 6),
        "messages_per_sec": round(msgs_per_sec, 1),
    }


# ===================================================================
# PART 3 — Dedup behavior validation
# ===================================================================

def run_dedup() -> dict:
    """Validate dedup logic with simulated timestamps."""
    window = DEFAULT_DEDUP_WINDOW_SEC
    dedup = DedupSimulator(window=window)
    node_name = "test_node"
    rule_id = "nav2_costmap_fail"

    scenarios: List[dict] = []

    # Use realistic epoch-scale base time (mirrors time.time() in production)
    T0 = 1_700_000_000.0

    # Scenario 1: 10 messages within dedup window (t=T0 .. T0+4.5)
    emitted_in_window = 0
    for i in range(10):
        ts = T0 + i * 0.5  # T0, T0+0.5, ... T0+4.5  (all within 5s of the first)
        if dedup.should_emit(rule_id, node_name, ts):
            emitted_in_window += 1

    scenarios.append({
        "name": "10 messages within 5s dedup window",
        "expected_faults": 1,
        "actual_faults": emitted_in_window,
        "pass": emitted_in_window == 1,
    })

    # Scenario 2: message after window expires
    ts_after = T0 + window + 0.1  # 5.1s after first emission
    emitted_after = 1 if dedup.should_emit(rule_id, node_name, ts_after) else 0

    scenarios.append({
        "name": "Message after dedup window expires",
        "expected_faults": 1,
        "actual_faults": emitted_after,
        "pass": emitted_after == 1,
    })

    # Scenario 3: different node_name should NOT be suppressed
    dedup2 = DedupSimulator(window=window)
    emitted_a = 1 if dedup2.should_emit(rule_id, "node_A", T0) else 0
    emitted_b = 1 if dedup2.should_emit(rule_id, "node_B", T0 + 0.1) else 0

    scenarios.append({
        "name": "Same rule, different nodes within window",
        "expected_faults": 2,
        "actual_faults": emitted_a + emitted_b,
        "pass": (emitted_a + emitted_b) == 2,
    })

    # Scenario 4: different rule_id on same node should NOT be suppressed
    dedup3 = DedupSimulator(window=window)
    e1 = 1 if dedup3.should_emit("rule_a", node_name, T0) else 0
    e2 = 1 if dedup3.should_emit("rule_b", node_name, T0 + 0.1) else 0

    scenarios.append({
        "name": "Different rules, same node within window",
        "expected_faults": 2,
        "actual_faults": e1 + e2,
        "pass": (e1 + e2) == 2,
    })

    all_pass = all(s["pass"] for s in scenarios)
    return {"dedup_window_sec": window, "scenarios": scenarios, "all_pass": all_pass}


# ===================================================================
# Main
# ===================================================================

def print_section(title: str) -> None:
    width = 72
    print()
    print("=" * width)
    print(f"  {title}")
    print("=" * width)


def main() -> None:
    # Load rules
    if not RULES_PATH.exists():
        print(f"ERROR: rules file not found at {RULES_PATH}", file=sys.stderr)
        sys.exit(1)

    rules = load_rules(RULES_PATH)
    print(f"Loaded {len(rules)} rules from {RULES_PATH}")

    # ── Part 1 ──────────────────────────────────────────────────────────
    print_section("Part 1: Detection Accuracy")
    acc = run_accuracy(rules)

    # Print detail table
    header = f"{'Label':5s} {'Correct':7s} {'Expected':<22s} {'Actual':<22s} Message"
    print(header)
    print("-" * len(header) + "-" * 30)
    for row in acc["details"]:
        mark = "OK" if row["correct"] else "FAIL"
        exp = row["expected"] or "(none)"
        act = row["actual"] or "(none)"
        msg_short = row["message"][:50]
        print(f"{row['label']:5s} {mark:7s} {exp:<22s} {act:<22s} {msg_short}")
        if row.get("note"):
            print(f"      NOTE: {row['note']}")

    print(f"\nOverall accuracy: {acc['total_correct']}/{acc['total_cases']}"
          f" = {acc['overall_accuracy']:.2%}")

    print("\nPer-rule precision / recall:")
    print(f"  {'Rule ID':<22s} {'TP':>3s} {'FP':>3s} {'FN':>3s} {'Prec':>6s} {'Recall':>6s}")
    for rid, stats in sorted(acc["per_rule"].items()):
        print(f"  {rid:<22s} {stats['tp']:3d} {stats['fp']:3d} {stats['fn']:3d}"
              f" {stats['precision']:6.2%} {stats['recall']:6.2%}")

    # ── Part 2 ──────────────────────────────────────────────────────────
    print_section("Part 2: Throughput")
    tp = run_throughput(rules, n_messages=10_000)
    print(f"Messages processed : {tp['total_messages']:,}")
    print(f"Matching messages  : {tp['matching_messages']:,}")
    print(f"Elapsed            : {tp['elapsed_sec']:.6f} s")
    print(f"Throughput         : {tp['messages_per_sec']:,.1f} msg/s")

    # ── Part 3 ──────────────────────────────────────────────────────────
    print_section("Part 3: Dedup Behavior Validation")
    dd = run_dedup()
    for sc in dd["scenarios"]:
        status = "PASS" if sc["pass"] else "FAIL"
        print(f"  [{status}] {sc['name']}: "
              f"expected={sc['expected_faults']}, actual={sc['actual_faults']}")
    print(f"\n  All dedup tests pass: {dd['all_pass']}")

    # ── Rule coverage summary ──────────────────────────────────────────
    print_section("Rule Coverage Summary")
    for rule in rules:
        rid = rule["id"]
        stats = acc["per_rule"].get(rid, {})
        tested = stats.get("tp", 0) + stats.get("fn", 0)
        print(f"  {rid:<22s}  test cases: {tested}")

    # ── Save JSON ──────────────────────────────────────────────────────
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    output = {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "rules_file": str(RULES_PATH),
        "num_rules": len(rules),
        "accuracy": acc,
        "throughput": tp,
        "dedup": dd,
    }
    with open(RESULTS_FILE, "w") as f:
        json.dump(output, f, indent=2)
    print(f"\nResults saved to {RESULTS_FILE}")


if __name__ == "__main__":
    main()
