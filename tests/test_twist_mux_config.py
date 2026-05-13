"""Static check of the twist_mux YAML the closedloop launch consumes.

This test runs in the no-ROS CI job (it only needs PyYAML), so the
priority-arbitration safety invariant is asserted on every push without
needing to spin twist_mux or rclpy.

The full sim-only harness with a model + a real-binary integration test
lives in ``src/helix_bringup/test/`` and runs in the ROS CI job.
"""
from __future__ import annotations

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
BRINGUP_YAML = REPO_ROOT / "src" / "helix_bringup" / "config" / "twist_mux.yaml"


def _load_topics() -> dict:
    doc = yaml.safe_load(BRINGUP_YAML.read_text())
    return doc["twist_mux"]["ros__parameters"]["topics"]


def test_bringup_yaml_exists():
    assert BRINGUP_YAML.is_file(), f"missing {BRINGUP_YAML}"


def test_helix_priority_invariant():
    """teleop > helix_recovery > navigation. Anything else lets autonomy
    or a stuck teleop preempt a recovery-tier STOP, which is unsafe."""
    topics = _load_topics()
    assert set(topics) == {"teleop", "helix_recovery", "navigation"}, (
        f"twist_mux YAML has unexpected input set: {set(topics)}"
    )
    teleop = int(topics["teleop"]["priority"])
    helix = int(topics["helix_recovery"]["priority"])
    nav = int(topics["navigation"]["priority"])
    assert teleop > helix > nav, (
        f"twist_mux priority order broken: teleop={teleop}, "
        f"helix={helix}, nav={nav}. Required: teleop > helix > nav."
    )


def test_helix_timeouts_are_uniform_and_short():
    """All three inputs share the same timeout. If we ever desync them
    we want to know explicitly; right now 0.5 s matches the LAB_CARD."""
    topics = _load_topics()
    timeouts = {name: float(cfg["timeout"]) for name, cfg in topics.items()}
    distinct = set(timeouts.values())
    assert distinct == {0.5}, (
        f"twist_mux timeouts not uniform 0.5 s: {timeouts}"
    )


def test_topic_names_match_documented_wiring():
    topics = _load_topics()
    assert topics["teleop"]["topic"] == "/teleop/cmd_vel"
    assert topics["helix_recovery"]["topic"] == "/helix/cmd_vel"
    assert topics["navigation"]["topic"] == "/nav/cmd_vel"
