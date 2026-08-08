"""End-to-end HELIX closure test against a running Isaac Sim GO2 bridge.

This is the only test that claims PHYSICAL closure: the robot was moving, the
loop stopped it, and odometry proves it. That claim needs a body to move, so
the test requires the Isaac Sim GO2 bridge to be publishing /utlidar/cloud and
/utlidar/robot_odom before it runs, and skips loudly when it is not. It is not
weakened to pass on a bare workstation, because a green run here is what gates
the next hardware session.
"""

import subprocess

import pytest

from scripts.validate_closed_loop_bag import evaluate, load_records

REQUIRED_SIM_TOPICS = ("/utlidar/cloud", "/utlidar/robot_odom")


def publisher_count(topic: str) -> int:
    """Publishers currently advertising `topic`, or 0 if it cannot be queried."""
    try:
        result = subprocess.run(
            ["ros2", "topic", "info", topic],
            capture_output=True, text=True, timeout=15,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return 0
    for line in result.stdout.splitlines():
        if line.startswith("Publisher count:"):
            return int(line.split(":", 1)[1])
    return 0


@pytest.fixture(scope="module", autouse=True)
def require_sim_bridge():
    missing = [topic for topic in REQUIRED_SIM_TOPICS if publisher_count(topic) == 0]
    if missing:
        pytest.skip(
            "Isaac Sim GO2 bridge is not publishing "
            + ", ".join(missing)
            + ". Physical closure cannot be proven without a moving body; start the "
              "bridge per docs/sim_launch_recipe.md and re-run.",
            allow_module_level=False,
        )


SCENARIO_CMD = [
    "python3",
    "scripts/sim_faults/run_closed_loop_scenario.py",
    "--duration",
    "75",
    "--schedule",
    "20:10,20:0,30:10",
    "--nav-speed",
    "0.20",
]


@pytest.fixture(scope="module")
def scenario_records(tmp_path_factory):
    output = tmp_path_factory.mktemp("sim_run")
    subprocess.run(
        SCENARIO_CMD + ["--artifact-dir", str(output)],
        check=True,
        timeout=180,
    )
    return load_records(output / "bag")


def test_simulation_closes_the_same_evidence_gate_as_hardware(scenario_records):
    result = evaluate(scenario_records)
    failed = [check for check in result["checks"] if not check["passed"]]
    assert result["passed"], failed


def test_simulation_captures_a_stale_topic_fault(scenario_records):
    stale = [
        fault
        for fault in scenario_records["faults"]
        if fault["violation_type"] == "stale" and "utlidar" in fault["metric_name"]
    ]
    assert stale, scenario_records["faults"]
