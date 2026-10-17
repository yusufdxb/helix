"""Expand helix_native.launch.py without starting processes or DDS.

The native profile must stay non-actuating by default, run no Python node,
and hand the supervisor the same heartbeat set the launch actually starts.
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest
import yaml

pytest.importorskip("launch.actions", reason="ROS 2 launch is not installed")
pytest.importorskip("launch_ros", reason="ROS 2 launch_ros is not installed")

import ament_index_python.packages  # noqa: E402
from launch.actions import DeclareLaunchArgument, OpaqueFunction  # noqa: E402
from launch_ros.actions import Node  # noqa: E402
from launch_ros.utilities import evaluate_parameters  # noqa: E402

from launch import LaunchContext, LaunchDescription  # noqa: E402

SOURCE = Path(__file__).resolve().parents[2]
NATIVE_PACKAGES = {
    "helix_sensing_cpp", "helix_supervisor_cpp", "helix_arbiter_cpp", "helix_go2_sink_cpp",
}


@pytest.fixture
def native(monkeypatch):
    monkeypatch.setattr(
        ament_index_python.packages,
        "get_package_share_directory",
        lambda package: str(SOURCE / package),
    )
    path = SOURCE / "helix_bringup" / "launch" / "helix_native.launch.py"
    spec = importlib.util.spec_from_file_location("helix_native_launch_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _expand(description, context):
    nodes = []
    for action in description.entities:
        if isinstance(action, Node):
            nodes.append(action)
        elif isinstance(action, OpaqueFunction):
            nodes.extend(_expand(LaunchDescription(action.execute(context) or []), context))
        elif isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return nodes


def _graph(module, **overrides):
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    nodes = _expand(module.generate_launch_description(), context)
    return {node.node_executable: node for node in nodes}, context


def _parameters(node, context):
    merged = {}
    for item in evaluate_parameters(context, node._Node__parameters):
        if isinstance(item, dict):
            merged.update(item)
        else:
            loaded = yaml.safe_load(Path(item).read_text())
            entry = loaded.get(node.node_executable, loaded.get("/**", {}))
            merged.update(entry.get("ros__parameters", {}))
    return merged


def test_defaults_are_non_actuating(native):
    nodes, ctx = _graph(native)
    supervisor = _parameters(nodes["helix_supervisor"], ctx)
    assert supervisor["actuation_enabled"] is False
    assert supervisor["auto_resume"] is False
    arbiter = _parameters(nodes["helix_arbiter"], ctx)
    assert arbiter["output_topic"] == "/helix_dry/cmd_vel"
    assert arbiter["sources"]["nav"]["topic"].startswith("/helix_dry/")
    assert arbiter["sources"]["teleop"]["topic"].startswith("/helix_dry/")
    sink = _parameters(nodes["helix_go2_sport_sink"], ctx)
    assert sink["mode"] == "dry_run"
    assert sink["input_topic"] == arbiter["output_topic"]


def test_arming_the_sink_alone_does_not_arm_the_supervisor(native):
    nodes, ctx = _graph(native, sink_mode="armed")
    assert _parameters(nodes["helix_supervisor"], ctx)["actuation_enabled"] is False
    assert _parameters(nodes["helix_arbiter"], ctx)["output_topic"] == "/helix_dry/cmd_vel"


def test_profile_runs_no_python_node(native):
    nodes, _ = _graph(native)
    assert {n.node_package for n in nodes.values()} <= NATIVE_PACKAGES
    assert len(nodes) == 6


def test_python_sink_backend_is_the_only_python_node(native):
    nodes, _ = _graph(native, sink_backend="python")
    python = {n.node_executable for n in nodes.values() if n.node_package not in NATIVE_PACKAGES}
    assert python == {"helix_go2_sport_sink"}


def test_supervisor_expects_exactly_the_heartbeating_nodes(native):
    nodes, ctx = _graph(native)
    expected = _parameters(nodes["helix_supervisor"], ctx)["expected_nodes"]
    assert set(expected) == {"helix_topic_rate_monitor", "helix_anomaly_detector",
                             "helix_log_parser"}
    assert set(expected) <= set(nodes)
    assert list(_parameters(nodes["helix_supervisor"], ctx)["critical_nodes"]) == ["*"]


def test_detector_runs_the_round2_config(native):
    nodes, ctx = _graph(native)
    detector = _parameters(nodes["helix_anomaly_detector"], ctx)
    for key, value in native.DETECTOR_OVERRIDES.items():
        assert detector[key] == value, key
    assert detector["freeze_baseline"] is True
    rates = _parameters(nodes["helix_topic_rate_monitor"], ctx)
    assert rates["stale_after_s"] == 1.0
    assert "/utlidar/cloud" in rates["topics"]


def test_unknown_sink_mode_is_refused(native):
    with pytest.raises(ValueError, match="sink_mode"):
        _graph(native, sink_mode="damp")
