"""Expand the real GO2 launch graph without starting processes or DDS.

These checks protect the separation between the preparation profile and
the real command path. They use ROS launch substitution evaluation rather
than matching launch-file source text.
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest
import yaml

pytest.importorskip("launch.actions", reason="ROS 2 launch is not installed")
pytest.importorskip("launch_ros", reason="ROS 2 launch_ros is not installed")

import ament_index_python.packages  # noqa: E402
from launch.actions import (  # noqa: E402
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch_ros.actions import Node  # noqa: E402
from launch_ros.utilities import evaluate_parameters  # noqa: E402

from launch import LaunchContext, LaunchDescription  # noqa: E402

SOURCE = Path(__file__).resolve().parents[2]


@pytest.fixture
def prep_graph(monkeypatch):
    """Load source launches, so these checks also run before colcon install."""
    monkeypatch.setattr(
        ament_index_python.packages,
        "get_package_share_directory",
        lambda package: str(SOURCE / package),
    )
    path = SOURCE / "helix_bringup" / "launch" / "helix_go2.launch.py"
    spec = importlib.util.spec_from_file_location("helix_go2_launch_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _expand(description, context):
    """Apply launch configuration actions; collect nodes without executing them."""
    nodes = []
    for action in description.entities:
        if action.condition is not None and not action.condition.evaluate(context):
            continue
        if isinstance(action, Node):
            nodes.append(action)
        elif isinstance(action, OpaqueFunction):
            # Backend selection runs at launch time; evaluate it here so the
            # chosen node is inspected rather than skipped.
            produced = action.execute(context) or []
            nodes.extend(_expand(LaunchDescription(produced), context))
        elif isinstance(action, IncludeLaunchDescription):
            for entity in action.execute(context):
                if isinstance(entity, SetLaunchConfiguration):
                    entity.execute(context)
                elif isinstance(entity, LaunchDescription):
                    nodes.extend(_expand(entity, context))
        elif isinstance(action, (DeclareLaunchArgument, SetLaunchConfiguration)):
            action.execute(context)
    return nodes


def _parameters(node, context):
    # Humble Node has no public parameter getter. Evaluate its normalized
    # parameter list through the same utility Node uses at execution time.
    merged = {}
    for item in evaluate_parameters(context, node._Node__parameters):
        if isinstance(item, dict):
            merged.update(item)
        else:
            loaded = yaml.safe_load(Path(item).read_text())
            entry = loaded.get(node.node_executable, loaded.get("/**", {}))
            merged.update(entry.get("ros__parameters", {}))
    return merged


def test_prep_launch_overrides_unsafe_outer_arguments(prep_graph):
    context = LaunchContext()
    context.launch_configurations.update({
        "sim_mode": "true",
        "enable_twist_mux": "true",
        "auto_activate_recovery": "true",
        "recovery_enabled": "true",
        "llm_enabled": "true",
        "cmd_vel_out": "/cmd_vel",
        "mode": "armed",
    })
    nodes = _expand(prep_graph, context)
    packages = [node.node_package for node in nodes]
    assert "twist_mux" not in packages
    for key in ("sim_mode", "enable_twist_mux", "auto_activate_recovery",
                "recovery_enabled", "llm_enabled"):
        assert context.launch_configurations[key] == "false"
    assert context.launch_configurations["cmd_vel_out"] == "/helix_dry/cmd_vel"
    assert not any(isinstance(action, DeclareLaunchArgument) for action in prep_graph.entities)


def test_prep_graph_has_only_isolated_command_route_and_dry_sink(prep_graph):
    context = LaunchContext()
    nodes = _expand(prep_graph, context)
    by_executable = {node.node_executable: node for node in nodes}

    recovery = _parameters(by_executable["helix_recovery_node"], context)
    assert recovery["enabled"] is False
    assert recovery["publish_legacy_cmd_vel"] is False

    arbiter = _parameters(by_executable["helix_arbiter"], context)
    assert arbiter["output_topic"] == "/helix_dry/cmd_vel"
    assert arbiter["sources"]["teleop"]["topic"] == "/helix_dry/teleop/cmd_vel"
    assert arbiter["sources"]["nav"]["topic"] == "/helix_dry/nav/cmd_vel"

    sink = _parameters(by_executable["helix_go2_sport_sink"], context)
    assert sink["mode"] == "dry_run"
    assert sink["input_topic"] == arbiter["output_topic"]


def test_prep_graph_uses_real_sensor_topics_and_shipped_detector(prep_graph):
    context = LaunchContext()
    nodes = _expand(prep_graph, context)
    by_executable = {node.node_executable: node for node in nodes}
    rates = _parameters(by_executable["helix_topic_rate_monitor"], context)
    assert rates["sim_mode"] is False
    assert "/utlidar/cloud" in rates["topics"]
    assert "/utlidar/cloud_throttled" not in rates["topics"]
    assert "/gnss" in rates["topics"]
    assert "/multiplestate" in rates["topics"]
    detector = _parameters(by_executable["helix_anomaly_detector"], context)
    assert detector == yaml.safe_load(
        (SOURCE / "helix_bringup" / "config" / "helix_params.yaml").read_text()
    )["helix_anomaly_detector"]["ros__parameters"]
