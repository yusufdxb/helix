#!/usr/bin/env python3
"""Validate deployment inputs without starting ROS nodes or contacting hardware."""
import argparse
import ast
import importlib
import xml.etree.ElementTree as ET
from pathlib import Path


# Every package the GO2 deployment builds. A package added or removed in src/
# must be added or removed here, so the bundle never ships an unexpected set.
EXPECTED_PACKAGES = frozenset({
    "helix_msgs", "helix_core", "helix_adapter", "helix_diagnosis",
    "helix_recovery", "helix_explanation", "helix_arbiter", "helix_arbiter_cpp",
    "helix_bringup", "helix_sensing_cpp",
})


def require(condition, message):
    if not condition:
        raise ValueError(message)


def check_source(root):
    packages = {}
    for manifest in sorted((root / "src").glob("*/package.xml")):
        tree = ET.parse(manifest).getroot()
        name = tree.findtext("name")
        require(name and name == manifest.parent.name, f"Invalid package name in {manifest}")
        require(name not in packages, f"Duplicate package {name}")
        packages[name] = manifest.parent
    require(set(packages) == EXPECTED_PACKAGES,
            f"Unexpected HELIX package set: missing {sorted(EXPECTED_PACKAGES - set(packages))}, "
            f"extra {sorted(set(packages) - EXPECTED_PACKAGES)}")
    for path in (root / "src").glob("*/launch/*.py"):
        ast.parse(path.read_text(), filename=str(path))
    require((root / "src/helix_bringup/launch/helix_go2.launch.py").is_file(), "Missing GO2 preparation launch")
    ET.parse(root / "cyclonedds_loopback.xml")
    for path in (root / "src").glob("*/config/*.xml"):
        ET.parse(path)
    # YAML is checked when available, including on a minimal Humble installation.
    try:
        import yaml
    except ImportError:
        print("UNVERIFIED: YAML parsing (install python3-yaml or run --check on Humble)")
    else:
        for path in (root / "src").glob("*/config/*.yaml"):
            require(isinstance(yaml.safe_load(path.read_text()), dict), f"Invalid YAML: {path}")
        p = yaml.safe_load((root / "src/helix_bringup/config/helix_params.yaml").read_text())
        p = p["helix_anomaly_detector"]["ros__parameters"]
        require((p["zscore_threshold"], p["window_size"]) == (4.0, 60), "Unexpected shipped detector parameters")
    print("OK: source manifests, launch syntax, configuration inputs")
    return packages


def check_ros(packages, installed):
    import os
    import sys
    require(os.environ.get("ROS_DISTRO") == "humble", "Source ROS 2 Humble")
    require(sys.version_info[:2] == (3, 10), "Humble requires the system Python 3.10 ABI")
    for name in ("rclpy", "launch", "launch_ros", "yaml", "requests"):
        importlib.import_module(name)
    from ament_index_python.packages import get_package_share_directory
    for name in ("rmw_cyclonedds_cpp", "twist_mux"):
        get_package_share_directory(name)
    try:
        from unitree_api.msg import Request, Response
    except ImportError:
        print("UNVERIFIED: unitree_api absent; dry_run works, real Sport API requires a target-native underlay")
    else:
        request = Request()
        request.header.identity.api_id = 1003
        request.header.lease.id = 0
        request.header.policy.priority = 0
        require(hasattr(Response().header.status, "code"), "Missing Sport response status code")
        print("OK: unitree_api Request/Response schema (no publisher created)")
    if installed:
        from launch.launch_description_sources import PythonLaunchDescriptionSource
        for name in packages:
            get_package_share_directory(name)
        for name in ("helix_core", "helix_adapter", "helix_diagnosis", "helix_recovery",
                     "helix_explanation", "helix_arbiter"):
            importlib.import_module(name)
        from helix_msgs.msg import FaultEvent, HelixHold, RecoveryAction
        for message_type in (FaultEvent, HelixHold, RecoveryAction):
            message_type()
        for name in ("helix_bringup", "helix_sensing_cpp"):
            share = Path(get_package_share_directory(name))
            for path in sorted((share / "launch").glob("*.launch.py")):
                src = PythonLaunchDescriptionSource(str(path))
                require(src.try_get_launch_description_without_context() is not None, f"Invalid launch: {path}")
                print(f"OK: installed launch {name}/{path.name}")
    print("OK: Humble Python ABI, imports and DDS dependencies")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--ros", action="store_true")
    parser.add_argument("--installed", action="store_true")
    args = parser.parse_args()
    packages = check_source(args.source)
    if args.ros:
        check_ros(packages, args.installed)


if __name__ == "__main__":
    main()
