"""Fixed native supervision + last-hop guard + FakeGo2, localhost simulation only.

This profile never starts a physical controller publisher or a communication
relay. A test driver owns the four relay links and explicit operator actions.
The normal helix_native.launch.py defaults and graph are unaffected.
"""
import importlib.util
import os
from pathlib import Path

from launch.actions import OpaqueFunction

from launch import LaunchDescription


def simulation_actions(context):
    """Reject unsafe environments and overrides before constructing processes."""
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise ValueError('guard simulation requires ROS_LOCALHOST_ONLY=1')
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_cyclonedds_cpp':
        raise ValueError('guard simulation requires RMW_IMPLEMENTATION=rmw_cyclonedds_cpp')
    domain = os.environ.get('ROS_DOMAIN_ID', '')
    if not domain.isdecimal() or not 1 <= int(domain) <= 232:
        raise ValueError('guard simulation requires an explicit isolated ROS_DOMAIN_ID (1..232)')
    overrides = set(context.launch_configurations) - {'ros_namespace'}
    if overrides or context.launch_configurations.get('ros_namespace', '') not in ('', '/'):
        raise ValueError('guard simulation has a fixed graph and accepts no launch overrides')
    path = Path(__file__).with_name('helix_native.launch.py')
    spec = importlib.util.spec_from_file_location('helix_native_guard_profile', path)
    native = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(native)
    if os.environ.get('CYCLONEDDS_URI', native.GUARD_SIM_CYCLONE_URI) != native.GUARD_SIM_CYCLONE_URI:
        raise ValueError('guard simulation accepts only its fixed loopback Cyclone configuration')
    os.environ['CYCLONEDDS_URI'] = native.GUARD_SIM_CYCLONE_URI
    return native.guard_sim_actions()


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=simulation_actions)])
