"""Launch file: full HELIX closed loop against sim (or real) GO2."""
import os

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    twist_mux_config = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'twist_mux.yaml')

    return LaunchDescription([
        # Sensing tier (existing)
        LifecycleNode(package='helix_core', executable='helix_anomaly_detector',
                      name='helix_anomaly_detector', namespace=''),
        LifecycleNode(package='helix_core', executable='helix_heartbeat_monitor',
                      name='helix_heartbeat_monitor', namespace=''),
        LifecycleNode(package='helix_core', executable='helix_log_parser',
                      name='helix_log_parser', namespace=''),
        # New tiers
        LifecycleNode(package='helix_diagnosis', executable='helix_context_buffer',
                      name='helix_context_buffer', namespace=''),
        LifecycleNode(package='helix_diagnosis', executable='helix_diagnosis_node',
                      name='helix_diagnosis_node', namespace=''),
        LifecycleNode(package='helix_recovery', executable='helix_recovery_node',
                      name='helix_recovery_node', namespace='',
                      parameters=[{'enabled': True, 'cooldown_seconds': 5.0}]),
        Node(package='helix_explanation', executable='helix_llm_explainer',
             name='helix_llm_explainer'),
        # Arbitration
        Node(package='twist_mux', executable='twist_mux',
             name='twist_mux', parameters=[twist_mux_config]),
    ])
