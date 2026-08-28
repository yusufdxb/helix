"""
Session-scoped rclpy init/shutdown for all HELIX core tests.

All test modules in this directory share this fixture automatically (autouse=True).
rclpy.init() is called exactly once; rclpy.shutdown() is called after all tests finish.
"""
import os

# Test isolation. colcon runs package test binaries CONCURRENTLY and every
# suite used to share DDS domain 0, so publishers from one package leaked
# into another package's assertions: helix_core's test_anomaly_detection
# failed on a 'nodeA/cpu_pct' FaultEvent that a sibling suite had published,
# and helix_adapter's rate-metric test saw foreign traffic the same way.
# Each package therefore gets its own domain. This is set at import time,
# before rclpy.init() reads it, and is forced rather than defaulted so a
# developer's exported ROS_DOMAIN_ID cannot silently reintroduce crosstalk
# (or point a test run at a live robot).
os.environ["ROS_DOMAIN_ID"] = "81"

import pytest
import rclpy


@pytest.fixture(scope="session", autouse=True)
def rclpy_session():
    """Initialize rclpy once for the entire test session."""
    rclpy.init()
    yield
    rclpy.shutdown()
