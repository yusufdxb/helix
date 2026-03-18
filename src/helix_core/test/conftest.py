"""
Session-scoped rclpy init/shutdown for all HELIX core tests.

All test modules in this directory share this fixture automatically (autouse=True).
rclpy.init() is called exactly once; rclpy.shutdown() is called after all tests finish.
"""
import pytest
import rclpy


@pytest.fixture(scope="session", autouse=True)
def rclpy_session():
    """Initialize rclpy once for the entire test session."""
    rclpy.init()
    yield
    rclpy.shutdown()
