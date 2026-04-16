"""Session-scoped rclpy init/shutdown for helix_adapter tests.

Defensive against co-running with sibling packages' conftest (helix_core also
declares a session-scoped rclpy fixture). If rclpy is already initialized by
another conftest, we don't touch it.
"""
import pytest
import rclpy


@pytest.fixture(scope="session", autouse=True)
def rclpy_session():
    owns = not rclpy.ok()
    if owns:
        rclpy.init()
    yield
    if owns and rclpy.ok():
        rclpy.shutdown()
