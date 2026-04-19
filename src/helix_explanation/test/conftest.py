"""Session-scoped rclpy init/shutdown for all helix_explanation tests."""
import pytest
import rclpy


@pytest.fixture(scope="session", autouse=True)
def rclpy_session():
    rclpy.init()
    yield
    rclpy.shutdown()
