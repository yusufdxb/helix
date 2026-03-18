"""
Tests for ActionExecutor.

Uses AsyncMock for subprocess and a live rclpy node for pub/sub verification.
"""
import asyncio
import time
import threading
import pytest
import rclpy
from unittest.mock import AsyncMock, patch, MagicMock
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String, Bool


@pytest.fixture(scope="module")
def exec_node():
    """Create a plain rclpy node that ActionExecutor will publish on."""
    node = rclpy.create_node("test_action_exec_node")
    yield node
    node.destroy_node()


@pytest.fixture(scope="module")
def executor(exec_node):
    """Create an ActionExecutor bound to the test node."""
    from helix_recovery.action_executor import ActionExecutor
    return ActionExecutor(node=exec_node)


def _spin_for(node, seconds: float) -> None:
    """Spin a single node in background for given duration."""
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    deadline = time.time() + seconds
    while time.time() < deadline:
        ex.spin_once(timeout_sec=0.05)
    ex.remove_node(node)


# ── restart_node ─────────────────────────────────────────────────────────────

def test_restart_node_calls_lifecycle_commands(executor):
    """restart_node must issue all four lifecycle transitions in order."""
    call_args_list = []

    async def fake_create_subprocess(*args, **kwargs):
        call_args_list.append(args)
        proc = MagicMock()
        proc.returncode = 0
        proc.communicate = AsyncMock(return_value=(b"", b""))
        return proc

    with patch(
        "asyncio.create_subprocess_exec",
        side_effect=fake_create_subprocess,
    ):
        success, detail, duration = asyncio.run(
            executor.restart_node("fake_nav_node")
        )

    assert success is True
    assert duration >= 0.0
    # Four subprocess calls: deactivate, cleanup, configure, activate
    assert len(call_args_list) == 4
    # ros2 lifecycle set <node_name> <transition> → transition is at index 4
    transitions = [args[4] for args in call_args_list]
    assert transitions == ["deactivate", "cleanup", "configure", "activate"]


def test_restart_node_fails_on_nonzero_exit(executor):
    """restart_node returns success=False if any subprocess returns non-zero."""
    async def fake_fail(*args, **kwargs):
        proc = MagicMock()
        proc.returncode = 1
        proc.communicate = AsyncMock(return_value=(b"", b"error"))
        return proc

    with patch("asyncio.create_subprocess_exec", side_effect=fake_fail):
        success, detail, _ = asyncio.run(executor.restart_node("bad_node"))

    assert success is False
    assert "error" in detail.lower() or "failed" in detail.lower()


# ── activate_fallback_topic ──────────────────────────────────────────────────

def test_activate_fallback_publishes_directive(exec_node, executor):
    """activate_fallback_topic must publish to /helix/fallback_directive."""
    received = []
    sub_node = rclpy.create_node("test_fallback_sub")
    sub_node.create_subscription(
        String,
        "/helix/fallback_directive",
        lambda msg: received.append(msg.data),
        10,
    )

    ex = SingleThreadedExecutor()
    ex.add_node(exec_node)
    ex.add_node(sub_node)

    spin_thread = threading.Thread(
        target=lambda: [ex.spin_once(timeout_sec=0.05) for _ in range(40)],
        daemon=True,
    )
    spin_thread.start()

    success, detail, _ = asyncio.run(
        executor.activate_fallback_topic(
            "camera_node",
            {"camera_node": "/camera/depth/image_raw_fallback"},
        )
    )
    spin_thread.join()
    sub_node.destroy_node()

    assert success is True
    assert len(received) >= 1
    assert "camera_node:/camera/depth/image_raw_fallback" in received[0]


# ── activate_safe_mode ───────────────────────────────────────────────────────

def test_activate_safe_mode_publishes(exec_node, executor):
    """activate_safe_mode must publish True to /helix/safe_mode_active."""
    received_bool = []
    sub_node = rclpy.create_node("test_safemode_sub")
    sub_node.create_subscription(
        Bool,
        "/helix/safe_mode_active",
        lambda msg: received_bool.append(msg.data),
        10,
    )

    ex = SingleThreadedExecutor()
    ex.add_node(exec_node)
    ex.add_node(sub_node)

    spin_thread = threading.Thread(
        target=lambda: [ex.spin_once(timeout_sec=0.05) for _ in range(40)],
        daemon=True,
    )
    spin_thread.start()

    success, _, _ = asyncio.run(executor.activate_safe_mode())
    spin_thread.join()
    sub_node.destroy_node()

    assert success is True
    assert True in received_bool


# ── reduce_velocity_limits ───────────────────────────────────────────────────

def test_reduce_velocity_limits_calls_param_get_and_set(executor):
    """reduce_velocity_limits should call ros2 param get then set for each param."""
    call_log = []

    async def fake_subproc(*args, **kwargs):
        call_log.append(list(args))
        proc = MagicMock()
        proc.returncode = 0
        # Simulate 'ros2 param get' returning "Double value is: 0.5"
        if "get" in args:
            proc.communicate = AsyncMock(return_value=(b"Double value is: 0.5", b""))
        else:
            proc.communicate = AsyncMock(return_value=(b"Set parameter successful", b""))
        return proc

    with patch("asyncio.create_subprocess_exec", side_effect=fake_subproc):
        loop = asyncio.new_event_loop()
        success, detail, _ = loop.run_until_complete(
            executor.reduce_velocity_limits("controller_server", factor=0.5)
        )
        loop.close()

    assert success is True
    # Should have called get + set for each of max_vel_x and max_vel_theta
    assert len(call_log) == 4  # 2 gets + 2 sets


# ── reconfigure_dds ──────────────────────────────────────────────────────────

def test_reconfigure_dds_publishes_request(exec_node, executor):
    """reconfigure_dds must publish to /helix/dds_reconfigure_request."""
    received = []
    sub_node = rclpy.create_node("test_dds_sub")
    sub_node.create_subscription(
        String,
        "/helix/dds_reconfigure_request",
        lambda msg: received.append(msg.data),
        10,
    )

    ex = SingleThreadedExecutor()
    ex.add_node(exec_node)
    ex.add_node(sub_node)
    spin_thread = threading.Thread(
        target=lambda: [ex.spin_once(timeout_sec=0.05) for _ in range(40)],
        daemon=True,
    )
    spin_thread.start()

    loop = asyncio.new_event_loop()
    success, _, _ = loop.run_until_complete(executor.reconfigure_dds())
    loop.close()

    spin_thread.join()
    sub_node.destroy_node()

    assert success is True
    assert len(received) >= 1


# ── standalone_mode ──────────────────────────────────────────────────────────

def test_standalone_mode_publishes(exec_node, executor):
    """standalone_mode must publish True to /helix/standalone_active."""
    received = []
    sub_node = rclpy.create_node("test_standalone_sub")
    sub_node.create_subscription(
        Bool,
        "/helix/standalone_active",
        lambda msg: received.append(msg.data),
        10,
    )

    ex = SingleThreadedExecutor()
    ex.add_node(exec_node)
    ex.add_node(sub_node)
    spin_thread = threading.Thread(
        target=lambda: [ex.spin_once(timeout_sec=0.05) for _ in range(40)],
        daemon=True,
    )
    spin_thread.start()

    loop = asyncio.new_event_loop()
    success, _, _ = loop.run_until_complete(executor.standalone_mode())
    loop.close()

    spin_thread.join()
    sub_node.destroy_node()

    assert success is True
    assert True in received


# ── trigger_global_localization + reinit_from_waypoint ───────────────────────
# These require external ROS topics (/amcl_pose, waypoint topic) and are
# covered by the integration smoke test (helix_full.launch.py + fault_injector).
# Unit test only checks they return the expected tuple structure:

def test_trigger_global_localization_returns_tuple(exec_node, executor):
    """trigger_global_localization should return (bool, str, float) within timeout."""
    loop = asyncio.new_event_loop()
    # With no /amcl_pose publisher, this should time out and return False
    result = loop.run_until_complete(executor.trigger_global_localization())
    loop.close()
    assert isinstance(result, tuple) and len(result) == 3
    assert isinstance(result[0], bool)
    assert isinstance(result[1], str)
    assert isinstance(result[2], float)
