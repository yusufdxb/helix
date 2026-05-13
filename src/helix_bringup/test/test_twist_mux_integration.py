"""Integration test: real twist_mux binary, synthetic publishers, sim-only.

This test starts the actual ``twist_mux`` ROS node with the production
HELIX config (``src/helix_bringup/config/twist_mux.yaml``), publishes
synthetic ``geometry_msgs/Twist`` streams on the three configured input
topics, and asserts that the muxed output topic carries the expected
winner.

It runs only when:
  * rclpy is importable (ROS sourced)
  * ros-humble-twist-mux is installed (executable on PATH)
  * geometry_msgs is importable

When any of those is missing the test self-skips, so it is safe to run
in the no-ROS pytest job (it skips) and meaningful in the ROS pytest job
(it actually starts twist_mux).

NO HARDWARE. NO GO2. NO Isaac Sim. Just twist_mux + rclpy.
"""
from __future__ import annotations

import os
import shutil
import subprocess
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy", reason="rclpy not available (ROS not sourced)")
geometry_msgs = pytest.importorskip("geometry_msgs.msg", reason="geometry_msgs not available")

from geometry_msgs.msg import Twist  # noqa: E402  (after importorskip)

TWIST_MUX_YAML = (
    Path(__file__).resolve().parent.parent / "config" / "twist_mux.yaml"
)


def _twist_mux_available() -> bool:
    """Return True iff the twist_mux executable is reachable.

    The binary lives in ``lib/twist_mux/twist_mux`` after a colcon-style
    install (not on PATH), so ``shutil.which`` is unreliable. We trust
    ``ros2 pkg executables twist_mux`` instead.
    """
    if shutil.which("twist_mux") is not None:
        return True
    if shutil.which("ros2") is None:
        return False
    try:
        out = subprocess.check_output(
            ["ros2", "pkg", "executables", "twist_mux"],
            stderr=subprocess.STDOUT,
            timeout=10,
        ).decode(errors="replace")
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired, FileNotFoundError):
        return False
    return "twist_mux twist_mux" in out


@pytest.fixture(scope="module")
def rclpy_module():
    if not _twist_mux_available():
        pytest.skip("twist_mux binary not on PATH; skipping integration test")
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def twist_mux_proc(rclpy_module):
    """Start the real twist_mux node with the HELIX YAML, wait for it,
    tear down on exit. Output is captured so test output stays clean."""
    env = os.environ.copy()
    # twist_mux subprocess MUST run on the same ROS_DOMAIN_ID as the
    # test's own rclpy.init(); we let it inherit from the parent. To
    # avoid colliding with whatever's running on the host's default
    # domain, run pytest with ROS_DOMAIN_ID=77 (or any unused value)
    # exported in the parent shell.
    proc = subprocess.Popen(
        [
            "ros2", "run", "twist_mux", "twist_mux",
            "--ros-args",
            "--params-file", str(TWIST_MUX_YAML),
            "--remap", "cmd_vel_out:=/cmd_vel",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=env,
    )
    # Wait for twist_mux to come up. It logs "Subscribed to topic" lines.
    # Crude readiness wait: just sleep long enough for the node to register.
    time.sleep(2.0)
    if proc.poll() is not None:
        out = proc.stdout.read().decode(errors="replace") if proc.stdout else ""
        raise RuntimeError(f"twist_mux exited early:\n{out}")
    try:
        yield proc
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5.0)


def _make_twist(linear_x: float) -> Twist:
    msg = Twist()
    msg.linear.x = float(linear_x)
    return msg


def _spin_until(node, predicate, timeout_sec: float) -> bool:
    """Spin ``node`` until ``predicate()`` is true or timeout."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def _spin_for(node, duration_sec: float) -> None:
    """Spin ``node`` for ``duration_sec``, letting executor callbacks run."""
    deadline = time.monotonic() + duration_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def test_real_twist_mux_obeys_helix_priority(twist_mux_proc):
    """End-to-end: teleop=0.7 + helix=0.0 + nav=0.3 -> /cmd_vel carries 0.7."""
    node = rclpy.create_node("twist_mux_integration_test")
    try:
        teleop_pub = node.create_publisher(Twist, "/teleop/cmd_vel", 10)
        helix_pub = node.create_publisher(Twist, "/helix/cmd_vel", 10)
        nav_pub = node.create_publisher(Twist, "/nav/cmd_vel", 10)

        received: list[float] = []

        def _on_cmd_vel(msg: Twist) -> None:
            received.append(msg.linear.x)

        node.create_subscription(Twist, "/cmd_vel", _on_cmd_vel, 10)

        # Let pub/sub discovery settle. twist_mux needs time to see our
        # publishers, AND our subscriber needs time to see twist_mux's
        # publisher. Spinning during the wait helps both halves discover.
        _spin_for(node, 1.5)

        # Publish all three at the same wall time. Highest priority must win.
        # We loop long enough to outlast twist_mux's diagnostics update_rate
        # and to give discovery enough headroom under the default CycloneDDS.
        for _ in range(40):
            teleop_pub.publish(_make_twist(0.7))
            helix_pub.publish(_make_twist(0.0))
            nav_pub.publish(_make_twist(0.3))
            rclpy.spin_once(node, timeout_sec=0.05)

        got_one = _spin_until(node, lambda: len(received) > 0, timeout_sec=3.0)
        assert got_one, (
            "no /cmd_vel messages received from twist_mux. "
            "Check ROS_DOMAIN_ID isolation and that the twist_mux fixture started."
        )

        # Skip the first message (mux may have only seen one input at startup).
        # Take the last few once all three are publishing.
        steady = received[-5:] if len(received) >= 5 else received
        assert all(abs(v - 0.7) < 1e-3 for v in steady), (
            f"twist_mux did not pick teleop as winner: linear.x samples = {received}"
        )
    finally:
        node.destroy_node()


def test_real_twist_mux_dropout_falls_through_to_helix(twist_mux_proc):
    """Teleop publishes once then stops; helix keeps publishing.
    After teleop's 0.5 s timeout, /cmd_vel must carry helix's value (0.0)."""
    node = rclpy.create_node("twist_mux_dropout_test")
    try:
        teleop_pub = node.create_publisher(Twist, "/teleop/cmd_vel", 10)
        helix_pub = node.create_publisher(Twist, "/helix/cmd_vel", 10)

        received: list[tuple[float, float]] = []  # (wall_ts, linear_x)

        def _on_cmd_vel(msg: Twist) -> None:
            received.append((time.monotonic(), msg.linear.x))

        node.create_subscription(Twist, "/cmd_vel", _on_cmd_vel, 10)

        time.sleep(1.0)  # discovery

        # Phase A: teleop alive. Pump for 0.3 s.
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.3:
            teleop_pub.publish(_make_twist(0.5))
            helix_pub.publish(_make_twist(0.0))
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.05)

        teleop_seen = [v for _, v in received if abs(v - 0.5) < 1e-3]
        assert teleop_seen, (
            f"teleop never won the mux during phase A: samples={received}"
        )

        # Phase B: teleop goes silent. Helix keeps publishing for >timeout.
        t1 = time.monotonic()
        while time.monotonic() - t1 < 1.0:   # well past 0.5 s timeout
            helix_pub.publish(_make_twist(0.0))
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.05)

        # After teleop's timeout, /cmd_vel must carry helix's value.
        after_dropout = [v for ts, v in received if ts > t1 + 0.6]
        assert after_dropout, (
            f"no /cmd_vel samples after teleop dropout window: {received}"
        )
        assert all(abs(v - 0.0) < 1e-3 for v in after_dropout), (
            "after teleop dropout, /cmd_vel did not fall through to helix. "
            f"samples after dropout = {after_dropout}"
        )
    finally:
        node.destroy_node()
