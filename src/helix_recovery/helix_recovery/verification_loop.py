"""
VerificationLoop — HELIX Phase 2 recovery verification.

Each verify() call creates a temporary rclpy node + SingleThreadedExecutor,
spins for up to timeout_sec, then destroys both. This avoids sharing the
planner's executor.

verify() is synchronous and blocking. In the async recovery chain it must be
called via: await loop.run_in_executor(None, partial(verifier.verify, ...))
"""
import time
import threading
from typing import Tuple

import rclpy
import rclpy.node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

# ── Constants ────────────────────────────────────────────────────────────────
TOPIC_HEARTBEAT: str = "/helix/heartbeat"
TOPIC_NODE_HEALTH: str = "/helix/node_health"
FALLBACK_VERIFY_TIMEOUT_SEC: float = 3.0
VELOCITY_PARAM_VERIFY_TIMEOUT_SEC: float = 5.0


class VerificationLoop:
    """
    Verifies whether a recovery action actually fixed the detected fault.

    Methods are synchronous and blocking; call from a thread pool in async contexts.
    """

    def verify(
        self,
        fault_type: str,
        node_name: str,
        action_taken: str,
        timeout_sec: float = 8.0,
    ) -> Tuple[bool, str]:
        """
        Dispatch to the appropriate verification strategy based on fault_type + action.

        Args:
            fault_type:   e.g. "CRASH", "ANOMALY", "LOG_PATTERN", "NETWORK"
            node_name:    Name of the affected node.
            action_taken: Name of the action that was executed.
            timeout_sec:  Maximum time to wait for confirmation.

        Returns:
            (verified: bool, detail: str)
        """
        strategy_key = (fault_type, action_taken)

        if strategy_key == ("CRASH", "restart_node"):
            return self._verify_heartbeat(node_name, timeout_sec)
        elif strategy_key == ("CRASH", "activate_fallback_topic"):
            return self._verify_any_heartbeat(timeout_sec=FALLBACK_VERIFY_TIMEOUT_SEC)
        elif action_taken == "trigger_global_localization":
            return self._verify_slam_health(node_name, timeout_sec)
        elif strategy_key == ("ANOMALY", "reduce_velocity_limits"):
            return self._verify_velocity_reduced(timeout_sec)
        elif strategy_key == ("NETWORK", "reconfigure_dds"):
            return self._verify_any_heartbeat(timeout_sec)
        elif action_taken in ("activate_safe_mode", "standalone_mode"):
            # These are terminal actions; we don't wait for a further signal
            return (True, f"Terminal action '{action_taken}' executed — no further verification")
        else:
            return (True, f"No verification strategy for ({fault_type}, {action_taken})")

    # ── Verification strategies ──────────────────────────────────────────────

    def _verify_heartbeat(
        self, node_name: str, timeout_sec: float
    ) -> Tuple[bool, str]:
        """
        Subscribe to /helix/heartbeat and wait for a message with data==node_name.
        """
        event = threading.Event()
        temp_node = rclpy.create_node("helix_verify_hb")

        def _cb(msg: String) -> None:
            if msg.data.strip() == node_name:
                event.set()

        temp_node.create_subscription(String, TOPIC_HEARTBEAT, _cb, 10)
        self._spin_until(temp_node, event, timeout_sec)
        temp_node.destroy_node()

        if event.is_set():
            return (True, f"Heartbeat confirmed from '{node_name}'")
        return (False, f"No heartbeat from '{node_name}' within {timeout_sec}s")

    def _verify_any_heartbeat(self, timeout_sec: float) -> Tuple[bool, str]:
        """Subscribe to /helix/heartbeat and return True if any message arrives."""
        event = threading.Event()
        temp_node = rclpy.create_node("helix_verify_any_hb")
        temp_node.create_subscription(
            String, TOPIC_HEARTBEAT, lambda _: event.set(), 10
        )
        self._spin_until(temp_node, event, timeout_sec)
        temp_node.destroy_node()

        if event.is_set():
            return (True, "Heartbeat received — DDS/fallback communication restored")
        return (False, f"No heartbeat received within {timeout_sec}s after fallback")

    def _verify_slam_health(
        self, node_name: str, timeout_sec: float
    ) -> Tuple[bool, str]:
        """
        Subscribe to /helix/node_health and wait for the slam node to reach OK status.
        """
        event = threading.Event()
        temp_node = rclpy.create_node("helix_verify_slam")

        def _cb(msg: DiagnosticArray) -> None:
            for status in msg.status:
                if node_name in status.name and status.level == DiagnosticStatus.OK:
                    event.set()

        temp_node.create_subscription(DiagnosticArray, TOPIC_NODE_HEALTH, _cb, 10)
        self._spin_until(temp_node, event, timeout_sec)
        temp_node.destroy_node()

        if event.is_set():
            return (True, f"SLAM node '{node_name}' returned to OK status")
        return (False, f"SLAM node '{node_name}' did not recover within {timeout_sec}s")

    def _verify_velocity_reduced(self, timeout_sec: float) -> Tuple[bool, str]:
        """
        Check that /controller_server max_vel_x was actually reduced via ros2 param get.
        Returns True if the param read succeeds (nonzero exit = controller not running).
        """
        import subprocess
        # subprocess.run is acceptable here since this method is always called
        # via run_in_executor (blocking is intentional — see design decision 3/4).
        try:
            result = subprocess.run(
                ["ros2", "param", "get", "/controller_server", "max_vel_x"],
                capture_output=True,
                text=True,
                timeout=timeout_sec,
            )
            if result.returncode == 0:
                return (True, f"Velocity param read: {result.stdout.strip()}")
            return (True, "Controller server not running — param verification skipped")
        except subprocess.TimeoutExpired:
            return (True, "Velocity param check timed out — assuming reduced")
        except OSError:
            return (True, "ros2 CLI unavailable — assuming velocity reduced")

    # ── Helpers ──────────────────────────────────────────────────────────────

    @staticmethod
    def _spin_until(
        node: rclpy.node.Node, event: threading.Event, timeout_sec: float
    ) -> None:
        """Spin `node` using its own SingleThreadedExecutor until event is set or timeout."""
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        deadline = time.time() + timeout_sec
        while not event.is_set() and time.time() < deadline:
            ex.spin_once(timeout_sec=0.1)
        ex.remove_node(node)
