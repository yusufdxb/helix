"""
ActionExecutor — HELIX Phase 2 recovery action implementation.

Not a ROS node — instantiated by RecoveryPlanner and receives a reference to
the planner's rclpy node for publishing. All action methods are async and use
asyncio.create_subprocess_exec (never subprocess.run) to avoid blocking the
event loop.
"""
import asyncio
import os
import time
from typing import Optional, Tuple

import rclpy
import rclpy.node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Empty, String

# ── Topic constants ──────────────────────────────────────────────────────────
TOPIC_FALLBACK_DIRECTIVE: str = "/helix/fallback_directive"
TOPIC_SAFE_MODE_ACTIVE: str = "/helix/safe_mode_active"
TOPIC_CMD_VEL_SAFE: str = "/cmd_vel_mux/engage_safe"
TOPIC_GLOBAL_LOCALIZATION: str = "/reinitialize_global_localization"
TOPIC_AMCL_POSE: str = "/amcl_pose"
TOPIC_INITIAL_POSE: str = "/initialpose"
TOPIC_DDS_RECONFIGURE: str = "/helix/dds_reconfigure_request"
TOPIC_STANDALONE_ACTIVE: str = "/helix/standalone_active"

LIFECYCLE_TRANSITION_TIMEOUT_SEC: float = 5.0
LOCALIZATION_COVARIANCE_THRESHOLD: float = 0.1
LOCALIZATION_POLL_INTERVAL_SEC: float = 0.5


class ActionExecutor:
    """
    Executes individual recovery actions on behalf of the RecoveryPlanner.

    All async methods return (success: bool, detail: str, duration_sec: float).
    The `node` argument is the planner's rclpy lifecycle node; publishers are
    created once at init and reused across calls.
    """

    def __init__(self, node: rclpy.node.Node) -> None:
        """
        Initialize publishers using the provided ROS 2 node.

        Args:
            node: The owning lifecycle node. Publishers created here are
                  thread-safe in rclpy and can be called from any thread.
        """
        self._node = node
        self._pub_fallback = node.create_publisher(String, TOPIC_FALLBACK_DIRECTIVE, 10)
        self._pub_safe_mode = node.create_publisher(Bool, TOPIC_SAFE_MODE_ACTIVE, 10)
        self._pub_cmd_vel_safe = node.create_publisher(Empty, TOPIC_CMD_VEL_SAFE, 10)
        self._pub_global_loc = node.create_publisher(Empty, TOPIC_GLOBAL_LOCALIZATION, 10)
        self._pub_initial_pose = node.create_publisher(
            PoseWithCovarianceStamped, TOPIC_INITIAL_POSE, 10
        )
        self._pub_dds_reconfig = node.create_publisher(String, TOPIC_DDS_RECONFIGURE, 10)
        self._pub_standalone = node.create_publisher(Bool, TOPIC_STANDALONE_ACTIVE, 10)

    # ── Action implementations ───────────────────────────────────────────────

    async def restart_node(self, node_name: str) -> Tuple[bool, str, float]:
        """
        Lifecycle-restart the named node via four sequential ros2 lifecycle calls.

        Transitions: deactivate → cleanup → configure → activate.
        Each call waits up to LIFECYCLE_TRANSITION_TIMEOUT_SEC seconds.
        Returns success=True only if all four transitions complete cleanly.
        """
        start = time.time()
        transitions = ["deactivate", "cleanup", "configure", "activate"]

        for transition in transitions:
            try:
                proc = await asyncio.create_subprocess_exec(
                    "ros2", "lifecycle", "set", node_name, transition,
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.PIPE,
                )
                try:
                    stdout, stderr = await asyncio.wait_for(
                        proc.communicate(),
                        timeout=LIFECYCLE_TRANSITION_TIMEOUT_SEC,
                    )
                except asyncio.TimeoutError:
                    proc.kill()
                    return (
                        False,
                        f"Timeout during '{transition}' transition for {node_name}",
                        time.time() - start,
                    )

                if proc.returncode != 0:
                    err_msg = stderr.decode(errors="replace").strip()
                    return (
                        False,
                        f"Lifecycle '{transition}' failed for {node_name}: {err_msg}",
                        time.time() - start,
                    )
            except OSError as exc:
                return (False, f"Failed to invoke ros2 CLI: {exc}", time.time() - start)

        return (
            True,
            f"Node {node_name} successfully restarted via lifecycle transitions",
            time.time() - start,
        )

    async def activate_fallback_topic(
        self, node_name: str, fallback_map: dict
    ) -> Tuple[bool, str, float]:
        """
        Publish to /helix/fallback_directive signalling topic rerouting.

        Looks up fallback_map[node_name] for the fallback topic.
        Publishes "<node_name>:<fallback_topic>" as a String message.
        """
        start = time.time()
        fallback_topic = fallback_map.get(node_name)
        if not fallback_topic:
            return (
                False,
                f"No fallback topic configured for node '{node_name}'",
                time.time() - start,
            )
        msg = String()
        msg.data = f"{node_name}:{fallback_topic}"
        self._pub_fallback.publish(msg)
        return (
            True,
            f"Fallback directive published: {node_name} → {fallback_topic}",
            time.time() - start,
        )

    def _poll_amcl_convergence(self, timeout: float) -> Tuple[bool, float]:
        """
        Blocking poll for AMCL covariance convergence. Runs in a thread pool via
        run_in_executor so the asyncio event loop stays unblocked.
        """
        import threading

        from geometry_msgs.msg import PoseWithCovarianceStamped as PWCS
        from rclpy.executors import SingleThreadedExecutor as STE

        covariance_trace: list = [float("inf")]
        event = threading.Event()
        temp_node = rclpy.create_node("helix_amcl_poller")

        def _amcl_cb(msg: PWCS) -> None:
            cov = msg.pose.covariance
            trace = cov[0] + cov[7]  # xx + yy position covariance trace
            covariance_trace[0] = trace
            if trace < LOCALIZATION_COVARIANCE_THRESHOLD:
                event.set()

        temp_node.create_subscription(PWCS, TOPIC_AMCL_POSE, _amcl_cb, 10)
        ex = STE()
        ex.add_node(temp_node)
        deadline = time.time() + timeout
        while not event.is_set() and time.time() < deadline:
            ex.spin_once(timeout_sec=LOCALIZATION_POLL_INTERVAL_SEC)
        ex.remove_node(temp_node)
        temp_node.destroy_node()
        return (event.is_set(), covariance_trace[0])

    async def trigger_global_localization(self) -> Tuple[bool, str, float]:
        """
        Trigger Nav2 global re-localization and wait for AMCL covariance to converge.

        Publishes to /reinitialize_global_localization, then polls /amcl_pose
        for up to 10 seconds via a thread pool (blocking spin kept off asyncio thread).
        """
        start = time.time()
        self._pub_global_loc.publish(Empty())

        timeout = 10.0
        loop = asyncio.get_event_loop()
        converged, trace = await loop.run_in_executor(
            None, self._poll_amcl_convergence, timeout
        )
        detail = (
            f"AMCL covariance converged (trace={trace:.4f})"
            if converged
            else f"AMCL covariance did not converge within {timeout}s (trace={trace:.4f})"
        )
        return (converged, detail, time.time() - start)

    def _poll_waypoint(self, waypoint_topic: str) -> Optional[PoseWithCovarianceStamped]:
        """
        Blocking one-shot subscriber read for waypoint pose. Runs in a thread pool via
        run_in_executor so the asyncio event loop stays unblocked.
        """
        import threading

        from rclpy.executors import SingleThreadedExecutor as STE

        received_pose: list = [None]
        event = threading.Event()
        temp_node = rclpy.create_node("helix_waypoint_reader")

        def _pose_cb(msg: PoseWithCovarianceStamped) -> None:
            received_pose[0] = msg
            event.set()

        temp_node.create_subscription(
            PoseWithCovarianceStamped, waypoint_topic, _pose_cb, 10
        )
        ex = STE()
        ex.add_node(temp_node)
        deadline = time.time() + 5.0
        while not event.is_set() and time.time() < deadline:
            ex.spin_once(timeout_sec=0.1)
        ex.remove_node(temp_node)
        temp_node.destroy_node()
        return received_pose[0]

    async def reinit_from_waypoint(
        self, waypoint_topic: str
    ) -> Tuple[bool, str, float]:
        """
        Read last known good pose from waypoint_topic and publish to /initialpose.

        Subscribes once (waits up to 5s via thread pool), then republishes to /initialpose.
        """
        start = time.time()
        loop = asyncio.get_event_loop()
        pose = await loop.run_in_executor(None, self._poll_waypoint, waypoint_topic)

        if pose is None:
            return (False, f"No pose received from {waypoint_topic}", time.time() - start)

        self._pub_initial_pose.publish(pose)
        return (True, f"Initial pose republished from {waypoint_topic}", time.time() - start)

    async def reduce_velocity_limits(
        self, node_name: str, factor: float = 0.5
    ) -> Tuple[bool, str, float]:
        """
        Reduce Nav2 controller_server velocity limits by `factor` via ros2 param set.

        Reads current max_vel_x and max_vel_theta, multiplies by factor, sets new values.
        """
        start = time.time()

        async def _get_param(param: str) -> Optional[float]:
            try:
                proc = await asyncio.create_subprocess_exec(
                    "ros2", "param", "get", "/controller_server", param,
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.PIPE,
                )
                stdout, _ = await asyncio.wait_for(proc.communicate(), timeout=5.0)
                # Output format: "Double value is: 0.5"
                text = stdout.decode(errors="replace")
                for word in text.split():
                    try:
                        return float(word)
                    except ValueError:
                        continue
            except (OSError, asyncio.TimeoutError):
                pass
            return None

        async def _set_param(param: str, value: float) -> bool:
            try:
                proc = await asyncio.create_subprocess_exec(
                    "ros2", "param", "set", "/controller_server", param, str(value),
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.PIPE,
                )
                _, _ = await asyncio.wait_for(proc.communicate(), timeout=5.0)
                return proc.returncode == 0
            except (OSError, asyncio.TimeoutError):
                return False

        for param in ("max_vel_x", "max_vel_theta"):
            current = await _get_param(param)
            if current is None:
                # Param unavailable (controller_server may not be running); non-fatal
                self._node.get_logger().warn(
                    f"Could not read /controller_server {param} — skipping"
                )
                continue
            new_val = round(current * factor, 4)
            ok = await _set_param(param, new_val)
            if not ok:
                return (
                    False,
                    f"Failed to set /controller_server {param} to {new_val}",
                    time.time() - start,
                )

        return (
            True,
            f"Velocity limits reduced by factor {factor} on /controller_server",
            time.time() - start,
        )

    async def activate_safe_mode(self) -> Tuple[bool, str, float]:
        """
        Engage safe mode: publish True to /helix/safe_mode_active and stop motion.

        Also publishes Empty to /cmd_vel_mux/engage_safe to halt the robot.
        """
        start = time.time()
        safe_msg = Bool()
        safe_msg.data = True
        self._pub_safe_mode.publish(safe_msg)
        self._pub_cmd_vel_safe.publish(Empty())
        self._node.get_logger().fatal(
            "HELIX SAFE MODE ACTIVE — motion halted, awaiting operator or LLM recovery"
        )
        return (True, "Safe mode engaged: /helix/safe_mode_active=True, motion stopped", time.time() - start)

    async def reconfigure_dds(self) -> Tuple[bool, str, float]:
        """
        Signal DDS reconfiguration by publishing to /helix/dds_reconfigure_request.

        Reads CYCLONEDDS_URI env var; logs whether a valid config file exists.
        The actual DDS restart must be handled by an external watchdog process.
        """
        start = time.time()
        cyclone_uri = os.environ.get("CYCLONEDDS_URI", "")
        if cyclone_uri and os.path.isfile(cyclone_uri):
            self._node.get_logger().info(f"CycloneDDS config present: {cyclone_uri}")
        else:
            self._node.get_logger().warn(
                f"CYCLONEDDS_URI not set or file missing: '{cyclone_uri}'"
            )

        req_msg = String()
        req_msg.data = str(time.time())
        self._pub_dds_reconfig.publish(req_msg)
        return (
            True,
            f"DDS reconfigure request published (CYCLONEDDS_URI={cyclone_uri!r})",
            time.time() - start,
        )

    async def standalone_mode(self) -> Tuple[bool, str, float]:
        """
        Enable standalone mode: set ROS param and publish /helix/standalone_active.

        Sets /helix_recovery_planner/standalone_mode = True via rclpy set_parameters.
        """
        start = time.time()
        try:
            self._node.set_parameters(
                [Parameter("standalone_mode", Parameter.Type.BOOL, True)]
            )
        except Exception as exc:
            self._node.get_logger().warn(f"Could not set standalone_mode param: {exc}")

        msg = Bool()
        msg.data = True
        self._pub_standalone.publish(msg)
        return (True, "Standalone mode activated", time.time() - start)
