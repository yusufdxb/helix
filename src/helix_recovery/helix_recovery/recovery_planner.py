"""
RecoveryPlanner — HELIX Phase 2 main recovery orchestration node.

Subscribes to /helix/faults. On each fault:
  1. Logs and persists the fault to SQLite.
  2. Looks up the tiered recovery policy (loaded from YAML at configure time).
  3. Determines starting tier based on recent history (flap detection).
  4. Runs the recovery chain on a background asyncio event loop.
  5. Publishes RecoveryAction to /helix/recovery_actions after each attempt.

The asyncio event loop runs in a daemon thread so the rclpy executor is never
blocked during long-running recovery operations.
"""
import asyncio
import functools
import time
import threading
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from helix_msgs.msg import FaultEvent, RecoveryAction

from helix_recovery.state_db import StateDB, DEFAULT_DB_PATH
from helix_recovery.action_executor import ActionExecutor
from helix_recovery.verification_loop import VerificationLoop


# ── Constants ────────────────────────────────────────────────────────────────
TOPIC_FAULTS: str = "/helix/faults"
TOPIC_RECOVERY_ACTIONS: str = "/helix/recovery_actions"

DEFAULT_POLICY_FILE: str = ""
DEFAULT_RECOVERY_TIMEOUT_SEC: float = 30.0
DEFAULT_MAX_CONCURRENT: int = 3

FLAP_WINDOW_SEC: float = 600.0   # 10 minutes
FLAP_THRESHOLD: int = 5           # >5 attempts in window → skip to tier 2


class RecoveryPlanner(LifecycleNode):
    """
    Lifecycle node orchestrating tiered fault recovery for HELIX.

    Exposes a test hook `db_path_override` for injecting a temp DB path
    in tests without touching the filesystem default.
    """

    def __init__(self, db_path_override: Optional[str] = None) -> None:
        """
        Initialize the planner node and declare ROS 2 parameters.

        Args:
            db_path_override: If set, uses this SQLite path instead of the default.
                              Intended for testing only.
        """
        super().__init__("helix_recovery_planner")

        self.declare_parameter("policy_file_path", DEFAULT_POLICY_FILE)
        self.declare_parameter("recovery_timeout_sec", DEFAULT_RECOVERY_TIMEOUT_SEC)
        self.declare_parameter("max_concurrent_recoveries", DEFAULT_MAX_CONCURRENT)
        self.declare_parameter("standalone_mode", False)

        self._db_path_override = db_path_override
        self._policy: Dict[str, Any] = {}
        self._state_db: Optional[StateDB] = None
        self._executor: Optional[ActionExecutor] = None
        self._verifier: Optional[VerificationLoop] = None

        # asyncio event loop running in background thread
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._loop_thread: Optional[threading.Thread] = None

        # node_name -> asyncio.Future (active recovery chains, for debouncing)
        self._active_recoveries: Dict[str, asyncio.Future] = {}
        self._active_lock = threading.Lock()

        self._fault_sub = None
        self._action_pub = None

    # ── Lifecycle callbacks ──────────────────────────────────────────────────

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Load policy YAML, initialise DB, create pub/sub."""
        policy_path = self.get_parameter("policy_file_path").value
        self._recovery_timeout = self.get_parameter("recovery_timeout_sec").value
        self._max_concurrent = self.get_parameter("max_concurrent_recoveries").value

        # Load recovery policy
        if not policy_path:
            self.get_logger().warn(
                "policy_file_path not set — RecoveryPlanner will have no policy"
            )
            self._policy = {}
        else:
            try:
                self._policy = self._load_policy(policy_path)
                self.get_logger().info(
                    f"Recovery policy loaded from '{policy_path}': "
                    f"{list(self._policy.keys())}"
                )
            except Exception as exc:
                self.get_logger().error(f"Failed to load policy: {exc}")
                return TransitionCallbackReturn.FAILURE

        # SQLite state persistence
        db_path = self._db_path_override or DEFAULT_DB_PATH
        self._state_db = StateDB(db_path=db_path)
        self.get_logger().info(f"StateDB initialised at '{db_path}'")

        # Publishers / subscribers
        self._action_pub = self.create_publisher(
            RecoveryAction, TOPIC_RECOVERY_ACTIONS, 10
        )
        self._fault_sub = self.create_subscription(
            FaultEvent, TOPIC_FAULTS, self._on_fault_received, 10
        )

        # Helper objects (ActionExecutor receives this node for publishing)
        self._executor = ActionExecutor(node=self)
        self._verifier = VerificationLoop()

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Start the asyncio event loop in a daemon thread."""
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(
            target=self._loop.run_forever, daemon=True, name="helix-recovery-loop"
        )
        self._loop_thread.start()
        self.get_logger().info(
            f"RecoveryPlanner activated — asyncio loop running in '{self._loop_thread.name}'"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Stop the asyncio event loop and wait for active recoveries to cancel."""
        if self._loop:
            # Cancel all pending recovery futures
            with self._active_lock:
                for fut in self._active_recoveries.values():
                    self._loop.call_soon_threadsafe(fut.cancel)
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._loop_thread:
            self._loop_thread.join(timeout=5.0)
        self.get_logger().info("RecoveryPlanner deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Destroy ROS resources."""
        if self._fault_sub:
            self.destroy_subscription(self._fault_sub)
        if self._action_pub:
            self.destroy_publisher(self._action_pub)
        self._active_recoveries.clear()
        return TransitionCallbackReturn.SUCCESS

    # ── ROS 2 fault callback ─────────────────────────────────────────────────

    def _on_fault_received(self, msg: FaultEvent) -> None:
        """
        Called by the rclpy executor on each /helix/faults message.

        Debounces same-node concurrent recovery, then schedules the async
        recovery chain on the background asyncio event loop.
        """
        self.get_logger().warn(
            f"Fault received: {msg.fault_type} on '{msg.node_name}' "
            f"(severity={msg.severity})"
        )

        with self._active_lock:
            # Debounce: drop if recovery already running for this node
            if msg.node_name in self._active_recoveries:
                existing = self._active_recoveries[msg.node_name]
                if not existing.done():
                    self.get_logger().warn(
                        f"Recovery already in progress for '{msg.node_name}' — "
                        "dropping duplicate fault"
                    )
                    return

            # Check concurrency cap
            active_count = sum(
                1 for f in self._active_recoveries.values() if not f.done()
            )
            if active_count >= self._max_concurrent:
                self.get_logger().error(
                    f"Max concurrent recoveries ({self._max_concurrent}) reached — "
                    f"dropping fault for '{msg.node_name}'"
                )
                return

        # Persist fault to DB
        fault_id = self._state_db.insert_fault({
            "node_name": msg.node_name,
            "fault_type": msg.fault_type,
            "severity": msg.severity,
            "detail": msg.detail,
            "timestamp": msg.timestamp,
        })

        # Schedule recovery chain on asyncio loop (non-blocking from rclpy thread)
        coro = self._recovery_chain(msg, fault_id)
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)

        def _done_cb(fut: asyncio.Future) -> None:
            with self._active_lock:
                self._active_recoveries.pop(msg.node_name, None)
            if fut.exception():
                self.get_logger().error(
                    f"Recovery chain for '{msg.node_name}' raised: {fut.exception()}"
                )

        future.add_done_callback(_done_cb)
        with self._active_lock:
            self._active_recoveries[msg.node_name] = future

    # ── Async recovery orchestration ─────────────────────────────────────────

    async def _recovery_chain(self, fault: FaultEvent, fault_id: str) -> None:
        """
        Execute the tiered recovery chain for the given fault, respecting
        recovery_timeout_sec as a hard overall deadline.
        """
        try:
            await asyncio.wait_for(
                self._run_tiers(fault, fault_id),
                timeout=self._recovery_timeout,
            )
        except asyncio.TimeoutError:
            self.get_logger().error(
                f"Recovery chain for '{fault.node_name}:{fault.fault_type}' "
                f"exceeded {self._recovery_timeout}s timeout — aborting"
            )
            self._publish_recovery_action(
                fault=fault,
                fault_id=fault_id,
                action_taken="timeout",
                tier=0,
                attempt_number=0,
                success=False,
                duration_sec=self._recovery_timeout,
                outcome_detail=f"Recovery timed out after {self._recovery_timeout}s",
            )

    async def _run_tiers(self, fault: FaultEvent, fault_id: str) -> None:
        """Iterate through policy tiers, escalating on failure."""
        policy = self._policy.get(fault.fault_type)
        if not policy:
            self.get_logger().warn(
                f"No recovery policy for fault_type '{fault.fault_type}'"
            )
            return

        tiers: List[Dict[str, Any]] = policy.get("tiers", [])

        # Flap detection: start at tier 2 if node has flapped recently
        recent_faults = self._state_db.get_recent_faults(fault.node_name, limit=20)
        now = time.time()
        recent_count = sum(
            1 for f in recent_faults
            if now - f.get("created_at", 0) < FLAP_WINDOW_SEC
        )
        start_tier = 2 if recent_count > FLAP_THRESHOLD else 1

        if start_tier == 2:
            self.get_logger().warn(
                f"'{fault.node_name}' has {recent_count} faults in the last "
                f"{FLAP_WINDOW_SEC/60:.0f}min — skipping to tier 2 (flap detected)"
            )

        self.get_logger().info(
            f"Recovery chain started: {fault.fault_type} on '{fault.node_name}'"
        )

        for tier_config in tiers:
            tier_num = tier_config.get("tier", 1)
            if tier_num < start_tier:
                continue

            action = tier_config.get("action", "")

            # LLM tier stub (Phase 3 hook)
            if action == "llm_requested":
                self.get_logger().warn(
                    f"LLM tier reached for '{fault.node_name}:{fault.fault_type}' "
                    "— Phase 3 pending"
                )
                self._publish_recovery_action(
                    fault=fault,
                    fault_id=fault_id,
                    action_taken="llm_requested",
                    tier=3,
                    attempt_number=1,
                    success=False,
                    duration_sec=0.0,
                    outcome_detail="Awaiting LLM integration (Phase 3)",
                )
                return

            max_attempts = tier_config.get("max_attempts", 1)
            backoff_list: List[float] = tier_config.get("backoff_sec", [1.0])

            for attempt_num in range(1, max_attempts + 1):
                backoff = backoff_list[min(attempt_num - 1, len(backoff_list) - 1)]
                self.get_logger().info(
                    f"Tier {tier_num} action '{action}' attempt {attempt_num}/{max_attempts} "
                    f"for '{fault.node_name}'"
                )

                # Execute the action
                success, detail, duration = await self._dispatch_action(
                    action, tier_config, fault
                )

                # Verification (blocking — run in thread pool so loop stays free)
                loop = asyncio.get_event_loop()
                verified, verify_detail = await loop.run_in_executor(
                    None,
                    functools.partial(
                        self._verifier.verify,
                        fault.fault_type,
                        fault.node_name,
                        action,
                    ),
                )

                overall_success = success and verified
                outcome = (
                    f"{detail} | Verification: {verify_detail}"
                    if verified
                    else f"{detail} | Verification FAILED: {verify_detail}"
                )

                # Persist to DB
                self._state_db.insert_recovery_attempt({
                    "fault_id": fault_id,
                    "action_taken": action,
                    "tier": tier_num,
                    "attempt_number": attempt_num,
                    "success": overall_success,
                    "duration_sec": duration,
                    "outcome_detail": outcome,
                    "timestamp": time.time(),
                })

                # Publish RecoveryAction
                self._publish_recovery_action(
                    fault=fault,
                    fault_id=fault_id,
                    action_taken=action,
                    tier=tier_num,
                    attempt_number=attempt_num,
                    success=overall_success,
                    duration_sec=duration,
                    outcome_detail=outcome,
                )

                if overall_success:
                    self._state_db.update_node_history(
                        fault.node_name, fault.fault_type, success=True
                    )
                    self.get_logger().info(
                        f"Tier {tier_num} action '{action}' succeeded in {duration:.2f}s"
                    )
                    return

                self.get_logger().warn(
                    f"Tier {tier_num} action '{action}' failed "
                    f"(attempt {attempt_num}/{max_attempts}) — "
                    f"{'retrying' if attempt_num < max_attempts else 'escalating to tier ' + str(tier_num + 1)}"
                )
                self._state_db.update_node_history(
                    fault.node_name, fault.fault_type, success=False
                )

                if attempt_num < max_attempts:
                    await asyncio.sleep(backoff)
                else:
                    # Exhausted attempts for this tier — break to next tier
                    await asyncio.sleep(backoff)

        # All tiers exhausted
        self.get_logger().error(
            f"All recovery tiers exhausted for '{fault.node_name}:{fault.fault_type}'"
        )
        self._publish_recovery_action(
            fault=fault,
            fault_id=fault_id,
            action_taken="exhausted",
            tier=0,
            attempt_number=0,
            success=False,
            duration_sec=0.0,
            outcome_detail="All recovery tiers exhausted",
        )

    async def _dispatch_action(
        self,
        action: str,
        tier_config: Dict[str, Any],
        fault: FaultEvent,
    ) -> Tuple[bool, str, float]:
        """Map action string to ActionExecutor method call."""
        ex = self._executor
        node_name = fault.node_name

        if action == "restart_node":
            return await ex.restart_node(node_name)
        elif action == "activate_fallback_topic":
            fallback_map = tier_config.get("fallback_map", {})
            return await ex.activate_fallback_topic(node_name, fallback_map)
        elif action == "trigger_global_localization":
            return await ex.trigger_global_localization()
        elif action == "reinit_from_waypoint":
            waypoint_topic = tier_config.get("waypoint_topic", "/last_waypoint")
            return await ex.reinit_from_waypoint(waypoint_topic)
        elif action == "reduce_velocity_limits":
            factor = tier_config.get("factor", 0.5)
            return await ex.reduce_velocity_limits(node_name, factor)
        elif action == "activate_safe_mode":
            return await ex.activate_safe_mode()
        elif action == "reconfigure_dds":
            return await ex.reconfigure_dds()
        elif action == "standalone_mode":
            return await ex.standalone_mode()
        else:
            return (False, f"Unknown action '{action}'", 0.0)

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _publish_recovery_action(
        self,
        fault: FaultEvent,
        fault_id: str,
        action_taken: str,
        tier: int,
        attempt_number: int,
        success: bool,
        duration_sec: float,
        outcome_detail: str,
    ) -> None:
        """Build and publish a RecoveryAction message."""
        msg = RecoveryAction()
        msg.fault_id = fault_id
        msg.fault_type = fault.fault_type
        msg.node_name = fault.node_name
        msg.action_taken = action_taken
        msg.tier = tier
        msg.attempt_number = attempt_number
        msg.success = success
        msg.duration_sec = duration_sec
        msg.outcome_detail = outcome_detail
        msg.timestamp = time.time()
        self._action_pub.publish(msg)
        self.get_logger().info(
            f"RecoveryAction published: {action_taken} "
            f"({'SUCCESS' if success else 'FAILED'}) tier={tier}"
        )

    @staticmethod
    def _load_policy(path: str) -> Dict[str, Any]:
        """Load and parse the YAML recovery policy file."""
        import yaml
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        return data.get("recovery_policies", {})


def main(args=None) -> None:
    """Entry point for helix_recovery_planner console script."""
    rclpy.init(args=args)
    node = RecoveryPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
