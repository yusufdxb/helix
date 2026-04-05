"""
LogParser — HELIX Phase 1 fault sensing node.

Subscribes to /rosout (rcl_interfaces/msg/Log). Filters for severity >= 40 (ERROR).
Loads YAML rules at configure time, compiles patterns as regex.
Emits FaultEvent when a log message matches a rule.
Deduplicates: same rule_id + node_name suppressed within `dedup_window_sec`.
"""
import re
import threading
import time
from typing import Any, Dict, List, Tuple

import rclpy
from rcl_interfaces.msg import Log
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn

from helix_msgs.msg import FaultEvent

# ── Constants ────────────────────────────────────────────────────────────────
DEFAULT_RULES_FILE_PATH: str = ""
DEFAULT_DEDUP_WINDOW_SEC: float = 5.0
MIN_LOG_SEVERITY: int = 40        # ERROR; DEBUG=10, INFO=20, WARN=30, ERROR=40, FATAL=50


class LogParser(LifecycleNode):
    """Lifecycle node that monitors /rosout for fault-indicating log patterns."""

    def __init__(self) -> None:
        """Initialize node and declare parameters."""
        super().__init__("helix_log_parser")

        self.declare_parameter("rules_file_path", DEFAULT_RULES_FILE_PATH)
        self.declare_parameter("dedup_window_sec", DEFAULT_DEDUP_WINDOW_SEC)

        self._rules: List[Dict[str, Any]] = []
        # (rule_id, node_name) -> last_emitted_timestamp
        self._dedup_cache: Dict[Tuple[str, str], float] = {}
        self._dedup_lock: threading.Lock = threading.Lock()

        self._fault_pub = None
        self._rosout_sub = None

    # ── Lifecycle callbacks ──────────────────────────────────────────────────

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Load rules, compile regex patterns, create publishers/subscribers."""
        self._dedup_window = self.get_parameter("dedup_window_sec").value
        rules_path = self.get_parameter("rules_file_path").value

        if not rules_path:
            self.get_logger().warn(
                "rules_file_path parameter is empty — LogParser will have no rules"
            )
            self._rules = []
        else:
            try:
                self._rules = self._load_rules(rules_path)
                self.get_logger().info(
                    f"Loaded {len(self._rules)} rules from '{rules_path}'"
                )
            except Exception as exc:
                self.get_logger().error(f"Failed to load rules from '{rules_path}': {exc}")
                return TransitionCallbackReturn.FAILURE

        self._fault_pub = self.create_publisher(FaultEvent, "/helix/faults", 10)
        self._rosout_sub = self.create_subscription(
            Log, "/rosout", self._on_log, 100
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate the log parser."""
        self.get_logger().info(
            f"LogParser activated with {len(self._rules)} rules, "
            f"dedup_window={self._dedup_window}s"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate the log parser."""
        self.get_logger().info("LogParser deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Destroy publishers/subscribers and clear state."""
        if self._fault_pub:
            self.destroy_publisher(self._fault_pub)
        if self._rosout_sub:
            self.destroy_subscription(self._rosout_sub)
        self._rules.clear()
        with self._dedup_lock:
            self._dedup_cache.clear()
        return TransitionCallbackReturn.SUCCESS

    # ── Internal helpers ─────────────────────────────────────────────────────

    def _load_rules(self, path: str) -> List[Dict[str, Any]]:
        """Load and validate YAML rules file. Compiles regex patterns in-place."""
        import yaml  # lazy import — only needed at configure time

        with open(path, "r") as f:
            data = yaml.safe_load(f)

        rules = data.get("rules", [])
        compiled = []
        for rule in rules:
            required = {"id", "pattern", "fault_type", "severity", "detail_template"}
            missing = required - set(rule.keys())
            if missing:
                self.get_logger().warn(
                    f"Rule '{rule.get('id', '?')}' missing fields {missing} — skipping"
                )
                continue
            rule["_compiled"] = re.compile(rule["pattern"], re.IGNORECASE)
            compiled.append(rule)
        return compiled

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _on_log(self, log: Log) -> None:
        """
        Process incoming /rosout message.
        Only act on severity >= MIN_LOG_SEVERITY (ERROR).
        """
        if log.level < MIN_LOG_SEVERITY:
            return

        for rule in self._rules:
            pattern: re.Pattern = rule["_compiled"]
            if not pattern.search(log.msg):
                continue

            rule_id: str = rule["id"]
            node_name: str = log.name

            # Deduplication check
            with self._dedup_lock:
                key = (rule_id, node_name)
                now = time.time()
                last_emitted = self._dedup_cache.get(key, 0.0)
                if now - last_emitted < self._dedup_window:
                    self.get_logger().debug(
                        f"Suppressing duplicate fault for rule '{rule_id}' "
                        f"from '{node_name}' (within dedup window)"
                    )
                    continue
                self._dedup_cache[key] = now

            self._emit_log_fault(log, rule)

    def _emit_log_fault(self, log: Log, rule: Dict[str, Any]) -> None:
        """Build and publish a FaultEvent from a matched log + rule."""
        truncated_msg = log.msg[:120]
        msg = FaultEvent()
        msg.node_name = log.name
        msg.fault_type = rule["fault_type"]
        msg.severity = int(rule["severity"])
        msg.detail = f"{rule['detail_template']} | Log: {truncated_msg}"
        msg.timestamp = time.time()
        msg.context_keys = [
            "matched_rule_id", "log_severity", "log_name", "log_file"
        ]
        msg.context_values = [
            rule["id"],
            str(log.level),
            log.name,
            log.file,
        ]
        self._fault_pub.publish(msg)
        self.get_logger().info(
            f"FaultEvent emitted: {rule['fault_type']} for '{log.name}' "
            f"(rule='{rule['id']}')"
        )


def main(args=None) -> None:
    """Entry point for helix_log_parser console script."""
    rclpy.init(args=args)
    node = LogParser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
