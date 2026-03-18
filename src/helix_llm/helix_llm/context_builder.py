"""
ContextBuilder — assembles Ollama prompts from fault events and recovery history.

Keeps prompts within MAX_PROMPT_TOKENS to avoid exceeding model context limits.
Token estimate: 1 token ≈ 4 characters (rough heuristic, good enough for budgeting).
"""
from typing import Any, Dict, List

MAX_PROMPT_TOKENS: int = 2000

# Valid action names the LLM may suggest — enumerated in the prompt.
VALID_ACTIONS: List[str] = [
    "restart_node",
    "activate_fallback_topic",
    "trigger_global_localization",
    "reinit_from_waypoint",
    "reduce_velocity_limits",
    "activate_safe_mode",
    "reconfigure_dds",
    "standalone_mode",
]

_SYSTEM_PREFIX = """\
You are HELIX, an autonomous fault recovery advisor for a ROS 2 quadruped robot.
You analyse fault events and recovery history, then recommend the single best recovery action.

VALID ACTIONS (you MUST choose one of these exactly):
{actions}

Respond ONLY with a JSON object — no other text:
{{
  "suggested_action": "<one of the valid actions above>",
  "confidence": <float 0.0–1.0>,
  "reasoning": "<one sentence explanation>"
}}
""".format(actions="\n".join(f"  - {a}" for a in VALID_ACTIONS))

_FAULT_TEMPLATE = """\

FAULT EVENT:
  node_name  : {node_name}
  fault_type : {fault_type}
  severity   : {severity}
  detail     : {detail}
"""

_HISTORY_HEADER = "\nRECENT RECOVERY HISTORY (oldest first):\n"
_HISTORY_ITEM = "  [{i}] action={action_taken} tier={tier} success={success}\n"

_NO_HISTORY = "\nRECENT RECOVERY HISTORY: none\n"

_INSTRUCTION_SUFFIX = "\nRecommend the best recovery action now:\n"


class ContextBuilder:
    """Builds Ollama generation prompts from fault events and recovery history."""

    def estimate_tokens(self, text: str) -> int:
        """Rough token estimate: 1 token ≈ 4 characters."""
        return len(text) // 4

    def build_prompt(
        self,
        fault: Dict[str, Any],
        history: List[Dict[str, Any]],
    ) -> str:
        """
        Assemble a prompt for the LLM.

        Args:
            fault: dict with keys node_name, fault_type, severity, detail, timestamp.
            history: list of recent recovery attempt dicts (action_taken, success, tier, etc.).
                     May be empty. Truncated from oldest end if prompt exceeds token budget.

        Returns:
            Prompt string ready to send to Ollama.
        """
        fault_block = _FAULT_TEMPLATE.format(
            node_name=fault.get("node_name", "unknown"),
            fault_type=fault.get("fault_type", "unknown"),
            severity=fault.get("severity", 0),
            detail=fault.get("detail", ""),
        )

        fixed_tokens = self.estimate_tokens(_SYSTEM_PREFIX + fault_block + _INSTRUCTION_SUFFIX)
        history_budget = MAX_PROMPT_TOKENS - fixed_tokens

        history_block = self._build_history_block(history, history_budget)

        return _SYSTEM_PREFIX + fault_block + history_block + _INSTRUCTION_SUFFIX

    def _build_history_block(
        self, history: List[Dict[str, Any]], token_budget: int
    ) -> str:
        """Build history section, dropping oldest entries until within budget."""
        if not history:
            return _NO_HISTORY

        # Build items newest-first, then reverse, to prioritise recent context.
        items = []
        used = self.estimate_tokens(_HISTORY_HEADER)

        for i, entry in enumerate(reversed(history)):
            item = _HISTORY_ITEM.format(
                i=len(history) - i,
                action_taken=entry.get("action_taken", "?"),
                tier=entry.get("tier", "?"),
                success=entry.get("success", "?"),
            )
            item_tokens = self.estimate_tokens(item)
            if used + item_tokens > token_budget:
                break
            items.append(item)
            used += item_tokens

        if not items:
            return _NO_HISTORY

        items.reverse()
        return _HISTORY_HEADER + "".join(items)
