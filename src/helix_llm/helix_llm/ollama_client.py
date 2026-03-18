"""
OllamaClient — synchronous HTTP client for the Ollama local API.

Uses only Python's stdlib urllib (no requests/httpx).
All methods are blocking — call from a thread pool (run_in_executor)
when used in an async context.

Ollama API:
  GET  /api/tags          — list available models (used for is_available check)
  POST /api/generate      — generate a completion (stream=False for single response)
"""
import json
import urllib.error
import urllib.request
from typing import Any, Dict

DEFAULT_BASE_URL: str = "http://localhost:11434"
DEFAULT_MODEL: str = "phi3:mini"
DEFAULT_TIMEOUT: float = 55.0  # seconds; leave headroom under 60s planner timeout

# Fallback parsed result returned when we cannot get a valid response.
_FALLBACK_RESULT: Dict[str, Any] = {
    "suggested_action": "unknown",
    "confidence": 0.0,
    "reasoning": "Failed to parse LLM response.",
}


class OllamaClient:
    """Synchronous Ollama API client."""

    def __init__(
        self,
        base_url: str = DEFAULT_BASE_URL,
        model: str = DEFAULT_MODEL,
        timeout: float = DEFAULT_TIMEOUT,
    ) -> None:
        self._base_url = base_url.rstrip("/")
        self._model = model
        self._timeout = timeout

    # ── Public API ────────────────────────────────────────────────────────────

    def is_available(self) -> bool:
        """Return True if Ollama is reachable at the configured base URL."""
        try:
            with urllib.request.urlopen(
                f"{self._base_url}/api/tags", timeout=5.0
            ):
                return True
        except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError, OSError):
            return False

    def generate(self, prompt: str) -> str:
        """
        Send a generation request to Ollama and return the raw response string.

        On any network or parse error, returns a JSON-encoded error payload that
        parse_response() will interpret as a 0-confidence fallback — callers
        should never need to catch exceptions from this method.
        """
        payload = json.dumps({
            "model": self._model,
            "prompt": prompt,
            "stream": False,
        }).encode()

        req = urllib.request.Request(
            f"{self._base_url}/api/generate",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )

        try:
            with urllib.request.urlopen(req, timeout=self._timeout) as resp:
                return resp.read().decode()
        except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError, OSError) as exc:
            return json.dumps({
                "response": json.dumps({
                    "suggested_action": "unknown",
                    "confidence": 0.0,
                    "reasoning": f"Ollama request failed: {exc}",
                }),
                "done": True,
            })

    def parse_response(self, raw: str) -> Dict[str, Any]:
        """
        Parse the raw Ollama generate response into a structured dict.

        Expected LLM output (inside the 'response' field) is a JSON object:
        {
            "suggested_action": "<action_name>",
            "confidence": <0.0-1.0>,
            "reasoning": "<explanation>"
        }

        Returns a dict with keys: suggested_action, confidence, reasoning.
        Never raises — always returns a dict (uses fallback on any error).
        """
        try:
            outer = json.loads(raw)
            inner_str = outer.get("response", "")
        except (json.JSONDecodeError, AttributeError):
            return {**_FALLBACK_RESULT, "reasoning": "Failed to parse outer Ollama JSON."}

        try:
            inner = json.loads(inner_str)
        except (json.JSONDecodeError, TypeError):
            # Inner text is not JSON — return low confidence with raw text captured
            return {
                "suggested_action": "unknown",
                "confidence": 0.0,
                "reasoning": f"LLM returned non-JSON: {inner_str[:200]}",
            }

        suggested_action = str(inner.get("suggested_action", "unknown"))
        confidence = float(inner.get("confidence", 0.0))
        confidence = max(0.0, min(1.0, confidence))  # clamp to [0, 1]
        reasoning = str(inner.get("reasoning", "No reasoning provided."))

        return {
            "suggested_action": suggested_action,
            "confidence": confidence,
            "reasoning": reasoning,
        }
