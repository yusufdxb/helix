"""
LLMAdvisor — HELIX Phase 3 LifecycleNode.

Subscribes to /helix/llm_requests (FaultEvent).
For each request:
  1. Checks Ollama availability (non-blocking, via run_in_executor).
  2. Builds a prompt with ContextBuilder.
  3. Calls OllamaClient.generate in a thread pool (so the rclpy executor
     is never blocked during the potentially slow LLM call).
  4. Parses the response and publishes LLMDiagnosis to /helix/llm_diagnoses.

Graceful degradation: if Ollama is unavailable, publishes a 0-confidence
LLMDiagnosis immediately so the RecoveryPlanner can log and move on.
"""
import asyncio
import threading
import time
from typing import Optional

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State

from helix_msgs.msg import FaultEvent, LLMDiagnosis

from helix_llm.ollama_client import OllamaClient, DEFAULT_MODEL, DEFAULT_BASE_URL
from helix_llm.context_builder import ContextBuilder

TOPIC_LLM_REQUESTS: str = "/helix/llm_requests"
TOPIC_LLM_DIAGNOSES: str = "/helix/llm_diagnoses"

DEFAULT_OLLAMA_URL: str = DEFAULT_BASE_URL
DEFAULT_OLLAMA_MODEL: str = DEFAULT_MODEL
DEFAULT_OLLAMA_TIMEOUT: float = 55.0


class LLMAdvisor(LifecycleNode):
    """
    Lifecycle node that bridges ROS 2 fault events to on-device Ollama inference.

    Threading model:
      - rclpy executor calls _on_llm_request() in the main thread.
      - Ollama HTTP call runs in a daemon asyncio event loop (background thread)
        via run_in_executor so the rclpy executor is never blocked.
      - Publishing back to /helix/llm_diagnoses is safe from the asyncio loop
        because rclpy publishers are thread-safe.
    """

    def __init__(self) -> None:
        super().__init__("helix_llm_advisor")
        self.declare_parameter("ollama_base_url", DEFAULT_OLLAMA_URL)
        self.declare_parameter("ollama_model", DEFAULT_OLLAMA_MODEL)
        self.declare_parameter("ollama_timeout_sec", DEFAULT_OLLAMA_TIMEOUT)

        self._client: Optional[OllamaClient] = None
        self._context_builder: Optional[ContextBuilder] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._loop_thread: Optional[threading.Thread] = None

        self._request_sub = None
        self._diagnosis_pub = None

    # ── Lifecycle ──────────────────────────────────────────────────────────────

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        url = self.get_parameter("ollama_base_url").value
        model = self.get_parameter("ollama_model").value
        timeout = self.get_parameter("ollama_timeout_sec").value

        self._client = OllamaClient(base_url=url, model=model, timeout=timeout)
        self._context_builder = ContextBuilder()

        self._request_sub = self.create_subscription(
            FaultEvent, TOPIC_LLM_REQUESTS, self._on_llm_request, 10
        )
        self._diagnosis_pub = self.create_publisher(LLMDiagnosis, TOPIC_LLM_DIAGNOSES, 10)

        self.get_logger().info(
            f"LLMAdvisor configured — Ollama at '{url}', model='{model}', "
            f"timeout={timeout}s"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(
            target=self._loop.run_forever, daemon=True, name="helix-llm-loop"
        )
        self._loop_thread.start()
        self.get_logger().info("LLMAdvisor activated — asyncio loop running.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        if self._loop:
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._loop_thread:
            self._loop_thread.join(timeout=5.0)
        self._loop = None
        self._loop_thread = None
        self.get_logger().info("LLMAdvisor deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        if self._request_sub:
            self.destroy_subscription(self._request_sub)
        if self._diagnosis_pub:
            self.destroy_publisher(self._diagnosis_pub)
        return TransitionCallbackReturn.SUCCESS

    # ── ROS 2 callback ────────────────────────────────────────────────────────

    def _on_llm_request(self, fault_msg: FaultEvent) -> None:
        """Received a fault event requesting LLM diagnosis."""
        if self._loop is None:
            self.get_logger().warn("LLMAdvisor not activated — dropping LLM request.")
            return
        self.get_logger().info(
            f"LLM request received: {fault_msg.fault_type} on '{fault_msg.node_name}'"
        )
        asyncio.run_coroutine_threadsafe(
            self._process_request(fault_msg), self._loop
        )

    # ── Async processing ──────────────────────────────────────────────────────

    async def _process_request(self, fault_msg: FaultEvent) -> None:
        """Async handler: calls Ollama in thread pool, publishes diagnosis."""
        loop = asyncio.get_running_loop()

        # Check availability (quick, 5s timeout, non-blocking to asyncio loop)
        available = await loop.run_in_executor(None, self._client.is_available)

        if not available:
            self.get_logger().warn(
                "Ollama unavailable — publishing 0-confidence fallback diagnosis."
            )
            self._publish_diagnosis(
                fault_id=str(fault_msg.timestamp),
                node_name=fault_msg.node_name,
                fault_type=fault_msg.fault_type,
                suggested_action="unknown",
                confidence=0.0,
                reasoning="Ollama not available at configured URL.",
                raw_response="",
            )
            return

        # Build prompt (pure Python — can run directly, no blocking I/O)
        fault_dict = {
            "node_name": fault_msg.node_name,
            "fault_type": fault_msg.fault_type,
            "severity": fault_msg.severity,
            "detail": fault_msg.detail,
            "timestamp": fault_msg.timestamp,
        }
        prompt = self._context_builder.build_prompt(fault_dict, history=[])

        # Call Ollama in thread pool (blocking HTTP, must not block event loop)
        # Use asyncio.get_running_loop() — correct in Python 3.10+ inside a coroutine
        raw = await loop.run_in_executor(None, self._client.generate, prompt)
        parsed = self._client.parse_response(raw)

        self.get_logger().info(
            f"LLM diagnosis for '{fault_msg.node_name}': "
            f"action='{parsed['suggested_action']}' confidence={parsed['confidence']:.2f}"
        )

        self._publish_diagnosis(
            fault_id=str(fault_msg.timestamp),
            node_name=fault_msg.node_name,
            fault_type=fault_msg.fault_type,
            suggested_action=parsed["suggested_action"],
            confidence=float(parsed["confidence"]),
            reasoning=parsed["reasoning"],
            raw_response=raw[:500],  # cap to avoid huge messages
        )

    def _publish_diagnosis(
        self,
        fault_id: str,
        node_name: str,
        fault_type: str,
        suggested_action: str,
        confidence: float,
        reasoning: str,
        raw_response: str,
    ) -> None:
        msg = LLMDiagnosis()
        msg.fault_id = fault_id
        msg.node_name = node_name
        msg.fault_type = fault_type
        msg.suggested_action = suggested_action
        msg.confidence = confidence
        msg.reasoning = reasoning
        msg.raw_response = raw_response
        msg.timestamp = time.time()
        self._diagnosis_pub.publish(msg)


def main(args=None) -> None:
    """Entry point for helix_llm_advisor console script."""
    rclpy.init(args=args)
    node = LLMAdvisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
