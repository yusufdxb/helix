"""
Tests for ContextBuilder — pure Python, no ROS, no Ollama required.
"""
import unittest

from helix_llm.context_builder import ContextBuilder, MAX_PROMPT_TOKENS


class TestContextBuilderConstants(unittest.TestCase):
    def test_max_prompt_tokens_is_reasonable(self):
        self.assertGreater(MAX_PROMPT_TOKENS, 500)
        self.assertLessEqual(MAX_PROMPT_TOKENS, 4096)


class TestEstimateTokens(unittest.TestCase):
    def setUp(self):
        self.cb = ContextBuilder()

    def test_empty_string(self):
        self.assertEqual(self.cb.estimate_tokens(""), 0)

    def test_rough_estimate_four_chars_per_token(self):
        text = "a" * 400  # 400 chars → ~100 tokens
        tokens = self.cb.estimate_tokens(text)
        self.assertAlmostEqual(tokens, 100, delta=10)

    def test_long_text_estimate(self):
        text = "word " * 1000  # 5000 chars → ~1250 tokens
        tokens = self.cb.estimate_tokens(text)
        self.assertGreater(tokens, 1000)


class TestBuildPrompt(unittest.TestCase):
    def setUp(self):
        self.cb = ContextBuilder()
        self.fault = {
            "node_name": "fake_slam_node",
            "fault_type": "LOG_PATTERN",
            "severity": 3,
            "detail": "SLAM diverged: covariance exceeded maximum threshold",
            "timestamp": 1700000000.0,
        }

    def test_prompt_contains_fault_node_name(self):
        prompt = self.cb.build_prompt(self.fault, [])
        self.assertIn("fake_slam_node", prompt)

    def test_prompt_contains_fault_type(self):
        prompt = self.cb.build_prompt(self.fault, [])
        self.assertIn("LOG_PATTERN", prompt)

    def test_prompt_contains_fault_detail(self):
        prompt = self.cb.build_prompt(self.fault, [])
        self.assertIn("SLAM diverged", prompt)

    def test_prompt_requests_json_output(self):
        """Prompt must ask LLM to respond in JSON with the expected keys."""
        prompt = self.cb.build_prompt(self.fault, [])
        self.assertIn("suggested_action", prompt)
        self.assertIn("confidence", prompt)
        self.assertIn("reasoning", prompt)

    def test_prompt_contains_history_when_provided(self):
        history = [
            {"action_taken": "restart_node", "success": True, "tier": 1},
            {"action_taken": "activate_safe_mode", "success": False, "tier": 2},
        ]
        prompt = self.cb.build_prompt(self.fault, history)
        self.assertIn("restart_node", prompt)
        self.assertIn("activate_safe_mode", prompt)

    def test_prompt_empty_history_does_not_crash(self):
        prompt = self.cb.build_prompt(self.fault, [])
        self.assertIsInstance(prompt, str)
        self.assertGreater(len(prompt), 50)

    def test_prompt_truncates_excessive_history(self):
        """When history is very long, prompt stays within token limit."""
        # 200 history items — each with verbose text
        long_history = [
            {
                "action_taken": "restart_node",
                "success": False,
                "tier": 1,
                "outcome_detail": "x" * 200,
            }
            for _ in range(200)
        ]
        prompt = self.cb.build_prompt(self.fault, long_history)
        tokens = self.cb.estimate_tokens(prompt)
        self.assertLessEqual(tokens, MAX_PROMPT_TOKENS + 50)  # small overshoot allowed

    def test_prompt_lists_valid_actions(self):
        """Prompt should enumerate the valid action names so LLM can pick one."""
        prompt = self.cb.build_prompt(self.fault, [])
        self.assertIn("restart_node", prompt)
        self.assertIn("activate_safe_mode", prompt)

    def test_different_fault_types_produce_different_prompts(self):
        crash_fault = {**self.fault, "fault_type": "CRASH", "detail": "Node process died"}
        prompt_log = self.cb.build_prompt(self.fault, [])
        prompt_crash = self.cb.build_prompt(crash_fault, [])
        self.assertNotEqual(prompt_log, prompt_crash)


if __name__ == "__main__":
    unittest.main()
