"""
Tests for OllamaClient — all offline, no running Ollama required.

Mocks urllib.request.urlopen at the module level so the client
never makes real HTTP calls.
"""
import json
import unittest
from unittest.mock import MagicMock, patch
from urllib.error import HTTPError, URLError
from io import BytesIO

from helix_llm.ollama_client import OllamaClient, DEFAULT_MODEL, DEFAULT_BASE_URL


def _make_urlopen_response(body: dict | str, status: int = 200):
    """Helper: create a mock urlopen response with read() and status."""
    if isinstance(body, dict):
        body = json.dumps(body)
    mock_resp = MagicMock()
    mock_resp.read.return_value = body.encode()
    mock_resp.status = status
    mock_resp.__enter__ = lambda s: s
    mock_resp.__exit__ = MagicMock(return_value=False)
    return mock_resp


class TestOllamaClientConstants(unittest.TestCase):
    def test_default_model(self):
        self.assertEqual(DEFAULT_MODEL, "phi3:mini")

    def test_default_base_url(self):
        self.assertEqual(DEFAULT_BASE_URL, "http://localhost:11434")


class TestParseResponse(unittest.TestCase):
    """parse_response parses Ollama generate API response JSON."""

    def setUp(self):
        self.client = OllamaClient()

    def test_parse_valid_structured_json(self):
        """LLM returned valid JSON with all expected fields."""
        inner = {
            "suggested_action": "restart_node",
            "confidence": 0.9,
            "reasoning": "Node crashed, restart is standard recovery.",
        }
        raw = json.dumps({"response": json.dumps(inner), "done": True})
        result = self.client.parse_response(raw)
        self.assertEqual(result["suggested_action"], "restart_node")
        self.assertAlmostEqual(result["confidence"], 0.9, places=3)
        self.assertIn("restart", result["reasoning"])

    def test_parse_invalid_outer_json(self):
        """Completely invalid JSON returns low-confidence fallback."""
        result = self.client.parse_response("not json at all")
        self.assertEqual(result["confidence"], 0.0)
        self.assertEqual(result["suggested_action"], "unknown")
        self.assertIn("parse", result["reasoning"].lower())

    def test_parse_inner_response_not_json(self):
        """Outer JSON valid but inner 'response' is plain text — extract what we can."""
        raw = json.dumps({"response": "I think you should restart the node.", "done": True})
        result = self.client.parse_response(raw)
        # Should return low confidence and capture raw text in reasoning
        self.assertLess(result["confidence"], 0.5)
        self.assertIsInstance(result["suggested_action"], str)

    def test_parse_missing_confidence_defaults_to_zero(self):
        """Inner JSON missing 'confidence' key → defaults to 0.0."""
        inner = {"suggested_action": "activate_safe_mode", "reasoning": "Safety first."}
        raw = json.dumps({"response": json.dumps(inner), "done": True})
        result = self.client.parse_response(raw)
        self.assertEqual(result["confidence"], 0.0)

    def test_parse_confidence_clamped_to_one(self):
        """Confidence > 1.0 is clamped to 1.0."""
        inner = {"suggested_action": "restart_node", "confidence": 1.5, "reasoning": "x"}
        raw = json.dumps({"response": json.dumps(inner), "done": True})
        result = self.client.parse_response(raw)
        self.assertLessEqual(result["confidence"], 1.0)

    def test_parse_confidence_clamped_to_zero(self):
        """Negative confidence is clamped to 0.0."""
        inner = {"suggested_action": "restart_node", "confidence": -0.3, "reasoning": "x"}
        raw = json.dumps({"response": json.dumps(inner), "done": True})
        result = self.client.parse_response(raw)
        self.assertGreaterEqual(result["confidence"], 0.0)


class TestIsAvailable(unittest.TestCase):
    """is_available checks /api/tags endpoint."""

    def setUp(self):
        self.client = OllamaClient()

    @patch("helix_llm.ollama_client.urllib.request.urlopen")
    def test_returns_true_on_200(self, mock_urlopen):
        mock_urlopen.return_value = _make_urlopen_response({"models": []})
        self.assertTrue(self.client.is_available())

    @patch("helix_llm.ollama_client.urllib.request.urlopen")
    def test_returns_false_on_http_error(self, mock_urlopen):
        mock_urlopen.side_effect = HTTPError(
            url="http://localhost:11434/api/tags",
            code=500,
            msg="Internal Server Error",
            hdrs=None,
            fp=None,
        )
        self.assertFalse(self.client.is_available())

    @patch("helix_llm.ollama_client.urllib.request.urlopen")
    def test_returns_false_on_url_error(self, mock_urlopen):
        mock_urlopen.side_effect = URLError("Connection refused")
        self.assertFalse(self.client.is_available())

    @patch("helix_llm.ollama_client.urllib.request.urlopen")
    def test_returns_false_on_timeout(self, mock_urlopen):
        mock_urlopen.side_effect = TimeoutError("timed out")
        self.assertFalse(self.client.is_available())


class TestGenerate(unittest.TestCase):
    """generate sends prompt to /api/generate and returns raw response string."""

    def setUp(self):
        self.client = OllamaClient()

    @patch("helix_llm.ollama_client.urllib.request.urlopen")
    def test_returns_response_string_on_success(self, mock_urlopen):
        inner = {"suggested_action": "restart_node", "confidence": 0.85, "reasoning": "ok"}
        api_resp = {"response": json.dumps(inner), "done": True}
        mock_urlopen.return_value = _make_urlopen_response(api_resp)
        raw = self.client.generate("some prompt")
        self.assertIn("restart_node", raw)

    @patch("helix_llm.ollama_client.urllib.request.urlopen")
    def test_returns_error_string_on_url_error(self, mock_urlopen):
        mock_urlopen.side_effect = URLError("Connection refused")
        raw = self.client.generate("some prompt")
        # Should return a JSON string with error info, not raise
        self.assertIsInstance(raw, str)
        result = self.client.parse_response(raw)
        self.assertEqual(result["confidence"], 0.0)

    @patch("helix_llm.ollama_client.urllib.request.urlopen")
    def test_returns_error_string_on_timeout(self, mock_urlopen):
        mock_urlopen.side_effect = TimeoutError("timed out")
        raw = self.client.generate("some prompt")
        self.assertIsInstance(raw, str)
        result = self.client.parse_response(raw)
        self.assertEqual(result["confidence"], 0.0)

    @patch("helix_llm.ollama_client.urllib.request.urlopen")
    def test_sends_correct_model_and_stream_false(self, mock_urlopen):
        """Verify the request body includes model name and stream=False."""
        inner = {"suggested_action": "activate_safe_mode", "confidence": 0.7, "reasoning": "y"}
        mock_urlopen.return_value = _make_urlopen_response(
            {"response": json.dumps(inner), "done": True}
        )
        self.client.generate("test prompt")
        call_args = mock_urlopen.call_args
        # First positional arg is the Request object
        req = call_args[0][0]
        body = json.loads(req.data.decode())
        self.assertEqual(body["model"], DEFAULT_MODEL)
        self.assertFalse(body["stream"])
        self.assertEqual(body["prompt"], "test prompt")


if __name__ == "__main__":
    unittest.main()
