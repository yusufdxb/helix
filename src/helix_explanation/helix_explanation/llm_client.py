"""
HELIX local-LLM client — pure logic, no ROS imports.

Talks to a llama.cpp `llama-server` sidecar over the OpenAI-compatible
`/v1/chat/completions` endpoint. Default port 8080 on localhost.

Three advisory roles:
  * healer    — given a FaultEvent + rosout ring, propose a recovery
                action from the RecoveryNode allowlist.
  * flagger   — given a rolling context window, flag novelty.
  * predictor — given a short history, flag failure precursors.

Output shape is constrained at the server side via OpenAI-style
`response_format: json_schema` (preferred on modern llama.cpp builds,
which map the field onto their GBNF engine internally). Client-side
we additionally validate the parsed JSON against the declared role
schema so malformed/ignored-constraint output is rejected before it
leaves this module.

This module MUST remain ROS-free. It is unit-tested by mocking the
HTTP layer.

Safety contract
---------------
LLM outputs are ADVISORY ONLY. The RecoveryNode allowlist (
STOP_AND_HOLD / RESUME / LOG_ONLY) is the hard gate and is enforced
downstream — not in this client. A call that returns successfully
here has passed structural validation; it still must be treated as
a second opinion on top of the deterministic rule-based path.
"""
from __future__ import annotations

import json
import logging
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, Mapping, Optional

import requests

LOG = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Role enum + schemas
# ---------------------------------------------------------------------------


class LLMRole(str, Enum):
    """Three advisory prompt roles per survey §4."""
    HEALER = 'healer'
    FLAGGER = 'flagger'
    PREDICTOR = 'predictor'


# RecoveryNode hard allowlist. Kept in sync with
# src/helix_recovery/helix_recovery/recovery_node.py. If you change this
# here without matching there the LLM may suggest actions the
# downstream gate will reject — which is fine safety-wise but wastes a
# round trip.
RECOVERY_ALLOWLIST = ('STOP_AND_HOLD', 'RESUME', 'LOG_ONLY')

HEALER_SCHEMA: Dict[str, Any] = {
    'type': 'object',
    'additionalProperties': False,
    'properties': {
        'action': {'type': 'string', 'enum': list(RECOVERY_ALLOWLIST)},
        'confidence': {'type': 'number', 'minimum': 0.0, 'maximum': 1.0},
        'reasoning': {'type': 'string', 'minLength': 1, 'maxLength': 240},
    },
    'required': ['action', 'confidence', 'reasoning'],
}

FLAGGER_SCHEMA: Dict[str, Any] = {
    'type': 'object',
    'additionalProperties': False,
    'properties': {
        'novel': {'type': 'boolean'},
        'why': {'type': 'string', 'minLength': 1, 'maxLength': 240},
    },
    'required': ['novel', 'why'],
}

PREDICTOR_SCHEMA: Dict[str, Any] = {
    'type': 'object',
    'additionalProperties': False,
    'properties': {
        'precursor_detected': {'type': 'boolean'},
        'what': {'type': 'string', 'minLength': 1, 'maxLength': 240},
        'confidence': {'type': 'number', 'minimum': 0.0, 'maximum': 1.0},
    },
    'required': ['precursor_detected', 'what', 'confidence'],
}

_SCHEMAS: Dict[LLMRole, Dict[str, Any]] = {
    LLMRole.HEALER: HEALER_SCHEMA,
    LLMRole.FLAGGER: FLAGGER_SCHEMA,
    LLMRole.PREDICTOR: PREDICTOR_SCHEMA,
}


_SYSTEM_PROMPTS: Dict[LLMRole, str] = {
    LLMRole.HEALER: (
        "You are HELIX's recovery advisor for a Unitree GO2 quadruped "
        "on ROS 2. You receive a FaultEvent and a recent rosout / metric "
        "context snapshot. Choose exactly one recovery action from this "
        "allowlist:\n"
        "  - STOP_AND_HOLD  (safe stop + hold pose for critical faults)\n"
        "  - RESUME         (clear transient fault and continue)\n"
        "  - LOG_ONLY       (no actuation; record only)\n"
        "Respond ONLY with JSON matching the provided schema. Do not "
        "invent actions. If unsure, return LOG_ONLY with low confidence. "
        "Your output is advisory only; a deterministic allowlist "
        "enforcer is the hard gate."
    ),
    LLMRole.FLAGGER: (
        "You are a novelty detector for HELIX. Given a rolling window of "
        "recent /rosout lines, per-topic metric rates, and node health, "
        "decide if anything in the window looks unusual compared to "
        "healthy steady-state operation of a GO2 + Nav2 stack. Respond "
        "with JSON only, matching the provided schema."
    ),
    LLMRole.PREDICTOR: (
        "You are a failure-precursor watcher for HELIX. Given the recent "
        "rolling window, identify any pattern that commonly precedes "
        "these known failure modes:\n"
        "  - sensor_dropout (topic silence)\n"
        "  - rate_degradation (sustained sub-nominal topic rate)\n"
        "  - dds_partition (QoS / discovery warnings in /rosout)\n"
        "If none are present, set precursor_detected=false. Respond with "
        "JSON only, matching the provided schema."
    ),
}


# ---------------------------------------------------------------------------
# Errors
# ---------------------------------------------------------------------------


class LLMClientError(Exception):
    """Base class for every llm_client failure mode."""


class LLMTimeoutError(LLMClientError):
    """HTTP request did not complete within the configured timeout."""


class LLMHTTPError(LLMClientError):
    """llama-server returned a non-2xx status."""

    def __init__(self, status_code: int, body: str):
        super().__init__(f'llama-server HTTP {status_code}: {body[:200]}')
        self.status_code = status_code
        self.body = body


class LLMTransportError(LLMClientError):
    """Network / connection failure before a response was received."""


class LLMInvalidResponseError(LLMClientError):
    """Response arrived but the JSON was malformed or schema-invalid."""


# ---------------------------------------------------------------------------
# Minimal JSON-schema validator (covers the subset our schemas use)
# ---------------------------------------------------------------------------


def _validate_against_schema(instance: Any, schema: Mapping[str, Any]) -> None:
    """Tiny, dependency-free validator for the subset of JSON schema we use.

    Supports: type (object/string/boolean/number), enum, minimum/maximum,
    minLength/maxLength, required, properties, additionalProperties=False.
    Raises LLMInvalidResponseError on first violation.
    """
    t = schema.get('type')
    if t == 'object':
        if not isinstance(instance, dict):
            raise LLMInvalidResponseError(f'expected object, got {type(instance).__name__}')
        required = schema.get('required', [])
        for key in required:
            if key not in instance:
                raise LLMInvalidResponseError(f'missing required key: {key}')
        props = schema.get('properties', {})
        if schema.get('additionalProperties') is False:
            for key in instance:
                if key not in props:
                    raise LLMInvalidResponseError(f'unexpected key: {key}')
        for key, subschema in props.items():
            if key in instance:
                _validate_against_schema(instance[key], subschema)
    elif t == 'string':
        if not isinstance(instance, str):
            raise LLMInvalidResponseError(f'expected string, got {type(instance).__name__}')
        if 'minLength' in schema and len(instance) < schema['minLength']:
            raise LLMInvalidResponseError('string too short')
        if 'maxLength' in schema and len(instance) > schema['maxLength']:
            raise LLMInvalidResponseError('string too long')
        if 'enum' in schema and instance not in schema['enum']:
            raise LLMInvalidResponseError(
                f'value {instance!r} not in enum {schema["enum"]}'
            )
    elif t == 'boolean':
        if not isinstance(instance, bool):
            raise LLMInvalidResponseError(f'expected bool, got {type(instance).__name__}')
    elif t == 'number':
        # Note: bool is a subclass of int in Python — explicitly reject.
        if isinstance(instance, bool) or not isinstance(instance, (int, float)):
            raise LLMInvalidResponseError(f'expected number, got {type(instance).__name__}')
        if 'minimum' in schema and instance < schema['minimum']:
            raise LLMInvalidResponseError(f'value {instance} below minimum {schema["minimum"]}')
        if 'maximum' in schema and instance > schema['maximum']:
            raise LLMInvalidResponseError(f'value {instance} above maximum {schema["maximum"]}')


# ---------------------------------------------------------------------------
# Client
# ---------------------------------------------------------------------------


@dataclass
class LLMRequest:
    """User-facing request payload. Role picks prompt + schema."""
    role: LLMRole
    fault_event: Optional[Mapping[str, Any]] = None
    context_snapshot: Mapping[str, Any] = field(default_factory=dict)


@dataclass
class LLMResponse:
    """Parsed, schema-validated response."""
    role: LLMRole
    data: Dict[str, Any]
    raw_content: str
    latency_ms: float


class LLMClient:
    """Thin, synchronous wrapper over llama-server `/v1/chat/completions`.

    Designed to be called from a ThreadPoolExecutor so the ROS 2
    executor never blocks on the HTTP round-trip. Not thread-safe
    on a single instance beyond what requests.Session provides, but
    the ROS node keeps a single-worker pool by default so this
    doesn't matter in practice.
    """

    def __init__(
        self,
        base_url: str = 'http://localhost:8080',
        model: str = 'qwen2.5-1.5b-instruct',
        timeout_s: float = 6.0,
        session: Optional[requests.Session] = None,
        max_tokens: int = 192,
        temperature: float = 0.2,
    ):
        self._base_url = base_url.rstrip('/')
        self._model = model
        self._timeout = float(timeout_s)
        self._session = session or requests.Session()
        self._max_tokens = int(max_tokens)
        self._temperature = float(temperature)

    @property
    def base_url(self) -> str:
        return self._base_url

    @property
    def model(self) -> str:
        return self._model

    # -- request building ---------------------------------------------------

    def _build_payload(self, req: LLMRequest) -> Dict[str, Any]:
        schema = _SCHEMAS[req.role]
        system = _SYSTEM_PROMPTS[req.role]
        user_body: Dict[str, Any] = {
            'context_snapshot': dict(req.context_snapshot or {}),
        }
        if req.fault_event is not None:
            user_body['fault_event'] = dict(req.fault_event)
        return {
            'model': self._model,
            'messages': [
                {'role': 'system', 'content': system},
                {'role': 'user', 'content': json.dumps(user_body, default=str)},
            ],
            # llama-server accepts the OpenAI-style response_format and
            # maps json_schema onto its GBNF engine internally — pick
            # json_schema because it's the stable OpenAI-compat surface,
            # not the raw llama.cpp `grammar` field (which is equally
            # supported but undocumented from the API caller's side).
            'response_format': {
                'type': 'json_schema',
                'json_schema': {
                    'name': f'helix_{req.role.value}',
                    'schema': schema,
                    'strict': True,
                },
            },
            'temperature': self._temperature,
            'max_tokens': self._max_tokens,
            'stream': False,
        }

    # -- main entry ---------------------------------------------------------

    def complete(self, req: LLMRequest) -> LLMResponse:
        """Blocking call. Raises LLMClientError subclasses on any failure."""
        if req.role not in _SCHEMAS:
            raise LLMClientError(f'unknown role: {req.role}')
        url = f'{self._base_url}/v1/chat/completions'
        payload = self._build_payload(req)

        import time
        start = time.monotonic()
        try:
            resp = self._session.post(url, json=payload, timeout=self._timeout)
        except requests.exceptions.Timeout as exc:
            raise LLMTimeoutError(f'llama-server timeout after {self._timeout}s') from exc
        except requests.exceptions.ConnectionError as exc:
            raise LLMTransportError(f'connection error: {exc}') from exc
        except requests.exceptions.RequestException as exc:
            raise LLMTransportError(f'request error: {exc}') from exc
        elapsed_ms = (time.monotonic() - start) * 1000.0

        if resp.status_code >= 400:
            raise LLMHTTPError(resp.status_code, resp.text or '')

        try:
            body = resp.json()
        except ValueError as exc:
            raise LLMInvalidResponseError(f'non-JSON response body: {exc}') from exc

        content = self._extract_content(body)
        try:
            parsed = json.loads(content)
        except (TypeError, ValueError) as exc:
            raise LLMInvalidResponseError(f'content is not JSON: {exc}') from exc

        _validate_against_schema(parsed, _SCHEMAS[req.role])
        return LLMResponse(
            role=req.role,
            data=parsed,
            raw_content=content,
            latency_ms=elapsed_ms,
        )

    @staticmethod
    def _extract_content(body: Mapping[str, Any]) -> str:
        try:
            choices = body['choices']
            first = choices[0]
            msg = first['message']
            content = msg['content']
        except (KeyError, IndexError, TypeError) as exc:
            raise LLMInvalidResponseError(f'missing choices[0].message.content: {exc}') from exc
        if not isinstance(content, str):
            raise LLMInvalidResponseError('content is not a string')
        return content


# ---------------------------------------------------------------------------
# Async wrapper — used by the ROS node
# ---------------------------------------------------------------------------


class AsyncLLMClient:
    """Wraps LLMClient in a ThreadPoolExecutor so callers never block.

    Intended for use inside a ROS 2 Python node: submit a request
    from the subscription callback, attach a done_callback, let the
    executor publish the resulting advisory message on whatever
    topic the node exposes.
    """

    def __init__(self, client: LLMClient, max_workers: int = 1):
        self._client = client
        # Single worker by default — we don't want two inferences in
        # flight on a 2 GB RAM budget.
        self._pool = ThreadPoolExecutor(
            max_workers=max_workers,
            thread_name_prefix='helix_llm',
        )

    @property
    def client(self) -> LLMClient:
        return self._client

    def submit(self, req: LLMRequest) -> 'Future[LLMResponse]':
        return self._pool.submit(self._client.complete, req)

    def submit_with_fallback(
        self,
        req: LLMRequest,
        fallback: Callable[[LLMRequest, Exception], Any],
    ) -> 'Future[Any]':
        """Submit and, on any LLMClientError, return `fallback(req, err)`.

        Use this to keep the deterministic template path alive when
        the LLM is down or slow: pass a callable that re-runs
        render_template and returns the string.
        """

        def _wrapped() -> Any:
            try:
                return self._client.complete(req)
            except LLMClientError as exc:
                LOG.warning('LLM call failed for role=%s: %s', req.role.value, exc)
                return fallback(req, exc)

        return self._pool.submit(_wrapped)

    def shutdown(self, wait: bool = True) -> None:
        self._pool.shutdown(wait=wait)
