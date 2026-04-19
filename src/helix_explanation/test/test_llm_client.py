"""Unit tests for helix_explanation.llm_client.

Pure-Python tests — do not require ROS 2. Use unittest.mock to fake
requests so no real HTTP or llama-server is needed.
"""
from __future__ import annotations

import json
from unittest.mock import MagicMock

import pytest
import requests

from helix_explanation.llm_client import (
    AsyncLLMClient,
    HEALER_SCHEMA,
    LLMClient,
    LLMClientError,
    LLMHTTPError,
    LLMInvalidResponseError,
    LLMRequest,
    LLMResponse,
    LLMRole,
    LLMTimeoutError,
    LLMTransportError,
    RECOVERY_ALLOWLIST,
    _validate_against_schema,
)


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def _mock_response(content_obj, status_code=200):
    resp = MagicMock(spec=requests.Response)
    resp.status_code = status_code
    resp.text = ''
    if status_code >= 400:
        resp.text = json.dumps({'error': content_obj})
        resp.json = MagicMock(return_value={'error': content_obj})
        return resp
    resp.json = MagicMock(return_value={
        'id': 'cmpl-test',
        'model': 'qwen2.5-1.5b-instruct',
        'choices': [
            {
                'index': 0,
                'message': {'role': 'assistant', 'content': json.dumps(content_obj)},
                'finish_reason': 'stop',
            }
        ],
    })
    return resp


def _healer_ok_payload():
    return {
        'action': 'STOP_AND_HOLD',
        'confidence': 0.87,
        'reasoning': 'utlidar_cloud rate dropped to 3 Hz, below baseline',
    }


def _flagger_ok_payload():
    return {'novel': True, 'why': 'dds qos warning paired with rate drop'}


def _predictor_ok_payload():
    return {
        'precursor_detected': True,
        'what': 'sustained sub-nominal lidar rate',
        'confidence': 0.66,
    }


def _healer_request():
    return LLMRequest(
        role=LLMRole.HEALER,
        fault_event={
            'node_name': 'anomaly_detector',
            'fault_type': 'ANOMALY',
            'severity': 2,
            'detail': 'rate_hz/utlidar_cloud Z=8.4',
        },
        context_snapshot={
            'rosout_recent': [{'level': 40, 'name': 'dds', 'msg': 'QoS mismatch'}],
            'metric_rates': {'utlidar_cloud': {'current_hz': 3.1, 'mean_hz_10s': 15.3}},
        },
    )


# ---------------------------------------------------------------------------
# schema validator
# ---------------------------------------------------------------------------


class TestValidator:
    def test_healer_valid(self):
        _validate_against_schema(_healer_ok_payload(), HEALER_SCHEMA)

    def test_healer_missing_field(self):
        bad = _healer_ok_payload()
        del bad['confidence']
        with pytest.raises(LLMInvalidResponseError, match='missing required key'):
            _validate_against_schema(bad, HEALER_SCHEMA)

    def test_healer_action_not_in_enum(self):
        bad = _healer_ok_payload()
        bad['action'] = 'RESTART_NODE'  # not in RecoveryNode allowlist
        with pytest.raises(LLMInvalidResponseError, match='not in enum'):
            _validate_against_schema(bad, HEALER_SCHEMA)

    def test_healer_confidence_out_of_range(self):
        bad = _healer_ok_payload()
        bad['confidence'] = 1.5
        with pytest.raises(LLMInvalidResponseError, match='above maximum'):
            _validate_against_schema(bad, HEALER_SCHEMA)

    def test_healer_reasoning_too_long(self):
        bad = _healer_ok_payload()
        bad['reasoning'] = 'x' * 300
        with pytest.raises(LLMInvalidResponseError, match='too long'):
            _validate_against_schema(bad, HEALER_SCHEMA)

    def test_healer_extra_property_rejected(self):
        bad = _healer_ok_payload()
        bad['hallucinated_field'] = 'oops'
        with pytest.raises(LLMInvalidResponseError, match='unexpected key'):
            _validate_against_schema(bad, HEALER_SCHEMA)

    def test_allowlist_matches_recovery_node(self):
        # Regression guard: keep in sync with helix_recovery allowlist.
        assert set(RECOVERY_ALLOWLIST) == {'STOP_AND_HOLD', 'RESUME', 'LOG_ONLY'}


# ---------------------------------------------------------------------------
# LLMClient happy paths
# ---------------------------------------------------------------------------


class TestClientHappyPath:
    def test_healer_ok(self):
        session = MagicMock()
        session.post.return_value = _mock_response(_healer_ok_payload())
        client = LLMClient(session=session, timeout_s=6.0)
        resp = client.complete(_healer_request())
        assert isinstance(resp, LLMResponse)
        assert resp.role == LLMRole.HEALER
        assert resp.data['action'] in RECOVERY_ALLOWLIST
        assert 0 <= resp.data['confidence'] <= 1

    def test_flagger_ok(self):
        session = MagicMock()
        session.post.return_value = _mock_response(_flagger_ok_payload())
        client = LLMClient(session=session)
        resp = client.complete(LLMRequest(role=LLMRole.FLAGGER, context_snapshot={'x': 1}))
        assert resp.data['novel'] is True

    def test_predictor_ok(self):
        session = MagicMock()
        session.post.return_value = _mock_response(_predictor_ok_payload())
        client = LLMClient(session=session)
        resp = client.complete(LLMRequest(role=LLMRole.PREDICTOR, context_snapshot={'x': 1}))
        assert resp.data['precursor_detected'] is True
        assert 0 <= resp.data['confidence'] <= 1

    def test_payload_structure_has_response_format_json_schema(self):
        """Regression: we must send json_schema so llama-server constrains output."""
        session = MagicMock()
        session.post.return_value = _mock_response(_healer_ok_payload())
        client = LLMClient(session=session)
        client.complete(_healer_request())

        sent_payload = session.post.call_args.kwargs['json']
        assert sent_payload['response_format']['type'] == 'json_schema'
        assert 'schema' in sent_payload['response_format']['json_schema']
        # system + user
        assert len(sent_payload['messages']) == 2
        assert sent_payload['messages'][0]['role'] == 'system'

    def test_timeout_forwarded_to_requests(self):
        session = MagicMock()
        session.post.return_value = _mock_response(_healer_ok_payload())
        client = LLMClient(session=session, timeout_s=2.5)
        client.complete(_healer_request())
        assert session.post.call_args.kwargs['timeout'] == 2.5


# ---------------------------------------------------------------------------
# LLMClient error paths
# ---------------------------------------------------------------------------


class TestClientErrors:
    def test_schema_invalid_response_rejected(self):
        bad = {'action': 'BLOW_UP_ROBOT', 'confidence': 0.9, 'reasoning': 'yolo'}
        session = MagicMock()
        session.post.return_value = _mock_response(bad)
        client = LLMClient(session=session)
        with pytest.raises(LLMInvalidResponseError):
            client.complete(_healer_request())

    def test_timeout(self):
        session = MagicMock()
        session.post.side_effect = requests.exceptions.Timeout('slow')
        client = LLMClient(session=session, timeout_s=1.0)
        with pytest.raises(LLMTimeoutError):
            client.complete(_healer_request())

    def test_http_500(self):
        session = MagicMock()
        session.post.return_value = _mock_response(
            {'message': 'model not loaded'}, status_code=500
        )
        client = LLMClient(session=session)
        with pytest.raises(LLMHTTPError) as excinfo:
            client.complete(_healer_request())
        assert excinfo.value.status_code == 500

    def test_connection_error(self):
        session = MagicMock()
        session.post.side_effect = requests.exceptions.ConnectionError('refused')
        client = LLMClient(session=session)
        with pytest.raises(LLMTransportError):
            client.complete(_healer_request())

    def test_missing_choices(self):
        resp = MagicMock(spec=requests.Response)
        resp.status_code = 200
        resp.text = '{}'
        resp.json = MagicMock(return_value={'no_choices': True})
        session = MagicMock()
        session.post.return_value = resp
        client = LLMClient(session=session)
        with pytest.raises(LLMInvalidResponseError):
            client.complete(_healer_request())

    def test_non_json_content(self):
        resp = MagicMock(spec=requests.Response)
        resp.status_code = 200
        resp.text = 'ok'
        resp.json = MagicMock(return_value={
            'choices': [{'message': {'content': 'this is not json'}}],
        })
        session = MagicMock()
        session.post.return_value = resp
        client = LLMClient(session=session)
        with pytest.raises(LLMInvalidResponseError):
            client.complete(_healer_request())

    def test_non_json_body(self):
        resp = MagicMock(spec=requests.Response)
        resp.status_code = 200
        resp.text = 'nope'
        resp.json = MagicMock(side_effect=ValueError('no json'))
        session = MagicMock()
        session.post.return_value = resp
        client = LLMClient(session=session)
        with pytest.raises(LLMInvalidResponseError):
            client.complete(_healer_request())


# ---------------------------------------------------------------------------
# AsyncLLMClient + fallback behavior
# ---------------------------------------------------------------------------


class TestAsyncClient:
    def test_submit_returns_future_with_valid_response(self):
        session = MagicMock()
        session.post.return_value = _mock_response(_healer_ok_payload())
        async_client = AsyncLLMClient(LLMClient(session=session))
        try:
            fut = async_client.submit(_healer_request())
            resp = fut.result(timeout=5)
            assert isinstance(resp, LLMResponse)
            assert resp.data['action'] == 'STOP_AND_HOLD'
        finally:
            async_client.shutdown()

    def test_fallback_runs_on_llm_error(self):
        """Timeout path → fallback invoked, future resolves to fallback value."""
        session = MagicMock()
        session.post.side_effect = requests.exceptions.Timeout('slow')
        async_client = AsyncLLMClient(LLMClient(session=session, timeout_s=0.1))
        try:
            calls: list = []

            def fallback(req, err):
                calls.append((req.role, type(err).__name__))
                return 'deterministic-template-string'

            fut = async_client.submit_with_fallback(_healer_request(), fallback)
            result = fut.result(timeout=5)
            assert result == 'deterministic-template-string'
            assert calls == [(LLMRole.HEALER, 'LLMTimeoutError')]
        finally:
            async_client.shutdown()

    def test_fallback_not_invoked_on_success(self):
        session = MagicMock()
        session.post.return_value = _mock_response(_healer_ok_payload())
        async_client = AsyncLLMClient(LLMClient(session=session))
        try:
            def fallback(req, err):  # pragma: no cover
                raise AssertionError('fallback should not be called on success')

            fut = async_client.submit_with_fallback(_healer_request(), fallback)
            resp = fut.result(timeout=5)
            assert isinstance(resp, LLMResponse)
        finally:
            async_client.shutdown()

    def test_unknown_role_rejected(self):
        session = MagicMock()
        client = LLMClient(session=session)

        class FakeRole:
            value = 'bogus'

        with pytest.raises(LLMClientError):
            client.complete(LLMRequest(role=FakeRole(), context_snapshot={}))  # type: ignore[arg-type]
