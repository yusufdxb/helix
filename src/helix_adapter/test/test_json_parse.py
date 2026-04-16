"""Unit tests for json_parse helpers (pure-function, no ROS)."""

from helix_adapter.json_parse import (
    extract_numeric_fields,
    parse_bool_metric,
    safe_float,
    try_load_json,
)


def test_safe_float_valid():
    assert safe_float(3.14) == 3.14
    assert safe_float("2.5") == 2.5
    assert safe_float(0) == 0.0
    assert safe_float("-0.5") == -0.5


def test_safe_float_rejects_nan_inf():
    assert safe_float(float("nan")) is None
    assert safe_float(float("inf")) is None
    assert safe_float(float("-inf")) is None


def test_safe_float_rejects_unparseable():
    assert safe_float("abc") is None
    assert safe_float(None) is None
    assert safe_float([1, 2]) is None
    assert safe_float({}) is None


def test_parse_bool_from_bool():
    assert parse_bool_metric(True) == 1.0
    assert parse_bool_metric(False) == 0.0


def test_parse_bool_from_int():
    assert parse_bool_metric(0) == 0.0
    assert parse_bool_metric(1) == 1.0
    assert parse_bool_metric(-3) == 1.0


def test_parse_bool_from_string():
    assert parse_bool_metric("true") == 1.0
    assert parse_bool_metric("True") == 1.0
    assert parse_bool_metric("1") == 1.0
    assert parse_bool_metric("false") == 0.0
    assert parse_bool_metric("False") == 0.0
    assert parse_bool_metric("0") == 0.0
    assert parse_bool_metric("no") == 0.0
    assert parse_bool_metric("") == 0.0


def test_parse_bool_rejects_nan_inf():
    assert parse_bool_metric(float("nan")) == 0.0
    assert parse_bool_metric(float("inf")) == 0.0


def test_parse_bool_unknown_type_is_zero():
    assert parse_bool_metric(None) == 0.0
    assert parse_bool_metric([1]) == 0.0


def test_try_load_json_valid_object():
    assert try_load_json('{"a": 1}') == {"a": 1}


def test_try_load_json_invalid_returns_none():
    assert try_load_json("not json") is None
    assert try_load_json("") is None
    assert try_load_json(None) is None


def test_try_load_json_non_object_returns_none():
    assert try_load_json("[1,2,3]") is None
    assert try_load_json("42") is None
    assert try_load_json('"hello"') is None


def test_extract_numeric_fields_basic():
    data = {"satellite_total": 13, "hdop": 1.5, "latitude": 30.0}
    out = extract_numeric_fields(
        data,
        numeric_keys=["satellite_total", "hdop"],
        bool_keys=[],
        prefix="gnss",
    )
    assert out == [("gnss/satellite_total", 13.0), ("gnss/hdop", 1.5)]


def test_extract_numeric_fields_skips_missing_and_bad():
    data = {"a": 1, "b": "not-a-number", "c": float("nan")}
    out = extract_numeric_fields(
        data,
        numeric_keys=["a", "b", "c", "missing"],
        bool_keys=[],
        prefix="p",
    )
    assert out == [("p/a", 1.0)]


def test_extract_numeric_fields_with_bool_keys():
    data = {"volume": 7, "obstaclesAvoidSwitch": True, "uwbSwitch": "false"}
    out = extract_numeric_fields(
        data,
        numeric_keys=["volume"],
        bool_keys=["obstaclesAvoidSwitch", "uwbSwitch"],
        prefix="go2_state",
    )
    assert out == [
        ("go2_state/volume", 7.0),
        ("go2_state/obstaclesAvoidSwitch", 1.0),
        ("go2_state/uwbSwitch", 0.0),
    ]


def test_extract_numeric_fields_empty_when_nothing_matches():
    out = extract_numeric_fields(
        {"unrelated": 1},
        numeric_keys=["a", "b"],
        bool_keys=["c"],
        prefix="x",
    )
    assert out == []
