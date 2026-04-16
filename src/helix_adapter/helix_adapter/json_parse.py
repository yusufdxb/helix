"""Pure-function helpers for parsing JSON string fields into numeric metrics.

No ROS dependency — directly unit-testable.
"""
import json
import math
from typing import Any, Dict, Iterable, List, Optional, Tuple

_FALSY_STRINGS = frozenset({"false", "0", "no", "off", "none", ""})


def safe_float(value: Any) -> Optional[float]:
    """Convert value to float; return None for NaN/inf/unparseable.

    Prevents NaN/inf from reaching downstream Z-score windows where they
    would corrupt rolling mean/std.
    """
    try:
        f = float(value)
        return f if math.isfinite(f) else None
    except (ValueError, TypeError):
        return None


def parse_bool_metric(value: Any) -> float:
    """Convert a JSON bool / int / string to 0.0 or 1.0.

    Plain Python truthiness is wrong here because ``bool("false") is True``.
    """
    if isinstance(value, bool):
        return 1.0 if value else 0.0
    if isinstance(value, (int, float)):
        if isinstance(value, float) and not math.isfinite(value):
            return 0.0
        return 0.0 if value == 0 else 1.0
    if isinstance(value, str):
        return 0.0 if value.strip().lower() in _FALSY_STRINGS else 1.0
    return 0.0


def try_load_json(raw: str) -> Optional[Dict[str, Any]]:
    """Parse a JSON object; return None on any failure or non-object payload."""
    try:
        data = json.loads(raw)
    except (json.JSONDecodeError, TypeError):
        return None
    return data if isinstance(data, dict) else None


def extract_numeric_fields(
    data: Dict[str, Any],
    numeric_keys: Iterable[str],
    bool_keys: Iterable[str],
    prefix: str,
) -> List[Tuple[str, float]]:
    """Extract (metric_name, value) pairs from a parsed JSON dict.

    Numeric keys are passed through ``safe_float`` (dropped if NaN/inf/unparseable).
    Boolean keys are passed through ``parse_bool_metric`` (always emit 0.0/1.0
    when the key is present).
    Missing keys are skipped silently. Metric names are ``{prefix}/{key}``.
    """
    out: List[Tuple[str, float]] = []
    for key in numeric_keys:
        if key in data:
            v = safe_float(data[key])
            if v is not None:
                out.append((f"{prefix}/{key}", v))
    for key in bool_keys:
        if key in data:
            out.append((f"{prefix}/{key}", parse_bool_metric(data[key])))
    return out
