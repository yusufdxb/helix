"""Offline replay harness for the HELIX anomaly detector.

Nothing here imports ROS. The shipped detector
(``src/helix_core/helix_core/anomaly_detector.py``) is treated as read-only
reference: ``load_shipped_process_sample()`` lifts its ``_process_sample``
method straight out of the source with ``ast`` and executes the SHIPPED
bytecode against a stub object, so equivalence with the mirror below can be
proven rather than asserted (see tests/test_detector_sweep.py).

Three detector variants share one control path (rolling window, evaluate
before append, consecutive_trigger, min_anomaly_duration_s, NaN-as-violation):

  zscore : mean / population std          <- exactly what ships today
  mad    : median / 1.4826 * MAD          <- robust to heavy tails
  ewma   : exponentially weighted mean/std

The R1 rule in helix_diagnosis turns an ANOMALY on any ``rate_hz/utlidar*``
metric into a STOP_AND_HOLD, and R2 resumes after 3.0 s with no anomaly. That
escalation is modelled here too, because "stops per hour" is the number that
actually matters to the robot, not "faults per hour".
"""
from __future__ import annotations

import ast
import math
import statistics
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Deque, Dict, Iterable, List, Optional

REPO = Path(__file__).resolve().parent.parent
SHIPPED_DETECTOR = REPO / "src" / "helix_core" / "helix_core" / "anomaly_detector.py"

FLAT_SIGNAL_EPSILON = 1e-6

# helix_diagnosis.rules constants, mirrored (rules.py is ROS-free but lives in a
# package we must not import from, so the two constants are restated).
ANOMALY_CLEAR_WINDOW_SECONDS = 3.0
STOP_METRIC_PREFIX = "rate_hz/utlidar"


# ── shipped-code loader (for the equivalence proof) ──────────────────────────


def load_shipped_process_sample() -> Callable:
    """Return the real ``AnomalyDetector._process_sample`` as a bare function.

    Parses the shipped source, extracts the module-level constants plus that
    single method, and compiles them in a namespace holding only stdlib names.
    No rclpy, no helix_msgs, no node instantiation.
    """
    tree = ast.parse(SHIPPED_DETECTOR.read_text())
    body: List[ast.stmt] = []
    for node in tree.body:
        if isinstance(node, ast.AnnAssign) or isinstance(node, ast.Assign):
            body.append(node)
        if isinstance(node, ast.ClassDef) and node.name == "AnomalyDetector":
            for item in node.body:
                if isinstance(item, ast.FunctionDef) and item.name == "_process_sample":
                    body.append(item)
    module = ast.Module(body=body, type_ignores=[])
    ast.fix_missing_locations(module)
    namespace: Dict[str, object] = {"math": math, "deque": deque}
    exec(compile(module, "<shipped_anomaly_detector>", "exec"), namespace)
    return namespace["_process_sample"]  # type: ignore[return-value]


class _NullLogger:
    def warn(self, *_a, **_k) -> None:  # noqa: D102
        pass

    def debug(self, *_a, **_k) -> None:  # noqa: D102
        pass

    def info(self, *_a, **_k) -> None:  # noqa: D102
        pass


class _NullLock:
    def __enter__(self):
        return self

    def __exit__(self, *exc) -> bool:
        return False


class ShippedShim:
    """Minimal stand-in for AnomalyDetector so the shipped method can run.

    ``clock`` supplies the value the shipped code reads from ``time.monotonic``;
    the harness drives it from the replay timeline.
    """

    def __init__(
        self,
        clock: Callable[[], float],
        zscore_threshold: float = 3.0,
        consecutive_trigger: int = 3,
        window_size: int = 60,
        min_anomaly_duration_s: float = 2.0,
    ) -> None:
        self._clock = clock
        self._zscore_threshold = zscore_threshold
        self._consecutive_trigger = consecutive_trigger
        self._window_size = window_size
        self._min_anomaly_duration_s = min_anomaly_duration_s
        self._windows: Dict[str, Deque[float]] = {}
        self._consecutive: Dict[str, int] = {}
        self._anomaly_start: Dict[str, float] = {}
        self._data_lock = _NullLock()
        self.emitted: List[dict] = []
        # The shipped source calls ``time.monotonic()``; give it a module-like
        # object whose monotonic() is our replay clock.
        self._time_stub = type("_T", (), {"monotonic": staticmethod(lambda: self._clock())})()

    def get_logger(self) -> _NullLogger:
        return _NullLogger()

    def _emit_anomaly_fault(self, metric_name, value, mean, std, zscore, consecutive):
        self.emitted.append(
            {
                "t": self._clock(),
                "metric": metric_name,
                "kind": "zscore",
                "value": value,
                "mean": mean,
                "std": std,
                "score": zscore,
                "consecutive": consecutive,
            }
        )

    def _emit_stale_fault(self, metric_name, consecutive):
        self.emitted.append(
            {
                "t": self._clock(),
                "metric": metric_name,
                "kind": "stale",
                "value": None,
                "mean": None,
                "std": None,
                "score": None,
                "consecutive": consecutive,
            }
        )


def run_shipped(samples: Iterable[tuple], **params) -> List[dict]:
    """Replay ``(t, metric, value)`` samples through the SHIPPED method body."""
    process = load_shipped_process_sample()
    clock_box = {"t": 0.0}
    shim = ShippedShim(clock=lambda: clock_box["t"], **params)
    # Rebind the global ``time`` the shipped code closes over.
    process.__globals__["time"] = shim._time_stub
    for t, metric, value in samples:
        clock_box["t"] = float(t)
        process(shim, metric, float("nan") if value is None else float(value))
    return shim.emitted


# ── mirrored detector with pluggable statistic ───────────────────────────────


def _stat_zscore(samples: List[float], value: float, _state: dict):
    """Population mean / std over the window. Identical to the shipped code."""
    mean = sum(samples) / len(samples)
    variance = sum((s - mean) ** 2 for s in samples) / len(samples)
    std = math.sqrt(variance)
    if std < FLAT_SIGNAL_EPSILON:
        return None
    return abs((value - mean) / std), mean, std


def _stat_mad(samples: List[float], value: float, _state: dict):
    """Median / scaled median-absolute-deviation. Robust to heavy tails."""
    med = statistics.median(samples)
    mad = statistics.median([abs(s - med) for s in samples])
    scale = 1.4826 * mad
    if scale < FLAT_SIGNAL_EPSILON:
        return None
    return abs((value - med) / scale), med, scale


def _stat_ewma(_samples: List[float], value: float, state: dict):
    """Exponentially weighted mean / std, alpha = 2/(window_size+1)."""
    mu = state.get("mu")
    var = state.get("var")
    if mu is None or var is None:
        return None
    std = math.sqrt(var)
    if std < FLAT_SIGNAL_EPSILON:
        return None
    return abs((value - mu) / std), mu, std


STATISTICS: Dict[str, Callable] = {
    "zscore": _stat_zscore,
    "mad": _stat_mad,
    "ewma": _stat_ewma,
}


@dataclass
class DetectorConfig:
    """One point in the parameter sweep."""

    statistic: str = "zscore"
    threshold: float = 3.0
    consecutive_trigger: int = 3
    window_size: int = 60
    min_anomaly_duration_s: float = 2.0

    def label(self) -> str:
        return (
            f"{self.statistic}/thr={self.threshold:g}/k={self.consecutive_trigger}"
            f"/W={self.window_size}/dur={self.min_anomaly_duration_s:g}"
        )


@dataclass
class ReplayDetector:
    """Mirror of the shipped detector with a swappable statistic.

    The control flow (evaluate before append, consecutive counting, streak-start
    timing, NaN-as-violation without polluting the window, flat-signal skip) is
    a line-for-line mirror of ``AnomalyDetector._process_sample``. With
    ``statistic='zscore'`` it is required to emit byte-identical faults; that
    requirement is enforced by tests/test_detector_sweep.py.
    """

    config: DetectorConfig = field(default_factory=DetectorConfig)
    _windows: Dict[str, Deque[float]] = field(default_factory=dict, init=False)
    _consecutive: Dict[str, int] = field(default_factory=dict, init=False)
    _anomaly_start: Dict[str, float] = field(default_factory=dict, init=False)
    _ewma: Dict[str, dict] = field(default_factory=dict, init=False)
    emitted: List[dict] = field(default_factory=list, init=False)

    def process(self, now: float, metric_name: str, value: float) -> None:
        cfg = self.config
        if metric_name not in self._windows:
            self._windows[metric_name] = deque(maxlen=cfg.window_size)
            self._consecutive[metric_name] = 0
            self._ewma[metric_name] = {}
        window = self._windows[metric_name]
        state = self._ewma[metric_name]

        if math.isnan(value):
            self._consecutive[metric_name] += 1
            consecutive = self._consecutive[metric_name]
            self._anomaly_start.setdefault(metric_name, now)
            if consecutive >= cfg.consecutive_trigger:
                elapsed = now - self._anomaly_start[metric_name]
                if cfg.min_anomaly_duration_s <= 0.0 or elapsed >= cfg.min_anomaly_duration_s:
                    self.emitted.append(
                        {
                            "t": now,
                            "metric": metric_name,
                            "kind": "stale",
                            "value": None,
                            "mean": None,
                            "std": None,
                            "score": None,
                            "consecutive": consecutive,
                        }
                    )
            return

        if len(window) >= 2:
            result = STATISTICS[cfg.statistic](list(window), value, state)
            if result is not None:
                score, centre, scale = result
                if score > cfg.threshold:
                    self._consecutive[metric_name] += 1
                    consecutive = self._consecutive[metric_name]
                    self._anomaly_start.setdefault(metric_name, now)
                    if consecutive >= cfg.consecutive_trigger:
                        elapsed = now - self._anomaly_start[metric_name]
                        if (
                            cfg.min_anomaly_duration_s <= 0.0
                            or elapsed >= cfg.min_anomaly_duration_s
                        ):
                            self.emitted.append(
                                {
                                    "t": now,
                                    "metric": metric_name,
                                    "kind": cfg.statistic,
                                    "value": value,
                                    "mean": centre,
                                    "std": scale,
                                    "score": score,
                                    "consecutive": consecutive,
                                }
                            )
                else:
                    self._consecutive[metric_name] = 0
                    self._anomaly_start.pop(metric_name, None)

        window.append(value)
        self._update_ewma(state, value)

    def _update_ewma(self, state: dict, value: float) -> None:
        alpha = 2.0 / (self.config.window_size + 1.0)
        if "mu" not in state:
            state["mu"] = value
            state["var"] = 0.0
            state["n"] = 1
            return
        delta = value - state["mu"]
        state["mu"] += alpha * delta
        state["var"] = (1.0 - alpha) * (state["var"] + alpha * delta * delta)
        state["n"] += 1
        # Withhold the statistic until the estimator has seen the same amount of
        # history the windowed variants require (>=2 prior samples).
        if state["n"] < 3:
            state["var"] = 0.0

    def run(self, samples: Iterable[tuple]) -> List[dict]:
        for t, metric, value in samples:
            self.process(float(t), metric, float("nan") if value is None else float(value))
        return self.emitted


def replay(samples: Iterable[tuple], config: DetectorConfig) -> List[dict]:
    """Convenience: run one config over one sample stream."""
    return ReplayDetector(config=config).run(samples)


# ── R1/R2 escalation model (faults -> robot stops) ───────────────────────────


def stop_episodes(
    faults: List[dict],
    clear_window_s: float = ANOMALY_CLEAR_WINDOW_SECONDS,
) -> List[dict]:
    """Collapse ANOMALY faults into STOP_AND_HOLD episodes per helix_diagnosis.

    R1 escalates only ``rate_hz/utlidar*`` ANOMALYs; R2 resumes once
    ``clear_window_s`` has passed with no further anomaly of any metric, which
    is how diagnosis_node tracks ``_last_anomaly_time``.
    """
    episodes: List[dict] = []
    state_stopped = False
    last_anomaly: Optional[float] = None
    start: Optional[float] = None
    for fault in sorted(faults, key=lambda f: f["t"]):
        t = fault["t"]
        if state_stopped and last_anomaly is not None and t - last_anomaly >= clear_window_s:
            episodes.append({"start": start, "end": last_anomaly + clear_window_s})
            state_stopped = False
            start = None
        last_anomaly = t
        if not state_stopped and str(fault["metric"]).startswith(STOP_METRIC_PREFIX):
            state_stopped = True
            start = t
    if state_stopped and start is not None and last_anomaly is not None:
        episodes.append({"start": start, "end": last_anomaly + clear_window_s})
    return episodes
