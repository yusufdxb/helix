"""Sliding-window callback-arrival rate estimator.

Pure-function core used by the topic_rate_monitor lifecycle node.
No ROS dependency — directly unit-testable.
"""
import math
import threading
import time
from collections import deque
from typing import Callable


class RateWindow:
    """Sliding-window Hz estimator over callback-arrival timestamps.

    Keeps the last ``window_sec`` seconds of timestamps and computes
    ``(n - 1) / span``. The timestamps are *receiver-side callback dispatch
    times*, not source header stamps — under executor queueing / scheduling
    jitter, measured rate can diverge from true publish rate.

    Startup behavior: returns 0.0 until 2+ samples are in the window.
    Stale (no samples in window): ``rate_or_nan()`` returns NaN.

    A monotonic clock source is injected so tests can drive synthetic time.
    """

    def __init__(self, window_sec: float = 5.0,
                 clock: Callable[[], float] = time.monotonic) -> None:
        if window_sec <= 0:
            raise ValueError("window_sec must be > 0")
        self._window_sec = window_sec
        self._clock = clock
        self._timestamps: deque = deque()
        self._lock = threading.Lock()

    def record(self) -> None:
        now = self._clock()
        with self._lock:
            self._timestamps.append(now)
            self._evict(now)

    def rate_or_nan(self) -> float:
        """Hz if >=2 samples in window; NaN if stale; 0.0 if exactly 1 sample."""
        now = self._clock()
        with self._lock:
            self._evict(now)
            n = len(self._timestamps)
            if n == 0:
                return math.nan
            if n == 1:
                return 0.0
            span = self._timestamps[-1] - self._timestamps[0]
            if span < 1e-6:
                return 0.0
            return (n - 1) / span

    def _evict(self, now: float) -> None:
        cutoff = now - self._window_sec
        while self._timestamps and self._timestamps[0] < cutoff:
            self._timestamps.popleft()
