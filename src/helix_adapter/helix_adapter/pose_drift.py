"""Pure-function displacement-rate tracker for a 3D position stream.

No ROS dependency — directly unit-testable.
"""
import math
from typing import Optional, Tuple


class DisplacementTracker:
    """Tracks 3D position updates and reports rolling displacement rate (m/s).

    ``update(x, y, z, t)`` is called at arrival time; ``rate_or_nan(now)``
    returns the last computed rate, or NaN if no update has arrived within
    ``stale_sec``. ``dt < min_dt`` updates are ignored (prevents division
    blow-up on tight bursts) but still refresh ``last_time`` so staleness
    is computed against the most recent arrival.
    """

    def __init__(self, stale_sec: float = 5.0, min_dt: float = 1e-3) -> None:
        if stale_sec <= 0:
            raise ValueError("stale_sec must be > 0")
        if min_dt <= 0:
            raise ValueError("min_dt must be > 0")
        self._stale_sec = stale_sec
        self._min_dt = min_dt
        self._last_pos: Optional[Tuple[float, float, float]] = None
        self._last_time: Optional[float] = None
        self._rate: float = 0.0

    def update(self, x: float, y: float, z: float, t: float) -> None:
        if self._last_pos is not None and self._last_time is not None:
            dt = t - self._last_time
            if dt >= self._min_dt:
                dx = x - self._last_pos[0]
                dy = y - self._last_pos[1]
                dz = z - self._last_pos[2]
                self._rate = math.sqrt(dx * dx + dy * dy + dz * dz) / dt
        self._last_pos = (x, y, z)
        self._last_time = t

    def rate_or_nan(self, now: float) -> float:
        if self._last_time is None:
            return math.nan
        if (now - self._last_time) >= self._stale_sec:
            return math.nan
        return self._rate
