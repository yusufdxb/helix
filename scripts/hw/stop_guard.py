"""Abort rule for native_stop_test.py's own motion source. No ROS imports.

The runner streams vx on the arbiter's nav source and must stop sourcing as
soon as the robot is not doing what the stage expects. On 2026-10-17 the
first version aborted on any single sample above --abort-speed from either
/sportmodestate or /utlidar/robot_odom. Two facts from those bags shaped
this rule:

* The two topics carry the same estimate. Their velocities differ by a
  median of exactly 0 and their positions match; odometry is every other
  SportModeState sample. A second topic is not a second sensor.
* Run 223245 aborted on a one-sample estimator transient at Move onset
  (0.431 m/s on one SportModeState sample and on the odometry sample
  0.2 ms later), before any fault was injected.

The guard therefore trips on any of:

* ``hard``: one sample above ``hard_limit``. The single-sample response is
  kept for speeds no low-speed stage can legitimately reach.
* ``overspeed``: speed above ``abort_speed`` continuously for ``persist_s``.
* ``travel``: displacement from the first position sample above
  ``max_travel_m``. Position integrates velocity, so a spike barely moves it.
* ``stale``: no state sample for ``stale_s`` while armed, so the robot is
  unmeasured.

Persistence delays an abort between abort_speed and hard_limit by at most
persist_s plus one sample period; at hard_limit that is under 3 cm.
"""
from __future__ import annotations

import math


class OverspeedGuard:

    def __init__(self, abort_speed: float, hard_limit: float = 0.60, persist_s: float = 0.03,
                 max_travel_m: float = 0.60, stale_s: float = 0.20) -> None:
        if not 0.0 < abort_speed < hard_limit:
            raise ValueError('need 0 < abort_speed < hard_limit')
        self.abort_speed = abort_speed
        self.hard_limit = hard_limit
        self.persist_s = persist_s
        self.max_travel_m = max_travel_m
        self.stale_s = stale_s
        self.reason: str | None = None
        self.detail: dict = {}
        self._above_since: dict[str, float] = {}
        self._origin: tuple[float, float] | None = None
        self._last_state_t: float | None = None

    def arm(self, now: float) -> None:
        """Start of streaming: the stale clock starts now if no sample came yet."""
        if self._last_state_t is None:
            self._last_state_t = now

    @property
    def tripped(self) -> bool:
        return self.reason is not None

    def _trip(self, reason: str, **detail) -> str:
        if self.reason is None:
            self.reason = reason
            self.detail = detail
        return self.reason

    def speed(self, t: float, v: float, source: str) -> str | None:
        """Feed one speed sample (m/s) from ``source``; return the trip reason, if any."""
        if source == 'state':
            self._last_state_t = t
        if not math.isfinite(v):
            return self._trip('nonfinite', t=t, source=source)
        if v > self.hard_limit:
            return self._trip('hard', t=t, source=source, v=v)
        if v <= self.abort_speed:
            self._above_since.pop(source, None)
            return self.reason
        since = self._above_since.setdefault(source, t)
        if t - since >= self.persist_s:
            return self._trip('overspeed', t=t, source=source, v=v, above_s=t - since)
        return self.reason

    def position(self, t: float, x: float, y: float) -> str | None:
        """Feed one estimated position (m)."""
        if self._origin is None:
            self._origin = (x, y)
            return self.reason
        d = math.hypot(x - self._origin[0], y - self._origin[1])
        if d > self.max_travel_m:
            return self._trip('travel', t=t, travel_m=d)
        return self.reason

    def check_stale(self, now: float) -> str | None:
        """Call while streaming: trips when the state topic has gone quiet."""
        if self._last_state_t is not None and now - self._last_state_t > self.stale_s:
            return self._trip('stale', t=now, age_s=now - self._last_state_t)
        return self.reason
