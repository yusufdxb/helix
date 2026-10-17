"""Pure logic for the GO2 sport sink. No ROS imports.

The sink is the only HELIX-side publisher on /api/sport/request, and its only
input is the arbiter output. It is a stateless translator plus a deadman:

  zero command          -> StopMove (1003), on the transition and then at
                           stop_repeat_hz while zero
  nonzero, armed/dry_run -> Move (1008) {"x","y","z"} at most move_hz
  nonzero, stop_only    -> StopMove (never Move)

dry_run makes exactly the armed decisions so its trace shows what would
have been sent; the node never publishes to the robot in dry_run.
  over sink limits      -> StopMove (rejected, never clamped)
  input older than input_timeout_sec -> StopMove (DEADMAN)

API ids are the ones verified on this robot's /api/sport/request
(docs/GO2_FIELD_NOTES.md section 4). 1001 on that topic is Damp and drops the
robot; it is refused by construction.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Optional

API_STOP_MOVE = 1003
API_MOVE = 1008
ALLOWED_API_IDS = frozenset({API_STOP_MOVE, API_MOVE})

MODE_DRY_RUN = 'dry_run'      # armed decisions, nothing reaches the robot
MODE_STOP_ONLY = 'stop_only'  # only StopMove reaches the robot
MODE_ARMED = 'armed'          # Move allowed within sink limits
MODES = (MODE_DRY_RUN, MODE_STOP_ONLY, MODE_ARMED)


@dataclass(frozen=True)
class SinkLimits:
    max_vx: float = 0.25
    max_vy: float = 0.20
    max_wz: float = 0.50


@dataclass(frozen=True)
class SportRequest:
    api_id: int
    parameter: str       # JSON string, '' for StopMove
    reason: str
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


def stop_move(reason: str) -> SportRequest:
    return SportRequest(API_STOP_MOVE, '', reason)


def move(x: float, y: float, z: float) -> SportRequest:
    return SportRequest(API_MOVE, json.dumps({'x': x, 'y': y, 'z': z}), 'MOVE', x, y, z)


class SinkLogic:

    def __init__(self, mode: str, limits: SinkLimits = SinkLimits(),
                 input_timeout_sec: float = 0.25, stop_repeat_hz: float = 2.0,
                 move_hz: float = 20.0) -> None:
        if mode not in MODES:
            raise ValueError(f'mode must be one of {MODES}, got {mode!r}')
        self.mode = mode
        self.limits = limits
        self.input_timeout_sec = input_timeout_sec
        self._stop_period = 1.0 / stop_repeat_hz
        self._move_period = 1.0 / move_hz
        self._last_input_at: Optional[float] = None
        self._last_sent_at: Optional[float] = None
        self._last_api: Optional[int] = None
        self.deadman_tripped = False

    def on_command(self, vx: float, vy: float, wz: float, now: float) -> Optional[SportRequest]:
        self._last_input_at = now
        self.deadman_tripped = False
        vals = (vx, vy, wz)
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals):
            return self._stop('REJECT_NON_FINITE', now, force=True)
        if vx == 0.0 and vy == 0.0 and wz == 0.0:
            return self._stop('ZERO', now)
        if (abs(vx) > self.limits.max_vx or abs(vy) > self.limits.max_vy
                or abs(wz) > self.limits.max_wz):
            return self._stop('REJECT_OVER_SINK_LIMIT', now, force=True)
        if self.mode == MODE_STOP_ONLY:
            return self._stop('NOT_ARMED', now)
        if (self._last_api == API_MOVE and self._last_sent_at is not None
                and now - self._last_sent_at < self._move_period):
            return None
        return self._sent(move(float(vx), float(vy), float(wz)), now)

    def on_tick(self, now: float) -> Optional[SportRequest]:
        """Deadman check; call periodically."""
        if self._last_input_at is None or now - self._last_input_at > self.input_timeout_sec:
            first = not self.deadman_tripped
            self.deadman_tripped = True
            return self._stop('DEADMAN', now, force=first)
        return None

    def _stop(self, reason: str, now: float, force: bool = False) -> Optional[SportRequest]:
        transition = self._last_api != API_STOP_MOVE
        due = self._last_sent_at is None or now - self._last_sent_at >= self._stop_period
        if force or transition or due:
            return self._sent(stop_move(reason), now)
        return None

    def _sent(self, req: SportRequest, now: float) -> SportRequest:
        assert req.api_id in ALLOWED_API_IDS
        self._last_api = req.api_id
        self._last_sent_at = now
        return req


def fill_unitree_request(msg, req: SportRequest, request_id: int):
    """Populate a unitree_api/msg/Request exactly as the field notes specify."""
    if req.api_id not in ALLOWED_API_IDS:
        raise ValueError(f'api_id {req.api_id} refused')
    msg.header.identity.id = int(request_id)
    msg.header.identity.api_id = int(req.api_id)
    msg.header.lease.id = 0
    msg.header.policy.priority = 0
    msg.header.policy.noreply = False
    msg.parameter = req.parameter
    return msg


@dataclass(frozen=True)
class StillThresholds:
    """Measured-stop thresholds on SportModeState (velocity is body frame)."""

    max_speed_xy: float = 0.05   # m/s
    max_yaw_speed: float = 0.10  # rad/s
    still_s: float = 0.3         # must hold this long to count as stopped


class SinkFeedback:
    """Controller ack and measured motion for the requests this sink sent.

    The robot answers every request on /api/sport/response, including the
    ones from the other publishers on /api/sport/request, so only request ids
    this sink sent are matched. stop_acked is true only while the newest sent
    request is a StopMove and the robot answered that StopMove, or one sent
    after the newest Move, with status code 0. A Move sent afterwards clears
    it. Time is the caller's monotonic clock.
    """

    MAX_TRACKED = 256

    def __init__(self, still: StillThresholds = StillThresholds(),
                 feedback_timeout_s: float = 1.0) -> None:
        self.still = still
        self.feedback_timeout_s = feedback_timeout_s
        self._sent: dict = {}          # request id -> (api_id, sent_at)
        self.last_api_id = 0
        self.last_request_id = 0
        self.last_reason = ''
        self.last_sent_at = 0.0
        self._last_move_sent_at: Optional[float] = None
        self.last_ack_request_id = 0
        self.last_ack_code = 0
        self.last_ack_at = 0.0
        self.last_response_at: Optional[float] = None
        self.stop_acked = False
        self.stop_ack_sent_at = 0.0
        self.stop_acked_at = 0.0
        self.speed_xy = math.nan
        self.yaw_speed = math.nan
        self.state_at = 0.0
        self.robot_mode = 0
        self._still_since: Optional[float] = None
        self.stopped = False
        self.stopped_since = 0.0

    def on_sent(self, request_id: int, api_id: int, reason: str, now: float) -> None:
        self._sent[int(request_id)] = (int(api_id), now)
        if len(self._sent) > self.MAX_TRACKED:
            self._sent.pop(next(iter(self._sent)))
        self.last_api_id = int(api_id)
        self.last_request_id = int(request_id)
        self.last_reason = reason
        self.last_sent_at = now
        if api_id == API_MOVE:
            self._last_move_sent_at = now
            self.stop_acked = False

    def on_response(self, request_id: int, api_id: int, code: int, now: float) -> bool:
        """Record a response; True if it answered one of this sink's requests."""
        self.last_response_at = now
        sent = self._sent.get(int(request_id))
        if sent is None or sent[0] != int(api_id):
            return False
        sent_api, sent_at = sent
        self.last_ack_request_id = int(request_id)
        self.last_ack_code = int(code)
        self.last_ack_at = now
        newer_than_move = (self._last_move_sent_at is None
                           or sent_at >= self._last_move_sent_at)
        if (sent_api == API_STOP_MOVE and code == 0 and newer_than_move
                and self.last_api_id == API_STOP_MOVE and not self.stop_acked):
            self.stop_acked = True
            self.stop_ack_sent_at = sent_at
            self.stop_acked_at = now
        return True

    def on_state(self, vx: float, vy: float, yaw_speed: float, mode: int, now: float) -> None:
        self.speed_xy = math.hypot(vx, vy)
        self.yaw_speed = yaw_speed
        self.robot_mode = int(mode)
        self.state_at = now
        still = (math.isfinite(self.speed_xy) and math.isfinite(yaw_speed)
                 and self.speed_xy <= self.still.max_speed_xy
                 and abs(yaw_speed) <= self.still.max_yaw_speed)
        if not still:
            self._still_since = None
            self.stopped = False
            return
        if self._still_since is None:
            self._still_since = now
        if not self.stopped and now - self._still_since >= self.still.still_s:
            self.stopped = True
            self.stopped_since = now

    def feedback_available(self, now: float) -> bool:
        """Robot state is fresh. Responses only arrive after requests, so
        their absence alone does not mean the feedback path is down."""
        return self.state_at > 0.0 and now - self.state_at <= self.feedback_timeout_s

    def stopped_now(self, now: float) -> bool:
        return self.stopped and self.feedback_available(now)
