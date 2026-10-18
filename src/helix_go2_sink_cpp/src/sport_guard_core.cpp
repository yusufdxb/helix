// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
#include "helix_go2_sink_cpp/sport_guard_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace helix_go2_sink_cpp
{

SportGuard::SportGuard(
  const std::string & guard_session, double move_timeout, double idle_timeout,
  double status_timeout, double stop_period)
: move_timeout_(move_timeout), idle_timeout_(idle_timeout), status_timeout_(status_timeout),
  stop_period_(stop_period), guard_session_(guard_session),
  release_token_(guard_session + ":1:0")
{
  if (guard_session.empty() ||
    guard_session.find_first_not_of(
      "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789:_-") !=
    std::string::npos)
  {
    throw std::invalid_argument("guard session must be nonempty and use token-safe characters");
  }
  for (double t : {move_timeout_, idle_timeout_, status_timeout_, stop_period_}) {
    if (!std::isfinite(t) || t <= 0.0) {
      throw std::invalid_argument("guard timeouts must be finite and positive");
    }
  }
}

void SportGuard::trip(const std::string & reason, double now)
{
  detected_at_ = now;
  latched_ = true;
  moving_ = false;
  fault_required_ = true;
  protocol_released_ = false;
  release_ready_ = false;
  readiness_generation_ = 0;
  release_token_ = guard_session_ + ":" + std::to_string(++protection_generation_) + ":0";
  reason_ = reason;
  blocked_challenge_ = current_challenge_;
  held_challenge_.clear();
  request_at_.reset();  // Readiness must be observed after this protection event.
}

bool SportGuard::request_fresh(double now) const
{
  return request_at_ && now - *request_at_ <= (moving_ ? move_timeout_ : idle_timeout_);
}

bool SportGuard::ready_for_release(double now) const
{
  return output_healthy_ && release_ready_ && latched_ && protocol_released_ &&
         !fault_required_ && status_at_ &&
         now - *status_at_ <= status_timeout_ && request_fresh(now);
}

void SportGuard::update_readiness(double now)
{
  const bool ready = output_healthy_ && latched_ && protocol_released_ &&
    !fault_required_ && status_at_ &&
    now - *status_at_ <= status_timeout_ && request_fresh(now);
  if (ready != release_ready_) {
    release_ready_ = ready;
    release_token_ = guard_session_ + ":" + std::to_string(protection_generation_) + ":" +
      std::to_string(++readiness_generation_);
  }
}

void SportGuard::on_output_health(bool healthy, const std::string & reason, double now)
{
  check_timeouts(now);
  if (healthy == output_healthy_) {
    return;
  }
  output_healthy_ = healthy;
  if (!healthy) {
    ++output_health_detections_;
    output_health_detected_at_ = now;
    output_health_reason_ = reason;
    trip(reason, now);
  }
  update_readiness(now);
}

bool SportGuard::on_operator_release(std::string token, double now)
{
  check_timeouts(now);
  if (!latched_ || token != release_token_) {
    return false;
  }
  if (!ready_for_release(now)) {
    // Consume premature current authorization by rotating it. Never bank it.
    trip("OPERATOR_RELEASE_NOT_READY", now);
    return false;
  }
  latched_ = false;
  moving_ = false;
  released_challenge_ = current_challenge_;
  held_challenge_.clear();
  reason_ = "CURRENT_GUARD_OPERATOR_RELEASE";
  update_readiness(now);  // Consume this ready authorization, including queued duplicates.
  return true;
}

void SportGuard::check_timeouts(double now)
{
  if (status_at_ && now - *status_at_ > status_timeout_) {
    if (!latched_ || !held_challenge_.empty()) {
      trip("ARBITER_STATUS_TIMEOUT", now);
    }
  }
  if ((!latched_ || release_ready_) && !request_fresh(now)) {
    trip("SINK_PROGRESS_TIMEOUT", now);
  }
  update_readiness(now);
}

bool SportGuard::on_status(
  std::uint64_t seq, const std::string & challenge, bool protective_latched,
  bool hold_active, double now)
{
  check_timeouts(now);  // Detect a gap before replacing its receipt time.
  const auto colon = challenge.rfind(':');
  if (colon == std::string::npos || colon == 0 || colon + 1 == challenge.size()) {
    return false;
  }
  const auto session = challenge.substr(0, colon);
  if (session != session_) {
    if (retired_sessions_.count(session) || !protective_latched) {
      return false;
    }
    if (!session_.empty()) {
      retired_sessions_.insert(session_);
      trip("ARBITER_SESSION_CHANGED", now);
    }
    session_ = session;
    status_seq_ = 0;
  }
  if (seq <= status_seq_) {
    return false;
  }
  if (protective_latched && !current_challenge_.empty() && challenge != current_challenge_) {
    trip("ARBITER_PROTECTION_GENERATION_CHANGED", now);
  }
  status_seq_ = seq;
  status_at_ = now;
  current_challenge_ = challenge;
  if (protective_latched) {
    protocol_released_ = false;
    if (!latched_) {
      trip("ARBITER_PROTECTIVE_HOLD", now);
      // This newly observed hold itself satisfies the protective handshake.
      if (challenge != released_challenge_) {
        blocked_challenge_.clear();
      }
    }
    latched_ = true;
    moving_ = false;
    if (challenge != blocked_challenge_) {
      held_challenge_ = challenge;
      fault_required_ = false;
    }
    update_readiness(now);
    return true;
  }
  if (hold_active) {
    protocol_released_ = false;
    if (!latched_ || !held_challenge_.empty()) {
      trip("ARBITER_HOLD", now);
    }
    update_readiness(now);
    return true;
  }
  if (latched_) {
    if (!held_challenge_.empty() && challenge == held_challenge_) {
      protocol_released_ = true;
      fault_required_ = false;
      reason_ = "AWAIT_GUARD_OPERATOR_RELEASE";
    } else if (!fault_required_) {
      trip("UNOBSERVED_PROTECTIVE_GENERATION", now);
    }
  } else if (challenge != released_challenge_) {
    // A new generation without an observed held state is not authorization.
    trip("UNOBSERVED_PROTECTIVE_GENERATION", now);
  }
  update_readiness(now);
  return true;
}

std::optional<Request> SportGuard::on_request(
  std::int64_t id, const Request & request, double now)
{
  check_timeouts(now);  // A delayed request cannot erase an already expired lease.
  if (id <= highest_request_id_) {
    return std::nullopt;
  }
  if (request.api_id != kApiMove && request.api_id != kApiStopMove) {
    trip("INVALID_SPORT_API", now);
    return std::nullopt;
  }
  highest_request_id_ = id;
  request_at_ = now;
  if (request.api_id == kApiStopMove) {
    moving_ = false;
    update_readiness(now);
    return request;
  }
  if (latched_) {
    update_readiness(now);
    return std::nullopt;
  }
  moving_ = true;
  update_readiness(now);
  return request;
}

std::optional<Request> SportGuard::on_tick(double now)
{
  check_timeouts(now);
  if (!latched_ || (last_stop_at_ && now - *last_stop_at_ < stop_period_)) {
    return std::nullopt;
  }
  last_stop_at_ = now;
  Request request;
  request.api_id = kApiStopMove;
  request.reason = reason_;
  return request;
}

ControllerWatchdog::ControllerWatchdog(
  double ack_timeout, double state_timeout, bool require_ack, bool require_state)
: ack_timeout_(ack_timeout), state_timeout_(state_timeout), require_ack_(require_ack),
  require_state_(require_state)
{
  for (double timeout : {ack_timeout_, state_timeout_}) {
    if (!std::isfinite(timeout) || timeout <= 0.0) {
      throw std::invalid_argument("controller timeouts must be finite and positive");
    }
  }
}

void ControllerWatchdog::fail(const std::string & reason, double now)
{
  failure_at_ = now;
  failure_reason_ = reason;
}

void ControllerWatchdog::expire(double now)
{
  for (auto it = pending_.begin(); it != pending_.end(); ) {
    if (now - it->second.sent_at > ack_timeout_) {
      fail("CONTROLLER_ACK_TIMEOUT", now);
      it = pending_.erase(it);
    } else {
      ++it;
    }
  }
}

void ControllerWatchdog::on_sent(std::int64_t id, std::int32_t api, double now)
{
  if (!require_ack_) {
    return;
  }
  expire(now);
  if (id == 0 || (api != kApiStopMove && api != kApiMove) || pending_.count(id)) {
    fail("CONTROLLER_REQUEST_ID_INVALID", now);
    return;
  }
  if (pending_.size() >= kMaxPending) {
    const auto oldest = std::min_element(
      pending_.begin(), pending_.end(),
      [](const auto & a, const auto & b) {return a.second.sent_at < b.second.sent_at;});
    pending_.erase(oldest);
    fail("CONTROLLER_PENDING_OVERFLOW", now);
  }
  pending_.emplace(id, Pending{api, now});
}

bool ControllerWatchdog::on_response(
  std::int64_t id, std::int32_t api, std::int32_t code, double now)
{
  expire(now);
  const auto it = pending_.find(id);
  if (it == pending_.end() || it->second.api != api) {
    return false;
  }
  pending_.erase(it);
  if (code != 0) {
    fail("CONTROLLER_REQUEST_REJECTED", now);
  } else {
    ack_at_ = now;
  }
  return true;
}

void ControllerWatchdog::on_state(bool valid, double now)
{
  if (state_at_ && now - *state_at_ > state_timeout_) {
    fail("CONTROLLER_STATE_TIMEOUT", now);
  }
  state_at_ = now;
  state_valid_ = valid;
  if (!valid) {
    fail("CONTROLLER_STATE_INVALID", now);
  }
}

ControllerWatchdog::Health ControllerWatchdog::health(double now)
{
  expire(now);
  if (require_ack_) {
    if (!ack_at_) {
      return {false, failure_at_ ? failure_reason_ : "CONTROLLER_ACK_MISSING"};
    }
    if (now - *ack_at_ > ack_timeout_) {
      fail("CONTROLLER_ACK_TIMEOUT", now);
      return {false, failure_reason_};
    }
  }
  if (require_state_) {
    if (!state_at_) {
      return {false, "CONTROLLER_STATE_MISSING"};
    }
    if (!state_valid_) {
      return {false, "CONTROLLER_STATE_INVALID"};
    }
    if (now - *state_at_ > state_timeout_) {
      fail("CONTROLLER_STATE_TIMEOUT", now);
      return {false, failure_reason_};
    }
  }
  // Require one complete ack window without a detected downstream failure.
  // Intermittently fresh replies cannot hide an ongoing stream of expiries.
  if (failure_at_ && now - *failure_at_ < ack_timeout_) {
    return {false, failure_reason_};
  }
  return {true, "CONTROLLER_PROOF_FRESH"};
}

}  // namespace helix_go2_sink_cpp
