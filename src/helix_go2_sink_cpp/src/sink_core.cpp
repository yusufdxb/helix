// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.

#include "helix_go2_sink_cpp/sink_core.hpp"

#include <charconv>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>

namespace helix_go2_sink_cpp
{

Mode parse_mode(const std::string & name)
{
  if (name == "dry_run") {
    return Mode::kDryRun;
  }
  if (name == "stop_only") {
    return Mode::kStopOnly;
  }
  if (name == "armed") {
    return Mode::kArmed;
  }
  throw std::invalid_argument(
          "mode must be one of ('dry_run', 'stop_only', 'armed'), got '" + name + "'");
}

const char * mode_name(Mode m) noexcept
{
  switch (m) {
    case Mode::kDryRun: return "dry_run";
    case Mode::kStopOnly: return "stop_only";
    case Mode::kArmed: return "armed";
  }
  return "?";
}

namespace
{

std::string json_number(double v)
{
  char buf[64];
  const auto res = std::to_chars(buf, buf + sizeof(buf), v);
  std::string out(buf, res.ptr);
  if (out.find_first_of(".en") == std::string::npos) {
    out += ".0";   // Python prints an integral float as 1.0, not 1
  }
  return out;
}

}  // namespace

std::string move_parameter(double x, double y, double z)
{
  return "{\"x\": " + json_number(x) + ", \"y\": " + json_number(y) + ", \"z\": " +
         json_number(z) + "}";
}

SinkLogic::SinkLogic(
  Mode mode, Limits limits, double input_timeout_sec, double stop_repeat_hz,
  double move_hz)
: mode_(mode), limits_(limits), input_timeout_sec_(input_timeout_sec),
  stop_period_(1.0 / stop_repeat_hz), move_period_(1.0 / move_hz)
{
}

std::optional<Request> SinkLogic::on_command(double vx, double vy, double wz, double now)
{
  last_input_at_ = now;
  deadman_tripped_ = false;
  if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) {
    return stop("REJECT_NON_FINITE", now, true);
  }
  if (vx == 0.0 && vy == 0.0 && wz == 0.0) {
    return stop("ZERO", now);
  }
  if (std::fabs(vx) > limits_.max_vx || std::fabs(vy) > limits_.max_vy ||
    std::fabs(wz) > limits_.max_wz)
  {
    return stop("REJECT_OVER_SINK_LIMIT", now, true);
  }
  if (mode_ == Mode::kStopOnly) {
    return stop("NOT_ARMED", now);
  }
  if (last_api_ == kApiMove && last_sent_at_ && now - *last_sent_at_ < move_period_) {
    return std::nullopt;
  }
  Request r;
  r.api_id = kApiMove;
  r.parameter = move_parameter(vx, vy, wz);
  r.reason = "MOVE";
  r.x = vx;
  r.y = vy;
  r.z = wz;
  return sent(std::move(r), now);
}

std::optional<Request> SinkLogic::on_tick(double now)
{
  if (!last_input_at_ || now - *last_input_at_ > input_timeout_sec_) {
    const bool first = !deadman_tripped_;
    deadman_tripped_ = true;
    return stop("DEADMAN", now, first);
  }
  return std::nullopt;
}

std::optional<Request> SinkLogic::stop(const char * reason, double now, bool force)
{
  const bool transition = last_api_ != kApiStopMove;
  const bool due = !last_sent_at_ || now - *last_sent_at_ >= stop_period_;
  if (force || transition || due) {
    Request r;
    r.api_id = kApiStopMove;
    r.reason = reason;
    return sent(std::move(r), now);
  }
  return std::nullopt;
}

Request SinkLogic::sent(Request r, double now)
{
  if (r.api_id != kApiStopMove && r.api_id != kApiMove) {
    throw std::logic_error("api_id refused");
  }
  last_api_ = r.api_id;
  last_sent_at_ = now;
  return r;
}

SinkFeedback::SinkFeedback(StillThresholds still, double feedback_timeout_s)
: speed_xy(std::numeric_limits<double>::quiet_NaN()),
  yaw_speed(std::numeric_limits<double>::quiet_NaN()),
  still_(still), feedback_timeout_s_(feedback_timeout_s)
{
}

void SinkFeedback::on_sent(
  std::int64_t request_id, std::int32_t api_id, const std::string & reason, double now)
{
  sent_[request_id] = Sent{api_id, now};
  if (sent_.size() > kMaxTracked) {
    sent_.erase(sent_.begin());
  }
  last_api_id = api_id;
  last_request_id = request_id;
  last_reason = reason;
  last_sent_at = now;
  if (api_id == kApiMove) {
    last_move_sent_at_ = now;
    stop_acked = false;
  }
}

bool SinkFeedback::on_response(
  std::int64_t request_id, std::int32_t api_id, std::int32_t code, double now)
{
  const auto it = sent_.find(request_id);
  if (it == sent_.end() || it->second.api_id != api_id) {
    return false;
  }
  const Sent s = it->second;
  last_ack_request_id = request_id;
  last_ack_code = code;
  last_ack_at = now;
  const bool newer_than_move = !last_move_sent_at_ || s.sent_at >= *last_move_sent_at_;
  if (s.api_id == kApiStopMove && code == 0 && newer_than_move &&
    last_api_id == kApiStopMove && !stop_acked)
  {
    stop_acked = true;
    stop_ack_sent_at = s.sent_at;
    stop_acked_at = now;
  }
  return true;
}

void SinkFeedback::on_state(
  double vx, double vy, double yaw, std::uint8_t mode, double now)
{
  speed_xy = std::hypot(vx, vy);
  yaw_speed = yaw;
  robot_mode = mode;
  state_at = now;
  const bool still = std::isfinite(speed_xy) && std::isfinite(yaw) &&
    speed_xy <= still_.max_speed_xy && std::fabs(yaw) <= still_.max_yaw_speed;
  if (!still) {
    still_since_.reset();
    stopped = false;
    return;
  }
  if (!still_since_) {
    still_since_ = now;
  }
  if (!stopped && now - *still_since_ >= still_.still_s) {
    stopped = true;
    stopped_since = now;
  }
}

bool SinkFeedback::feedback_available(double now) const noexcept
{
  return state_at > 0.0 && now - state_at <= feedback_timeout_s_;
}

}  // namespace helix_go2_sink_cpp
