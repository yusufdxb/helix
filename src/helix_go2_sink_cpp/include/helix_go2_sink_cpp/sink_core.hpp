// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// ROS-free core of the GO2 sport sink, ported from
// helix_arbiter/sport_sink_core.py (SinkLogic, SinkFeedback) with the same
// decisions. test_python_parity.py replays one event stream through both.
//
// The sink is the only HELIX-side publisher on /api/sport/request, and its
// only input is the arbiter output:
//
//   zero command           -> StopMove (1003) on the transition, then at
//                             stop_repeat_hz while zero
//   nonzero, armed/dry_run -> Move (1008) {"x","y","z"} at most move_hz
//   nonzero, stop_only     -> StopMove (never Move)
//   over sink limits       -> StopMove (rejected, never clamped)
//   non-finite             -> StopMove
//   input older than input_timeout_sec -> StopMove (DEADMAN)
//
// 1001 on /api/sport/request is Damp and drops the robot. Only 1003 and
// 1008 can ever be produced, and make_request refuses anything else.
#ifndef HELIX_GO2_SINK_CPP__SINK_CORE_HPP_
#define HELIX_GO2_SINK_CPP__SINK_CORE_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <string>

namespace helix_go2_sink_cpp
{

constexpr std::int32_t kApiStopMove = 1003;
constexpr std::int32_t kApiMove = 1008;

enum class Mode : std::uint8_t {kDryRun, kStopOnly, kArmed};

/// "dry_run" | "stop_only" | "armed"; throws std::invalid_argument otherwise.
Mode parse_mode(const std::string & name);
const char * mode_name(Mode m) noexcept;

struct Limits
{
  double max_vx{0.25};
  double max_vy{0.20};
  double max_wz{0.50};
};

struct Request
{
  std::int32_t api_id{kApiStopMove};
  std::string parameter;   // JSON, empty for StopMove
  std::string reason;
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/// Same JSON text as json.dumps({'x': x, 'y': y, 'z': z}) for ordinary
/// values (shortest round-trip digits, ".0" on integral values).
std::string move_parameter(double x, double y, double z);

class SinkLogic
{
public:
  SinkLogic(
    Mode mode, Limits limits = {}, double input_timeout_sec = 0.25,
    double stop_repeat_hz = 2.0, double move_hz = 20.0);

  std::optional<Request> on_command(double vx, double vy, double wz, double now);
  /// Deadman check; call periodically.
  std::optional<Request> on_tick(double now);

  Mode mode() const noexcept {return mode_;}
  double input_timeout_sec() const noexcept {return input_timeout_sec_;}
  bool deadman_tripped() const noexcept {return deadman_tripped_;}

private:
  std::optional<Request> stop(const char * reason, double now, bool force = false);
  Request sent(Request r, double now);

  Mode mode_;
  Limits limits_;
  double input_timeout_sec_;
  double stop_period_;
  double move_period_;
  std::optional<double> last_input_at_;
  std::optional<double> last_sent_at_;
  std::optional<std::int32_t> last_api_;
  bool deadman_tripped_{false};
};

struct StillThresholds
{
  double max_speed_xy{0.05};   // m/s
  double max_yaw_speed{0.10};  // rad/s
  double still_s{0.3};
};

/// Controller ack and measured motion for the requests this sink sent.
///
/// The robot answers every request on /api/sport/response, including those
/// of the other publishers on /api/sport/request, so only this sink's own
/// request ids are matched. stop_acked is true only while the newest sent
/// request is a StopMove and the robot answered that StopMove, or one sent
/// after the newest Move, with status code 0. A Move sent afterwards clears
/// it. Time is the caller's monotonic clock.
class SinkFeedback
{
public:
  static constexpr std::size_t kMaxTracked = 256;

  explicit SinkFeedback(StillThresholds still = {}, double feedback_timeout_s = 1.0);

  void on_sent(
    std::int64_t request_id, std::int32_t api_id, const std::string & reason,
    double now);
  /// True if the response answered one of this sink's requests.
  bool on_response(std::int64_t request_id, std::int32_t api_id, std::int32_t code, double now);
  void on_state(double vx, double vy, double yaw_speed, std::uint8_t mode, double now);
  /// Robot state is fresh.
  bool feedback_available(double now) const noexcept;
  bool stopped_now(double now) const noexcept {return stopped && feedback_available(now);}

  // Public state, mirrored into helix_msgs/SinkStatus.
  std::int32_t last_api_id{0};
  std::int64_t last_request_id{0};
  std::string last_reason;
  double last_sent_at{0.0};
  std::int64_t last_ack_request_id{0};
  std::int32_t last_ack_code{0};
  double last_ack_at{0.0};
  bool stop_acked{false};
  double stop_ack_sent_at{0.0};
  double stop_acked_at{0.0};
  double speed_xy;    // NaN until the first state
  double yaw_speed;
  double state_at{0.0};
  std::uint8_t robot_mode{0};
  bool stopped{false};
  double stopped_since{0.0};

private:
  struct Sent
  {
    std::int32_t api_id;
    double sent_at;
  };

  StillThresholds still_;
  double feedback_timeout_s_;
  // Request ids only increase, so the smallest key is the oldest entry.
  std::map<std::int64_t, Sent> sent_;
  std::optional<double> last_move_sent_at_;
  std::optional<double> still_since_;
};

}  // namespace helix_go2_sink_cpp

#endif  // HELIX_GO2_SINK_CPP__SINK_CORE_HPP_
