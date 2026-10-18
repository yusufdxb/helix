// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
#ifndef HELIX_GO2_SINK_CPP__SPORT_GUARD_CORE_HPP_
#define HELIX_GO2_SINK_CPP__SPORT_GUARD_CORE_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_set>

#include "helix_go2_sink_cpp/sink_core.hpp"

namespace helix_go2_sink_cpp
{

/// Experimental last-hop gate, tested only with a localhost controller.
/// All times are monotonic receipt times. No controller command expiry is assumed.
/// Startup and every trip require a newly observed protective challenge followed
/// by its current released status AND an explicit current guard-instance token.
/// Arbiter status alone never opens this gate. Earlier requests are never queued.
class SportGuard
{
public:
  SportGuard(
    const std::string & guard_session, double move_timeout = 0.25, double idle_timeout = 0.75,
    double status_timeout = 0.30, double stop_period = 0.05);

  bool on_status(
    std::uint64_t seq, const std::string & challenge, bool protective_latched,
    bool hold_active, double now);
  /// Tokens mint on entering readiness, not before health. Every trip, readiness
  /// loss and successful release invalidates older authorization without banking it.
  bool on_operator_release(std::string token, double now);
  bool ready_for_release(double now) const;
  /// Positive, strictly increasing sink request ids are required. Stop intents
  /// retain their id and may pass even while latched; Move intents may not.
  std::optional<Request> on_request(std::int64_t id, const Request & request, double now);
  /// Independently repeats StopMove while protected, including before any input.
  std::optional<Request> on_tick(double now);

  bool latched() const noexcept {return latched_;}
  bool fault_required() const noexcept {return fault_required_;}
  const std::string & reason() const noexcept {return reason_;}
  const std::string & release_token() const noexcept {return release_token_;}

private:
  void trip(const std::string & reason);
  void check_timeouts(double now);
  void update_readiness(double now);
  bool request_fresh(double now) const;

  double move_timeout_;
  double idle_timeout_;
  double status_timeout_;
  double stop_period_;
  bool latched_{true};
  bool fault_required_{true};
  bool moving_{false};
  bool protocol_released_{false};
  std::string guard_session_;
  std::uint64_t protection_generation_{1};
  std::uint64_t readiness_generation_{0};
  bool release_ready_{false};
  std::string release_token_;
  std::string reason_{"STARTUP"};
  std::string session_;
  std::unordered_set<std::string> retired_sessions_;
  std::string current_challenge_;
  std::string blocked_challenge_;
  std::string held_challenge_;
  std::string released_challenge_;
  std::uint64_t status_seq_{0};
  std::optional<double> status_at_;
  std::optional<double> request_at_;
  std::optional<double> last_stop_at_;
  std::int64_t highest_request_id_{0};
};

}  // namespace helix_go2_sink_cpp

#endif  // HELIX_GO2_SINK_CPP__SPORT_GUARD_CORE_HPP_
