// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// SupervisorCore: the single HELIX fault-state machine. ROS-free; the node
// feeds it events and a steady-clock time on every call, so every decision
// is deterministic and replayable.
//
// It replaces three Python components that each kept their own state and
// could disagree: HeartbeatMonitor (node liveness), DiagnosisNode (rules and
// a STOP/IDLE state) and RecoveryNode (enable flag, cooldown, hold).
//
// Faults
//   Every fault has a key "<TYPE>:<source>" and is active until it is
//   cleared by evidence:
//     ANOMALY      cleared by the detector's clear record for that metric
//                  (/helix/fault_clear), i.e. the metric was measured normal
//                  again for clear_samples samples. Never by silence.
//     CRASH        cleared when that node's heartbeat has been continuous
//                  for heartbeat_clear_s after the fault.
//     LOG_PATTERN  critical ones are LATCHED: only an operator release
//                  clears them (a log line has no "recovered" counterpart).
//   Log-only faults (not stop-worthy) that have no clear signal expire after
//   log_only_expiry_s; they never affect the hold.
//
// Stop-worthy (the hold decision)
//   R1  ANOMALY on a metric matching stop_metric_prefixes, severity >= 2
//   R3  LOG_PATTERN with severity 3 (CRITICAL)
//   R4  CRASH of a node in critical_nodes ("*" = every node). The Python
//       rule set mapped every CRASH to LOG_ONLY.
//   Heartbeats: expected_nodes are seeded at start; one never seen within
//   startup_grace_s raises a CRASH ("never started"), so a node that fails
//   to start is no longer invisible.
//
// Hold episode (what /helix/hold carries)
//   STARTING -> (all critical expected nodes beating) -> NOMINAL
//   NOMINAL  -> stop-worthy fault -> HOLD_REQUESTED -> arbiter ack ->
//   HOLD_APPLIED -> controller ack -> STOP_ACKED -> measured stop ->
//   STOP_CONFIRMED. A missing ack or measured stop past its timeout gives
//   STOP_UNCONFIRMED (hold stays asserted); late evidence still advances it.
//   Motion measured while confirmed drops back to STOP_UNCONFIRMED.
//   All stop-worthy faults cleared -> CLEARING; still clear after
//   clear_hold_s -> release (auto_resume and no latched fault in the
//   episode) or AWAIT_RELEASE (operator). Any stop-worthy fault during
//   CLEARING / AWAIT_RELEASE returns to the ack stage of the same episode.
//
// actuation_enabled gates only the published hold. The same machine runs
// either way: hold is would_hold && actuation_enabled, applied immediately
// when the flag flips. The native arbiter separately owns protective latching;
// a false supervisor hold never authorizes release of that arbiter latch.
#ifndef HELIX_SUPERVISOR_CPP__SUPERVISOR_CORE_HPP_
#define HELIX_SUPERVISOR_CPP__SUPERVISOR_CORE_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace helix_supervisor_cpp
{

enum class State : std::uint8_t
{
  kStarting,
  kNominal,
  kHoldRequested,
  kHoldApplied,
  kStopAcked,
  kStopConfirmed,
  kStopUnconfirmed,
  kClearing,
  kAwaitRelease,
};

const char * to_string(State s) noexcept;

struct Config
{
  std::vector<std::string> stop_metric_prefixes{"rate_hz/utlidar"};
  std::int64_t stop_min_severity{2};
  bool stop_on_critical_log{true};
  std::vector<std::string> critical_nodes;   // "*" = every node
  std::vector<std::string> expected_nodes;   // seeded into the heartbeat registry
  double startup_grace_s{5.0};
  double heartbeat_timeout_s{0.35};
  double heartbeat_clear_s{1.0};
  double clear_hold_s{2.0};
  double log_only_expiry_s{10.0};
  bool auto_resume{false};
  bool actuation_enabled{false};
  bool require_controller_ack{true};
  double arbiter_ack_timeout_s{0.25};
  double controller_ack_timeout_s{1.5};
  double motion_stop_timeout_s{3.0};
};

/// Throws std::invalid_argument for a configuration that cannot be safe
/// (non-positive timeouts, negative windows).
void validate(const Config & c);

struct Fault
{
  std::string key;
  std::string type;
  std::string source;
  std::string rule;        // R1 | R3 | R4 | '' (log only)
  std::int64_t severity{0};
  bool stop_worthy{false};
  bool latched{false};
  bool active{false};
  double first_at{0.0};    // start of the current activation
  double last_at{0.0};     // newest evidence of this activation
  double cleared_at{0.0};
  std::uint32_t activations{0};
  std::string detail;
};

/// A record the node publishes for the audit trail. kind is HINT, ACTION or
/// TRANSITION. HINT/ACTION reuse the RecoveryHint/RecoveryAction schema so
/// existing trace tooling keeps working.
struct AuditEvent
{
  std::string kind;
  std::string fault_id;
  std::string action;      // STOP_AND_HOLD | RESUME | state name
  std::string status;      // ACCEPTED | SUPPRESSED_DISABLED | TRANSITION
  std::string rule;
  std::string reason;
  double confidence{0.0};
  double stamp{0.0};
};

struct SinkFeedback
{
  bool feedback_available{false};
  bool stop_acked{false};
  double stop_ack_sent_at{0.0};   // when the acknowledged StopMove was sent
  double stop_acked_at{0.0};
  bool stopped{false};
  double state_at{0.0};           // receipt of the newest robot state
};

class SupervisorCore
{
public:
  explicit SupervisorCore(Config cfg);

  /// Begin (or restart) supervision at @p now: seeds the heartbeat registry
  /// and enters STARTING.
  void start(double now);

  // -- events --------------------------------------------------------------
  void on_fault(
    const std::string & type, const std::string & source, std::int64_t severity,
    const std::string & detail, double now);
  void on_fault_clear(const std::string & type, const std::string & source, double now);
  void on_heartbeat(const std::string & node, double now);
  void on_arbiter_status(
    const std::string & reason, const std::string & hold_fault_id, double now);
  void on_sink_feedback(const SinkFeedback & fb, double now);
  void tick(double now);

  /// Operator release: clears latched faults, then releases the hold if no
  /// other stop-worthy fault is active. Returns false (with @p why) if refused.
  bool operator_release(double now, std::string & why);

  void set_actuation_enabled(bool enabled, double now);
  void set_auto_resume(bool enabled, double now);

  // -- outputs -------------------------------------------------------------
  State state() const noexcept;
  bool would_hold() const noexcept;
  bool hold() const noexcept {return would_hold() && cfg_.actuation_enabled;}
  const Config & config() const noexcept {return cfg_;}
  const std::string & hold_id() const noexcept {return hold_id_;}
  const std::string & hold_cause() const noexcept {return hold_cause_;}
  const std::string & reason() const noexcept {return reason_;}
  std::uint64_t revision() const noexcept {return revision_;}
  std::uint32_t hold_episodes() const noexcept {return episodes_;}
  double hold_requested_at() const noexcept {return hold_requested_at_;}
  double arbiter_ack_at() const noexcept {return arbiter_ack_at_;}
  double controller_ack_at() const noexcept {return controller_ack_at_;}
  double motion_stopped_at() const noexcept {return motion_stopped_at_;}
  double cleared_at() const noexcept {return cleared_at_;}

  /// Active faults, stop-worthy first, then by activation time.
  std::vector<const Fault *> active_faults() const;
  std::vector<std::string> missing_nodes(double now) const;
  const Fault * fault(const std::string & key) const;

  bool feedback_seen() const noexcept {return have_fb_;}
  const SinkFeedback & last_feedback() const noexcept {return last_fb_;}

  /// Audit records produced since the last call.
  std::vector<AuditEvent> take_audit();

private:
  enum class Stage : std::uint8_t {kRequested, kApplied, kAcked, kConfirmed, kUnconfirmed};
  enum class ClearPhase : std::uint8_t {kNone, kClearing, kAwaitRelease};

  struct NodeBeat
  {
    bool expected{false};
    std::optional<double> last_beat;
    double beating_since{0.0};
  };

  static std::string key_of(const std::string & type, const std::string & source);
  void classify(Fault & f) const;
  bool is_critical(const std::string & node) const;
  bool node_lost(const NodeBeat & b, double now) const;
  bool any_stop_active() const;
  bool latched_active() const;
  const Fault * first_stop_fault() const;
  void evaluate(double now);
  void open_episode(double now);
  void release(double now, const std::string & why);
  void restart_ack_chain(double now);
  void update_stage(double now);
  void finish(double now);
  void set_phase(ClearPhase p, double now, const std::string & why);
  void audit(
    const std::string & kind, const std::string & action,
    const std::string & status, const std::string & rule, const std::string & reason,
    double confidence, double now);
  void bump(const std::string & why);
  void check_heartbeats(double now);
  void expire_log_only(double now);

  Config cfg_;
  bool started_{false};
  double started_at_{0.0};
  bool startup_done_{false};
  std::map<std::string, Fault> faults_;
  std::map<std::string, NodeBeat> nodes_;

  bool episode_{false};
  Stage stage_{Stage::kRequested};
  ClearPhase phase_{ClearPhase::kNone};
  bool episode_latched_{false};
  std::string hold_id_;
  std::string hold_cause_;
  std::string reason_{"not started"};
  std::uint32_t episodes_{0};
  double hold_requested_at_{0.0};
  double arbiter_ack_at_{0.0};
  double controller_ack_at_{0.0};
  double motion_stopped_at_{0.0};
  double cleared_at_{0.0};
  bool arbiter_conflict_{false};
  bool motion_violation_{false};
  State last_state_{State::kStarting};
  SinkFeedback last_fb_;
  bool have_fb_{false};
  std::uint64_t revision_{0};
  std::vector<AuditEvent> audit_;
};

}  // namespace helix_supervisor_cpp

#endif  // HELIX_SUPERVISOR_CPP__SUPERVISOR_CORE_HPP_
