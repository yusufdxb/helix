// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// See include/helix_supervisor_cpp/supervisor_core.hpp for the contract.

#include "helix_supervisor_cpp/supervisor_core.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace helix_supervisor_cpp
{

namespace
{

constexpr const char * kAnomaly = "ANOMALY";
constexpr const char * kCrash = "CRASH";
constexpr const char * kLogPattern = "LOG_PATTERN";
constexpr std::int64_t kSeverityCritical = 3;

bool starts_with(const std::string & s, const std::string & prefix)
{
  return s.size() >= prefix.size() && s.compare(0, prefix.size(), prefix) == 0;
}

void require_positive(double v, const char * name)
{
  if (!(v > 0.0)) {
    throw std::invalid_argument(std::string(name) + " must be > 0");
  }
}

}  // namespace

const char * to_string(State s) noexcept
{
  switch (s) {
    case State::kStarting: return "STARTING";
    case State::kNominal: return "NOMINAL";
    case State::kHoldRequested: return "HOLD_REQUESTED";
    case State::kHoldApplied: return "HOLD_APPLIED";
    case State::kStopAcked: return "STOP_ACKED";
    case State::kStopConfirmed: return "STOP_CONFIRMED";
    case State::kStopUnconfirmed: return "STOP_UNCONFIRMED";
    case State::kClearing: return "CLEARING";
    case State::kAwaitRelease: return "AWAIT_RELEASE";
  }
  return "UNKNOWN";
}

void validate(const Config & c)
{
  require_positive(c.startup_grace_s, "startup_grace_s");
  require_positive(c.heartbeat_timeout_s, "heartbeat_timeout_s");
  require_positive(c.heartbeat_clear_s, "heartbeat_clear_s");
  require_positive(c.arbiter_ack_timeout_s, "arbiter_ack_timeout_s");
  require_positive(c.controller_ack_timeout_s, "controller_ack_timeout_s");
  require_positive(c.motion_stop_timeout_s, "motion_stop_timeout_s");
  require_positive(c.log_only_expiry_s, "log_only_expiry_s");
  if (c.clear_hold_s < 0.0) {
    throw std::invalid_argument("clear_hold_s must be >= 0");
  }
}

SupervisorCore::SupervisorCore(Config cfg)
: cfg_(std::move(cfg))
{
  validate(cfg_);
}

std::string SupervisorCore::key_of(const std::string & type, const std::string & source)
{
  return type + ":" + source;
}

bool SupervisorCore::is_critical(const std::string & node) const
{
  for (const auto & n : cfg_.critical_nodes) {
    if (n == "*" || n == node) {
      return true;
    }
  }
  return false;
}

void SupervisorCore::classify(Fault & f) const
{
  f.stop_worthy = false;
  f.latched = false;
  f.rule.clear();
  if (f.type == kAnomaly) {
    f.rule = "R1";
    if (f.severity >= cfg_.stop_min_severity) {
      for (const auto & p : cfg_.stop_metric_prefixes) {
        if (starts_with(f.source, p)) {
          f.stop_worthy = true;
          break;
        }
      }
    }
  } else if (f.type == kLogPattern) {
    f.rule = "R3";
    if (cfg_.stop_on_critical_log && f.severity >= kSeverityCritical) {
      f.stop_worthy = true;
      f.latched = true;
    }
  } else if (f.type == kCrash) {
    f.rule = "R4";
    f.stop_worthy = is_critical(f.source);
  }
}

void SupervisorCore::start(double now)
{
  started_ = true;
  started_at_ = now;
  startup_done_ = false;
  faults_.clear();
  nodes_.clear();
  for (const auto & n : cfg_.expected_nodes) {
    nodes_[n].expected = true;
  }
  episode_ = false;
  phase_ = ClearPhase::kNone;
  stage_ = Stage::kRequested;
  hold_id_.clear();
  hold_cause_.clear();
  arbiter_ack_at_ = controller_ack_at_ = motion_stopped_at_ = 0.0;
  hold_requested_at_ = cleared_at_ = 0.0;
  arbiter_conflict_ = motion_violation_ = false;
  have_fb_ = false;
  reason_ = "supervision started";
  last_state_ = State::kStarting;
  audit("TRANSITION", to_string(State::kStarting), "TRANSITION", "", reason_, 0.0, now);
  tick(now);
}

// -- events -------------------------------------------------------------------

void SupervisorCore::on_fault(
  const std::string & type, const std::string & source, std::int64_t severity,
  const std::string & detail, double now)
{
  if (!started_ || source.empty()) {
    return;
  }
  const std::string key = key_of(type, source);
  auto it = faults_.find(key);
  if (it == faults_.end()) {
    Fault f;
    f.key = key;
    f.type = type;
    f.source = source;
    it = faults_.emplace(key, std::move(f)).first;
  }
  Fault & f = it->second;
  f.severity = severity;
  f.detail = detail;
  classify(f);
  if (!f.active) {
    f.active = true;
    f.first_at = now;
    f.cleared_at = 0.0;
    f.activations += 1;
    bump("fault " + key);
  }
  f.last_at = now;
  finish(now);
}

void SupervisorCore::on_fault_clear(
  const std::string & type, const std::string & source, double now)
{
  if (!started_) {
    return;
  }
  auto it = faults_.find(key_of(type, source));
  if (it == faults_.end()) {
    return;   // e.g. the detector announcing a healthy metric at startup
  }
  Fault & f = it->second;
  // CRASH clears only through heartbeats; latched faults only by operator.
  if (!f.active || f.latched || f.type == kCrash) {
    return;
  }
  f.active = false;
  f.cleared_at = now;
  bump("cleared " + f.key);
  finish(now);
}

void SupervisorCore::on_heartbeat(const std::string & node, double now)
{
  if (!started_ || node.empty()) {
    return;
  }
  NodeBeat & b = nodes_[node];
  if (!b.last_beat || now - *b.last_beat > cfg_.heartbeat_timeout_s) {
    b.beating_since = now;
  }
  b.last_beat = now;
}

void SupervisorCore::on_arbiter_status(
  const std::string & reason, const std::string & hold_fault_id, double now)
{
  if (!episode_ || !cfg_.actuation_enabled) {
    return;
  }
  if (reason == "HELIX_HOLD" && hold_fault_id == hold_id_) {
    if (arbiter_ack_at_ == 0.0) {
      arbiter_ack_at_ = now;
    }
    arbiter_conflict_ = false;
  } else if (reason == "SOURCE" && now - hold_requested_at_ > cfg_.arbiter_ack_timeout_s) {
    // The arbiter passed a source command while our hold should be in force.
    if (!arbiter_conflict_) {
      arbiter_conflict_ = true;
      bump("arbiter published a source command during hold " + hold_id_);
    }
  }
  finish(now);
}

void SupervisorCore::on_sink_feedback(const SinkFeedback & fb, double now)
{
  last_fb_ = fb;
  have_fb_ = true;
  if (!episode_ || !cfg_.actuation_enabled) {
    return;
  }
  if (controller_ack_at_ == 0.0 && fb.stop_acked &&
    fb.stop_ack_sent_at >= hold_requested_at_)
  {
    controller_ack_at_ = std::max(fb.stop_acked_at, hold_requested_at_);
  }
  if (controller_ack_at_ > 0.0) {
    if (fb.stopped && fb.state_at >= controller_ack_at_ && motion_stopped_at_ == 0.0) {
      motion_stopped_at_ = fb.state_at;
      motion_violation_ = false;
    } else if (!fb.stopped && motion_stopped_at_ > 0.0 && fb.state_at > motion_stopped_at_) {
      motion_stopped_at_ = 0.0;
      motion_violation_ = true;
      bump("robot motion measured during hold " + hold_id_);
    }
  }
  finish(now);
}

void SupervisorCore::tick(double now)
{
  if (!started_) {
    return;
  }
  check_heartbeats(now);
  if (!startup_done_) {
    bool all_up = true;
    for (const auto & [name, b] : nodes_) {
      if (b.expected && is_critical(name) && (!b.last_beat || node_lost(b, now))) {
        all_up = false;
        break;
      }
    }
    if (all_up) {
      startup_done_ = true;
      bump("all critical expected nodes beating");
    } else if (now - started_at_ > cfg_.startup_grace_s) {
      // Missing nodes are now CRASH faults (check_heartbeats); a critical one
      // keeps the hold through an episode.
      startup_done_ = true;
      bump("startup grace expired with missing nodes");
    }
  }
  expire_log_only(now);
  finish(now);
}

bool SupervisorCore::operator_release(double now, std::string & why)
{
  if (!episode_) {
    why = "no hold to release";
    return false;
  }
  for (auto & [key, f] : faults_) {
    if (f.active && f.latched) {
      f.active = false;
      f.cleared_at = now;
      audit(
        "TRANSITION", "LATCH_CLEARED", "TRANSITION", f.rule,
        "operator cleared latched " + key, 0.0, now);
    }
  }
  if (any_stop_active()) {
    why = "stop-worthy faults still active:";
    for (const Fault * f : active_faults()) {
      if (f->stop_worthy) {
        why += " " + f->key;
      }
    }
    bump("operator release refused");
    finish(now);
    return false;
  }
  release(now, "operator release");
  finish(now);
  why = "released";
  return true;
}

void SupervisorCore::set_actuation_enabled(bool enabled, double now)
{
  if (enabled == cfg_.actuation_enabled) {
    return;
  }
  cfg_.actuation_enabled = enabled;
  if (episode_) {
    if (enabled) {
      restart_ack_chain(now);
      audit(
        "ACTION", "STOP_AND_HOLD", "ACCEPTED", "",
        "actuation enabled while fault " + hold_cause_ + " is active", 0.0, now);
    } else {
      audit(
        "ACTION", "STOP_AND_HOLD", "SUPPRESSED_DISABLED", "",
        "actuation disabled during hold " + hold_id_, 0.0, now);
    }
  }
  bump(enabled ? "actuation enabled" : "actuation disabled");
  finish(now);
}

void SupervisorCore::set_auto_resume(bool enabled, double now)
{
  if (enabled == cfg_.auto_resume) {
    return;
  }
  cfg_.auto_resume = enabled;
  bump(enabled ? "auto_resume enabled" : "auto_resume disabled");
  finish(now);
}

// -- outputs ------------------------------------------------------------------

State SupervisorCore::state() const noexcept
{
  if (!started_) {
    return State::kStarting;
  }
  if (episode_) {
    if (phase_ == ClearPhase::kClearing) {
      return State::kClearing;
    }
    if (phase_ == ClearPhase::kAwaitRelease) {
      return State::kAwaitRelease;
    }
    switch (stage_) {
      case Stage::kRequested: return State::kHoldRequested;
      case Stage::kApplied: return State::kHoldApplied;
      case Stage::kAcked: return State::kStopAcked;
      case Stage::kConfirmed: return State::kStopConfirmed;
      case Stage::kUnconfirmed: return State::kStopUnconfirmed;
    }
  }
  return startup_done_ ? State::kNominal : State::kStarting;
}

bool SupervisorCore::would_hold() const noexcept
{
  if (!started_) {
    return true;
  }
  return episode_ || !startup_done_;
}

std::vector<const Fault *> SupervisorCore::active_faults() const
{
  std::vector<const Fault *> out;
  for (const auto & [key, f] : faults_) {
    if (f.active) {
      out.push_back(&f);
    }
  }
  std::stable_sort(
    out.begin(), out.end(), [](const Fault * a, const Fault * b) {
      if (a->stop_worthy != b->stop_worthy) {
        return a->stop_worthy;
      }
      return a->first_at < b->first_at;
    });
  return out;
}

std::vector<std::string> SupervisorCore::missing_nodes(double now) const
{
  std::vector<std::string> out;
  for (const auto & [name, b] : nodes_) {
    if ((b.expected && !b.last_beat) || (b.last_beat && node_lost(b, now))) {
      out.push_back(name);
    }
  }
  return out;
}

const Fault * SupervisorCore::fault(const std::string & key) const
{
  auto it = faults_.find(key);
  return it == faults_.end() ? nullptr : &it->second;
}

std::vector<AuditEvent> SupervisorCore::take_audit()
{
  std::vector<AuditEvent> out;
  out.swap(audit_);
  return out;
}

// -- internals ----------------------------------------------------------------

bool SupervisorCore::node_lost(const NodeBeat & b, double now) const
{
  if (b.last_beat) {
    return now - *b.last_beat > cfg_.heartbeat_timeout_s;
  }
  return b.expected && now - started_at_ > cfg_.startup_grace_s;
}

void SupervisorCore::check_heartbeats(double now)
{
  for (const auto & [name, b] : nodes_) {
    const std::string key = key_of(kCrash, name);
    auto it = faults_.find(key);
    if (node_lost(b, now)) {
      if (it == faults_.end() || !it->second.active) {
        const std::string detail = b.last_beat ?
          "heartbeat lost for " + std::to_string(now - *b.last_beat) + " s" :
          "no heartbeat within startup_grace_s (never started)";
        on_fault(kCrash, name, kSeverityCritical, detail, now);
      } else {
        it->second.last_at = now;
      }
    } else if (b.last_beat && it != faults_.end() && it->second.active) {
      // Continuous beating for heartbeat_clear_s after the last evidence of
      // the crash is the clearance proof.
      const double since = std::max(b.beating_since, it->second.last_at);
      if (now - since >= cfg_.heartbeat_clear_s) {
        it->second.active = false;
        it->second.cleared_at = now;
        bump("heartbeat restored " + name);
      }
    }
  }
}

void SupervisorCore::expire_log_only(double now)
{
  for (auto & [key, f] : faults_) {
    if (f.active && !f.stop_worthy && f.type != kCrash &&
      now - f.last_at > cfg_.log_only_expiry_s)
    {
      f.active = false;
      f.cleared_at = now;
      bump("log-only fault expired " + key);
    }
  }
}

bool SupervisorCore::any_stop_active() const
{
  for (const auto & [key, f] : faults_) {
    if (f.active && f.stop_worthy) {
      return true;
    }
  }
  return false;
}

bool SupervisorCore::latched_active() const
{
  for (const auto & [key, f] : faults_) {
    if (f.active && f.latched) {
      return true;
    }
  }
  return false;
}

const Fault * SupervisorCore::first_stop_fault() const
{
  for (const Fault * f : active_faults()) {
    if (f->stop_worthy) {
      return f;
    }
  }
  return nullptr;
}

void SupervisorCore::evaluate(double now)
{
  const bool stop = any_stop_active();
  if (!episode_) {
    if (stop) {
      open_episode(now);
    }
    return;
  }
  if (stop) {
    if (latched_active()) {
      episode_latched_ = true;
    }
    if (phase_ != ClearPhase::kNone) {
      const Fault * f = first_stop_fault();
      set_phase(
        ClearPhase::kNone, now,
        "stop-worthy fault re-asserted: " + (f ? f->key : std::string("?")));
    }
    return;
  }
  if (phase_ == ClearPhase::kNone) {
    cleared_at_ = now;
    set_phase(ClearPhase::kClearing, now, "every stop-worthy fault cleared");
  }
  if (phase_ == ClearPhase::kClearing && now - cleared_at_ >= cfg_.clear_hold_s) {
    if (cfg_.auto_resume && !episode_latched_) {
      release(now, "faults stayed clear for clear_hold_s (auto_resume)");
      return;
    }
    set_phase(
      ClearPhase::kAwaitRelease, now, episode_latched_ ?
      "latched fault in this episode: operator release required" :
      "auto_resume false: operator release required");
  }
  if (phase_ == ClearPhase::kAwaitRelease && cfg_.auto_resume && !episode_latched_) {
    release(now, "auto_resume enabled after clearance");
  }
}

void SupervisorCore::open_episode(double now)
{
  const Fault * f = first_stop_fault();
  episode_ = true;
  episodes_ += 1;
  phase_ = ClearPhase::kNone;
  hold_cause_ = f->key;
  hold_id_ = f->source;
  episode_latched_ = latched_active();
  restart_ack_chain(now);
  const double confidence = f->rule == "R1" ? 0.9 : (f->rule == "R3" ? 0.7 : 0.8);
  const std::string why = f->rule + " " + f->key + (f->detail.empty() ? "" : ": " + f->detail);
  audit("HINT", "STOP_AND_HOLD", "", f->rule, why, confidence, now);
  audit(
    "ACTION", "STOP_AND_HOLD",
    cfg_.actuation_enabled ? "ACCEPTED" : "SUPPRESSED_DISABLED", f->rule,
    cfg_.actuation_enabled ? "hold asserted" : "actuation disabled: hold not published",
    confidence, now);
  bump("hold episode " + std::to_string(episodes_) + " opened by " + f->key);
}

void SupervisorCore::release(double now, const std::string & why)
{
  audit("HINT", "RESUME", "", "R2", why, 0.9, now);
  audit(
    "ACTION", "RESUME", cfg_.actuation_enabled ? "ACCEPTED" : "SUPPRESSED_DISABLED",
    "R2", why, 0.9, now);
  episode_ = false;
  phase_ = ClearPhase::kNone;
  stage_ = Stage::kRequested;
  episode_latched_ = false;
  hold_id_.clear();
  hold_cause_.clear();
  arbiter_ack_at_ = controller_ack_at_ = motion_stopped_at_ = 0.0;
  hold_requested_at_ = 0.0;
  arbiter_conflict_ = motion_violation_ = false;
  bump(why);
}

void SupervisorCore::restart_ack_chain(double now)
{
  hold_requested_at_ = now;
  arbiter_ack_at_ = controller_ack_at_ = motion_stopped_at_ = 0.0;
  arbiter_conflict_ = motion_violation_ = false;
  stage_ = Stage::kRequested;
}

void SupervisorCore::update_stage(double now)
{
  if (!episode_ || !cfg_.actuation_enabled) {
    stage_ = Stage::kRequested;   // advisory: nothing to acknowledge
    return;
  }
  Stage proven = Stage::kRequested;
  double proven_at = hold_requested_at_;
  double timeout = cfg_.arbiter_ack_timeout_s;
  if (arbiter_ack_at_ > 0.0 && !arbiter_conflict_) {
    proven = Stage::kApplied;
    proven_at = arbiter_ack_at_;
    timeout = cfg_.controller_ack_timeout_s;
    if (cfg_.require_controller_ack && controller_ack_at_ > 0.0) {
      proven = Stage::kAcked;
      proven_at = controller_ack_at_;
      timeout = cfg_.motion_stop_timeout_s;
      if (motion_stopped_at_ > 0.0) {
        proven = Stage::kConfirmed;
      }
    }
  }
  bool timed_out = false;
  if (arbiter_conflict_ || motion_violation_) {
    timed_out = true;
  } else if (proven == Stage::kConfirmed) {
    timed_out = false;
  } else if (proven == Stage::kApplied && !cfg_.require_controller_ack) {
    timed_out = false;   // terminal stage without controller feedback
  } else {
    timed_out = now - proven_at > timeout;
  }
  const Stage next = timed_out ? Stage::kUnconfirmed : proven;
  if (next != stage_) {
    std::string why;
    if (next == Stage::kUnconfirmed) {
      if (arbiter_conflict_) {
        why = "arbiter passed a source command while holding";
      } else if (motion_violation_) {
        why = "robot motion measured while holding";
      } else if (proven == Stage::kRequested) {
        why = "no arbiter HELIX_HOLD for this hold within arbiter_ack_timeout_s";
      } else if (proven == Stage::kApplied) {
        why = "no controller StopMove ack within controller_ack_timeout_s";
      } else {
        why = "measured motion not below threshold within motion_stop_timeout_s";
      }
    } else {
      why = next == Stage::kApplied ? "arbiter forced zero for this hold" :
        next == Stage::kAcked ? "robot controller acknowledged StopMove" :
        next == Stage::kConfirmed ? "measured robot motion below stop threshold" :
        "hold requested";
    }
    stage_ = next;
    reason_ = why;
  }
}

void SupervisorCore::set_phase(ClearPhase p, double /*now*/, const std::string & why)
{
  phase_ = p;
  bump(why);
}

void SupervisorCore::audit(
  const std::string & kind, const std::string & action, const std::string & status,
  const std::string & rule, const std::string & reason, double confidence, double now)
{
  AuditEvent e;
  e.kind = kind;
  e.fault_id = hold_id_;
  e.action = action;
  e.status = status;
  e.rule = rule;
  e.reason = reason;
  e.confidence = confidence;
  e.stamp = now;
  audit_.push_back(std::move(e));
}

void SupervisorCore::bump(const std::string & why)
{
  reason_ = why;
  revision_ += 1;
}

void SupervisorCore::finish(double now)
{
  evaluate(now);
  update_stage(now);
  const State s = state();
  if (s != last_state_) {
    // The hold id of a closing episode is gone by now; RESUME audit carries it.
    audit("TRANSITION", to_string(s), "TRANSITION", "", reason_, 0.0, now);
    last_state_ = s;
    revision_ += 1;
  }
}

}  // namespace helix_supervisor_cpp
