#include "helix_healer/policy.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace helix_healer {

const char* to_string(Fault f) {
  switch (f) {
    case Fault::None: return "none";
    case Fault::Crash: return "crash";
    case Fault::CrashLoop: return "crash_loop";
    case Fault::Hang: return "hang";
    case Fault::EndpointLoss: return "endpoint_loss";
    case Fault::CpuRunaway: return "cpu_runaway";
    case Fault::MemoryLeak: return "memory_leak";
    case Fault::Degraded: return "degraded";
    case Fault::QosMismatch: return "qos_mismatch";
    case Fault::Upstream: return "upstream";
  }
  return "?";
}

const char* to_string(Action a) {
  switch (a) {
    case Action::None: return "none";
    case Action::Restart: return "restart";
    case Action::LifecycleReset: return "lifecycle_reset";
    case Action::Fallback: return "fallback";
    case Action::Report: return "report";
  }
  return "?";
}

PolicyKind policy_kind(const std::string& name) {
  if (name == "none") return PolicyKind::None;
  if (name == "respawn") return PolicyKind::Respawn;
  if (name == "watchdog") return PolicyKind::Watchdog;
  if (name == "helix") return PolicyKind::Helix;
  throw std::runtime_error("unknown policy '" + name + "' (none|respawn|watchdog|helix)");
}

Policy::Policy(PolicyKind kind, const Spec& spec) : kind_(kind), spec_(spec) {
  for (const auto& n : spec_.nodes) st_[n.name];
}

bool Policy::healthy(const std::string& node) const {
  auto it = st_.find(node);
  if (it == st_.end()) return false;
  const State& st = it->second;
  return st.phase == Phase::Normal && !st.sick && st.pending == Fault::None;
}

std::string Policy::phase(const std::string& node) const {
  auto it = st_.find(node);
  if (it == st_.end()) return "unknown";
  switch (it->second.phase) {
    case Phase::Normal: return it->second.sick ? "symptomatic" : "ok";
    case Phase::Acting: return "repairing";
    case Phase::Verifying: return "verifying";
    case Phase::Retired: return "retired";
  }
  return "?";
}

namespace {
double stale_limit(const HealerParams& p, double hz) { return std::max(p.stale_min_s, p.stale_periods / hz); }
}  // namespace

bool Policy::output_symptom(const NodeObs& n, bool* stale, bool* own_missing, double* oldest, bool* sharp) const {
  const auto& p = spec_.p;
  const bool adaptive = kind_ == PolicyKind::Helix && p.adaptive > 0;
  bool st = false, miss = false, slow = false, sh = false;
  double old = 0;
  for (const auto& o : n.outputs) {
    if (o.expected_hz <= 0) continue;
    const bool env = adaptive && o.stale_limit_s > 0;
    const double limit = env ? o.stale_limit_s : stale_limit(p, o.expected_hz);
    if (o.age_s > limit) {
      st = true;
      sh = sh || env;
      old = std::max(old, o.age_s);
      if (!o.own_endpoint) miss = true;
    } else if (adaptive && o.deficit_limit > 0 && o.deficit > o.deficit_limit) {
      slow = true;
      sh = true;
    } else if (o.rate_hz < p.degraded_ratio * o.expected_hz) {
      slow = true;
    }
  }
  if (stale) *stale = st;
  if (own_missing) *own_missing = miss;
  if (oldest) *oldest = old;
  if (sharp) *sharp = sh;
  return st || slow;
}

bool Policy::leaking(const NodeObs& n) const {
  const auto& p = spec_.p;
  const NodeSpec& ns = *spec_.find(n.name);
  if (!n.alive) return false;
  if (n.rss_mb > ns.rss_max_mb) return true;
  const double growth = n.rss_mb - n.rss_base;
  if (n.rss_slope <= 0 || growth < p.leak_min_growth_mb) return false;
  return (ns.rss_max_mb - n.rss_mb) / n.rss_slope < p.leak_horizon_s;
}

void Policy::observe(const NodeObs& n, const Snapshot& s, State& st) {
  const auto& p = spec_.p;
  const NodeSpec& ns = *spec_.find(n.name);
  const double t = s.t;
  if (n.alive && n.cpu > ns.cpu_max) {
    if (st.cpu_high_since < 0) st.cpu_high_since = t;
  } else {
    st.cpu_high_since = -1;
  }
  if (st.cpu_high_since >= 0 || leaking(n)) st.resource_until = t + p.victim_grace_s;

  bool stale = false;
  double oldest = 0;
  const bool symptom = n.alive && n.up_s >= p.startup_grace_s && output_symptom(n, &stale, nullptr, &oldest);
  bool sick;
  if (st.phase == Phase::Retired) {
    // A retired node with a running replacement is the replacement's problem now; one whose
    // replacement failed (or that had none) stays a root so its consumers are not blamed.
    sick = !st.replaced;
  } else {
    sick = !n.alive || symptom || st.phase == Phase::Acting || st.phase == Phase::Verifying;
  }
  if (sick) {
    if (st.onset == State::kNoOnset) {
      if (!n.alive) st.onset = t - n.down_s;
      else if (stale) st.onset = t - oldest;
      else st.onset = t;
    }
    st.sick = true;
  } else {
    if (st.sick) st.healthy_since = t;
    st.sick = false;
    st.onset = State::kNoOnset;
  }
}

bool Policy::upstream_sick(const NodeObs& n, const State& self, double t) const {
  const double grace = spec_.p.victim_grace_s;
  for (const auto& u : n.upstream) {
    if (u == n.name) continue;
    auto it = st_.find(u);
    if (it == st_.end()) continue;
    const State& us = it->second;
    // Upstream went bad no later than we did: we are downstream of its fault.
    if (us.sick && us.onset != State::kNoOnset && (self.onset == State::kNoOnset || us.onset <= self.onset)) return true;
    // Upstream just recovered: give us time to follow before blaming us.
    if (!us.sick && us.healthy_since > 0 && t - us.healthy_since < grace) return true;
  }
  return false;
}

bool Policy::resource_root_elsewhere(const std::string& self, double t) const {
  for (const auto& [name, st] : st_)
    if (name != self && st.phase != Phase::Retired && st.resource_until > t) return true;
  return false;
}

// A stale node whose input went silent no later than its own output is starved, not hung,
// even before the upstream node is itself judged sick (it may have a longer stale limit).
bool Policy::input_silent_longer(const NodeObs& n, const Snapshot& s) const {
  const NodeSpec* ns = spec_.find(n.name);
  if (!ns || ns->inputs.empty()) return false;
  double mine = 0;
  for (const auto& o : n.outputs) mine = std::max(mine, o.expected_hz > 0 ? o.age_s : 0.0);
  for (const auto& u : n.upstream) {
    if (u == n.name) continue;
    for (const auto& un : s.nodes) {
      if (un.name != u) continue;
      for (const auto& o : un.outputs)
        if (o.expected_hz > 0 && std::find(ns->inputs.begin(), ns->inputs.end(), o.topic) != ns->inputs.end() &&
            o.age_s + 1e-3 >= mine)
          return true;
    }
  }
  return false;
}

Fault Policy::diagnose(const NodeObs& n, const Snapshot& s) {
  const auto& p = spec_.p;
  State& st = st_[n.name];
  const double t = s.t;
  if (!n.alive) return n.quick_deaths >= p.loop_deaths ? Fault::CrashLoop : Fault::Crash;
  if (leaking(n)) return Fault::MemoryLeak;
  if (st.cpu_high_since >= 0) return t - st.cpu_high_since >= p.cpu_sustain_s ? Fault::CpuRunaway : Fault::None;
  if (n.up_s < p.startup_grace_s) return Fault::None;
  bool stale = false, missing = false, sharp = false;
  if (!output_symptom(n, &stale, &missing, nullptr, &sharp)) return Fault::None;
  st.sharp = sharp;
  if (n.qos_incompatible) return Fault::QosMismatch;
  if (upstream_sick(n, st, t) || resource_root_elsewhere(n.name, t)) return Fault::Upstream;
  if (stale && input_silent_longer(n, s)) return Fault::Upstream;
  if (stale) return missing ? Fault::EndpointLoss : Fault::Hang;
  return Fault::Degraded;
}

Fault Policy::baseline_fault(const NodeObs& n, State&, double) {
  const auto& p = spec_.p;
  switch (kind_) {
    case PolicyKind::None:
    case PolicyKind::Helix:
      return Fault::None;
    case PolicyKind::Respawn:
      return n.alive ? Fault::None : Fault::Crash;
    case PolicyKind::Watchdog: {
      if (!n.alive) return Fault::Crash;
      if (n.rss_mb > spec_.find(n.name)->rss_max_mb) return Fault::MemoryLeak;  // MemoryMax-style hard limit
      bool stale = false;
      if (n.up_s >= p.startup_grace_s && output_symptom(n, &stale, nullptr, nullptr) && stale) return Fault::Hang;
      return Fault::None;
    }
  }
  return Fault::None;
}

int Policy::budget_used(State& st, double t) {
  while (!st.attempts.empty() && t - st.attempts.front() > spec_.p.window_s) st.attempts.pop_front();
  return static_cast<int>(st.attempts.size());
}

Decision Policy::decide(const NodeObs& n, Fault f, State& st, double t, StepOut& out) {
  const auto& p = spec_.p;
  const NodeSpec& ns = *spec_.find(n.name);
  Decision d;
  d.node = n.name;
  d.fault = f;
  const int used = budget_used(st, t);
  d.attempt = used + 1;
  auto backoff = [&](int k) { return std::min(p.backoff_max_s, p.backoff_s * std::pow(2.0, k)); };
  // Terminal action: hand off to a backup, request a safe stop, or (non-critical) just report.
  auto give_up = [&](const std::string& why) {
    d.action = (!ns.fallback.empty() || ns.critical) ? Action::Fallback : Action::Report;
    d.detail = why;
  };

  if (kind_ == PolicyKind::Respawn) {
    d.action = Action::Restart;
    d.delay_s = p.fast_repair > 0 ? 0.0 : p.backoff_s;  // launch respawn=true uses no delay by default
  } else if (kind_ == PolicyKind::Watchdog) {
    if (used >= p.max_attempts) {
      give_up("budget");
    } else {
      d.action = Action::Restart;
      d.delay_s = (n.alive || (p.fast_repair > 0 && used == 0)) ? 0.0 : p.backoff_s;
    }
  } else {  // Helix
    if (f == Fault::QosMismatch) give_up("qos_mismatch: restart cannot fix a QoS contract");
    else if (!ns.restart) give_up("restart=0");
    else if (f == Fault::CrashLoop) give_up("crash_loop");
    else if (used >= p.max_attempts) give_up("budget");
    // A lifecycle reset re-runs the node's own cleanup/configure code, which fixes
    // corrupted state and lost endpoints. It cannot unblock a stuck executor (a
    // hung node does not even answer the lifecycle services), so a hang restarts.
    else if (ns.lifecycle && (f == Fault::Degraded || f == Fault::EndpointLoss)) d.action = Action::LifecycleReset;
    else {
      d.action = Action::Restart;
      d.delay_s = ((n.alive || p.fast_repair > 0) && used == 0) ? 0.0 : backoff(used);
    }
  }

  if (d.action == Action::Restart || d.action == Action::LifecycleReset) st.attempts.push_back(t);
  st.last_fault = f;
  st.last_action = d.action;
  st.phase = d.action == Action::Report ? Phase::Retired : Phase::Acting;
  out.events.push_back({t, n.name, d.action == Action::Fallback || d.action == Action::Report ? "given_up" : "act", f,
                        d.action, d.attempt, d.detail});
  return d;
}

StepOut Policy::step(const Snapshot& s) {
  StepOut out;
  out.events = std::move(pending_events_);
  pending_events_.clear();
  const auto& p = spec_.p;
  const double t = s.t;
  for (const auto& n : s.nodes) observe(n, s, st_[n.name]);
  if (kind_ == PolicyKind::None) return out;

  for (const auto& n : s.nodes) {
    State& st = st_[n.name];
    if (st.phase == Phase::Retired || st.phase == Phase::Acting) continue;
    if (st.phase == Phase::Verifying) {
      bool symptom = output_symptom(n, nullptr, nullptr, nullptr);
      if (n.alive && !symptom) {
        st.phase = Phase::Normal;
        out.events.push_back({t, n.name, "verified", st.last_fault, st.last_action, 0, ""});
        continue;
      }
      if (n.alive && t < st.verify_deadline) continue;
      st.phase = Phase::Normal;
      out.events.push_back({t, n.name, "verify_failed", st.last_fault, st.last_action, 0, n.alive ? "timeout" : "died"});
      if (n.alive && st.last_action == Action::LifecycleReset) st.escalate = true;
    }
    if (st.escalate) {
      st.escalate = false;
      Decision d;
      d.node = n.name;
      d.fault = st.last_fault;
      d.action = Action::Restart;
      d.attempt = budget_used(st, t);
      d.detail = "escalated from lifecycle_reset";
      st.last_action = Action::Restart;
      st.phase = Phase::Acting;
      out.events.push_back({t, n.name, "escalate", d.fault, d.action, d.attempt, d.detail});
      out.decisions.push_back(d);
      continue;
    }

    const Fault f = kind_ == PolicyKind::Helix ? diagnose(n, s) : baseline_fault(n, st, t);
    if (f != st.pending) {
      st.pending = f;
      st.pending_since = t;
      st.victim_reported = false;
    }
    if (f == Fault::None) continue;
    const bool immediate = f == Fault::Crash || f == Fault::CrashLoop;
    // A symptom flagged by a ready envelope is already judged against this output's own
    // jitter, so it needs only a short confirmation. Resource and fixed-rule faults keep confirm_s.
    const bool symptom = f == Fault::Hang || f == Fault::EndpointLoss || f == Fault::Degraded;
    const double confirm = (kind_ == PolicyKind::Helix && symptom && st.sharp) ? p.adaptive_confirm_s : p.confirm_s;
    if (!immediate && t - st.pending_since < confirm) continue;
    if (f == Fault::Upstream) {
      if (!st.victim_reported) {
        st.victim_reported = true;
        out.events.push_back({t, n.name, "victim", f, Action::None, 0, "symptom explained by an upstream or resource root"});
      }
      continue;
    }
    st.pending = Fault::None;
    out.decisions.push_back(decide(n, f, st, t, out));
  }
  return out;
}

void Policy::on_result(const std::string& node, Action a, bool ok, double t) {
  auto it = st_.find(node);
  if (it == st_.end()) return;
  State& st = it->second;
  const auto& p = spec_.p;
  if (a == Action::Fallback || a == Action::Report) {
    st.phase = Phase::Retired;
    const NodeSpec* ns = spec_.find(node);
    st.replaced = a == Action::Fallback && ok && ns && !ns->fallback.empty();
    if (a == Action::Fallback && ns && !ns->fallback.empty()) {
      State& b = st_[ns->fallback];
      b.phase = ok ? Phase::Verifying : Phase::Normal;
      b.verify_deadline = t + p.verify_s;
      b.last_fault = st.last_fault;
      b.last_action = Action::Fallback;
    }
    const bool has_backup = ns && !ns->fallback.empty();
    std::string detail = has_backup ? "-> " + ns->fallback : "no backup: SAFE_STOP";
    if (a == Action::Report) detail = "reported: SAFE_STOP";
    pending_events_.push_back({t, node, ok ? "fallback" : "fallback_failed", st.last_fault, a, 0, detail});
    return;
  }
  if (ok) {
    st.phase = Phase::Verifying;
    st.verify_deadline = t + p.verify_s;
  } else {
    st.phase = Phase::Normal;
    pending_events_.push_back({t, node, "action_failed", st.last_fault, a, 0, ""});
    if (a == Action::LifecycleReset) st.escalate = true;
  }
}

}  // namespace helix_healer
