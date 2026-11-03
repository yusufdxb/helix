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
    case Fault::CommonMode: return "common_mode";
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
    case Action::ParamRestore: return "param_restore";
    case Action::RestartLKG: return "restart_lkg";
  }
  return "?";
}

Action action_from(const std::string& name) {
  for (Action a : {Action::Restart, Action::LifecycleReset, Action::Fallback, Action::Report, Action::ParamRestore,
                   Action::RestartLKG})
    if (name == to_string(a)) return a;
  return Action::None;
}

PolicyKind policy_kind(const std::string& name) {
  if (name == "none") return PolicyKind::None;
  if (name == "respawn") return PolicyKind::Respawn;
  if (name == "watchdog") return PolicyKind::Watchdog;
  if (name == "helix") return PolicyKind::Helix;
  if (name == "agent") return PolicyKind::Agent;
  throw std::runtime_error("unknown policy '" + name + "' (none|respawn|watchdog|helix|agent)");
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
  const bool adaptive = diagnoses() && p.adaptive > 0;
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

// True if data can flow from a to b or from b to a, through any chain of upstream links.
bool Policy::linked(const std::string& a, const std::string& b, const Snapshot& s) const {
  auto reaches = [&](const std::string& from, const std::string& to) {
    std::vector<std::string> todo{from}, seen{from};
    while (!todo.empty()) {
      const std::string cur = todo.back();
      todo.pop_back();
      for (const auto& nn : s.nodes) {
        if (nn.name != cur) continue;
        for (const auto& u : nn.upstream) {
          if (u == to) return true;
          if (std::find(seen.begin(), seen.end(), u) == seen.end()) {
            seen.push_back(u);
            todo.push_back(u);
          }
        }
      }
    }
    return false;
  };
  return reaches(a, b) || reaches(b, a);
}

// A symptom that starts together with one on an unrelated, alive, unrepaired node has a shared
// cause outside both. Holding it avoids restarting healthy processes for a host or middleware stall.
bool Policy::common_mode(const NodeObs& n, const State& self, const Snapshot& s) const {
  const auto& p = spec_.p;
  if (p.common_hold_s <= 0 || self.onset == State::kNoOnset || s.t - self.onset > p.common_hold_s) return false;
  for (const auto& o : s.nodes) {
    if (o.name == n.name || !o.alive) continue;
    auto it = st_.find(o.name);
    if (it == st_.end()) continue;
    const State& os = it->second;
    if (os.phase != Phase::Normal || !os.sick || os.onset == State::kNoOnset) continue;
    if (std::abs(os.onset - self.onset) > p.common_window_s) continue;
    if (!linked(n.name, o.name, s)) return true;
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
  if (common_mode(n, st, s)) return Fault::CommonMode;
  if (stale) return missing ? Fault::EndpointLoss : Fault::Hang;
  return Fault::Degraded;
}

Fault Policy::baseline_fault(const NodeObs& n, State&, double) {
  const auto& p = spec_.p;
  switch (kind_) {
    case PolicyKind::None:
    case PolicyKind::Helix:
    case PolicyKind::Agent:
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
  } else if (kind_ == PolicyKind::Agent) {
    decide_agent(n, f, st, t, d, used);
    if (d.action == Action::Fallback || d.action == Action::Report) {
      if (d.action == Action::Fallback && ns.fallback.empty() && !ns.critical) d.action = Action::Report;
    } else {
      // Same restart pacing as Helix; ParamRestore and LifecycleReset ignore the delay.
      const bool young = !n.alive && n.quick_deaths > 0;
      if (p.fast_repair > 0) d.delay_s = (used > 0 && young && ns.fallback.empty()) ? backoff(used) : 0.0;
      else d.delay_s = (n.alive && used == 0) ? 0.0 : backoff(used);
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
      // Back off only after a young death on a node with no backup. A process that ran past
      // loop_s, or one that is alive and judged sick, says nothing about the next start; and
      // with a backup the crash-loop detector ends a fast loop within loop_deaths starts, so
      // waiting would only lengthen the outage.
      const bool young = !n.alive && n.quick_deaths > 0;
      if (p.fast_repair > 0) d.delay_s = (used > 0 && young && ns.fallback.empty()) ? backoff(used) : 0.0;
      else d.delay_s = (n.alive && used == 0) ? 0.0 : backoff(used);
    }
  }

  if (d.action == Action::Restart || d.action == Action::LifecycleReset || !d.candidates.empty()) st.attempts.push_back(t);
  st.last_fault = f;
  st.last_action = d.action;
  st.phase = d.action == Action::Report ? Phase::Retired : Phase::Acting;
  if (!d.candidates.empty()) {
    // The healer ranks the candidates and logs the "act" with the action it actually runs.
    std::string c;
    for (Action a : d.candidates) c += (c.empty() ? "" : ",") + std::string(to_string(a));
    d.detail = c;
    out.events.push_back({t, n.name, "incident", f, Action::None, d.incident_action, c});
  } else {
    out.events.push_back({t, n.name, d.action == Action::Fallback || d.action == Action::Report ? "given_up" : "act", f,
                          d.action, d.attempt, d.detail});
  }
  return d;
}

void Policy::decide_agent(const NodeObs& n, Fault f, State& st, double t, Decision& d, int used) {
  const auto& p = spec_.p;
  const NodeSpec& ns = *spec_.find(n.name);
  if (!st.incident_open || t - st.incident_t > p.window_s) {
    st.incident_open = true;
    st.incident_actions = 0;
    st.tried.clear();
  }
  auto give_up = [&](const std::string& why) {
    d.action = (!ns.fallback.empty() || ns.critical) ? Action::Fallback : Action::Report;
    d.detail = why;
    st.incident_open = false;
  };
  auto tried = [&](Action a) { return std::find(st.tried.begin(), st.tried.end(), a) != st.tried.end(); };
  const auto& ev = n.ev;
  if (txn_) {
    // TXN baseline: a transactional supervisor that never reads the fault class. Undo
    // uncommitted live parameter changes, roll back a config the failed process ran before it was
    // committed, else restart; if restarts do not take and the disk config is uncommitted, roll
    // it back. No lifecycle reset, no crash-loop or QoS special cases.
    std::vector<Action> c;
    const bool lkg = ns.restart && ev.has_lkg_config && ev.config_changed && !ev.config_pinned && !tried(Action::RestartLKG);
    if (n.alive && ev.param_diff > 0 && !tried(Action::ParamRestore)) c.push_back(Action::ParamRestore);
    st.cfg_strikes = ev.config_ran ? st.cfg_strikes + 1 : 0;
    const bool revert_first = ev.config_ran && st.cfg_strikes >= txn_strikes_;
    if (lkg && revert_first) c.push_back(Action::RestartLKG);
    if (ns.restart) c.push_back(Action::Restart);
    if (lkg && !revert_first) c.push_back(Action::RestartLKG);
    if (st.incident_actions >= kAgentK) return give_up("agent_budget");
    if (used >= p.max_attempts) return give_up("budget");
    if (c.empty()) return give_up(!ns.restart ? "restart=0" : "no_candidates");
    d.candidates = std::move(c);
    d.action = d.candidates.front();
    d.incident_action = ++st.incident_actions;
    st.incident_t = t;
    return;
  }
  if (f == Fault::QosMismatch) return give_up("qos_mismatch: no repair action changes a QoS contract");
  // Feasible repairs in rule order: undo a live parameter change first, then roll back a config
  // the running process actually loaded, then the v3 order (lifecycle reset first for a degraded or
  // endpoint-loss lifecycle node, else restart). A config changed on disk but not yet loaded by a
  // healthy run is tried as deployed (plain restart) before it is rolled back. Restart may repeat
  // within the incident; the other actions run at most once.
  std::vector<Action> c;
  if (n.alive && ev.param_diff > 0 && !tried(Action::ParamRestore)) c.push_back(Action::ParamRestore);
  const bool lkg = ns.restart && ev.has_lkg_config && ev.config_changed && !ev.config_pinned && !tried(Action::RestartLKG);
  const bool lc = ns.lifecycle && n.alive && !tried(Action::LifecycleReset);
  const bool lc_first = lc && (f == Fault::Degraded || f == Fault::EndpointLoss);
  if (lkg && ev.config_ran) c.push_back(Action::RestartLKG);
  if (lc_first) c.push_back(Action::LifecycleReset);
  if (ns.restart && f != Fault::CrashLoop) c.push_back(Action::Restart);
  if (lc && !lc_first) c.push_back(Action::LifecycleReset);
  if (lkg && !ev.config_ran) c.push_back(Action::RestartLKG);

  if (st.incident_actions >= kAgentK) return give_up("agent_budget");
  if (used >= p.max_attempts) return give_up("budget");
  if (c.empty()) return give_up(!ns.restart ? "restart=0" : f == Fault::CrashLoop ? "crash_loop" : "no_candidates");
  d.candidates = std::move(c);
  d.action = d.candidates.front();
  d.incident_action = ++st.incident_actions;
  st.incident_t = t;
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
      // Verified means the repaired node has published on every judged output since the repair,
      // not merely that it is alive: a respawned process that never gets to publish is not fixed.
      bool symptom = output_symptom(n, nullptr, nullptr, nullptr);
      bool fresh = true;
      for (const auto& o : n.outputs)
        if (o.expected_hz > 0 && t - o.age_s <= st.verify_from) fresh = false;
      if (n.alive && !symptom && fresh) {
        st.phase = Phase::Normal;
        out.events.push_back({t, n.name, "verified", st.last_fault, st.last_action, 0, ""});
        st.incident_open = false;
        continue;
      }
      // Still silent after the startup grace: the repair did not take, so judge the node again now
      // instead of waiting out the whole verification window.
      const bool failed_early = n.alive && symptom && t - st.verify_from >= p.startup_grace_s;
      if (n.alive && t < st.verify_deadline && !failed_early) continue;
      st.phase = Phase::Normal;
      out.events.push_back({t, n.name, "verify_failed", st.last_fault, st.last_action, 0, !n.alive ? "died" : failed_early ? "still_silent" : "timeout"});
      // The agent picks its next repair itself instead of escalating to a restart.
      if (kind_ != PolicyKind::Agent && n.alive && st.last_action == Action::LifecycleReset) st.escalate = true;
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

    const Fault f = diagnoses() ? diagnose(n, s) : baseline_fault(n, st, t);
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
    const double confirm = (diagnoses() && symptom && st.sharp) ? p.adaptive_confirm_s : p.confirm_s;
    if (!immediate && t - st.pending_since < confirm) continue;
    if (f == Fault::Upstream || f == Fault::CommonMode) {
      if (!st.victim_reported) {
        st.victim_reported = true;
        if (f == Fault::Upstream)
          out.events.push_back({t, n.name, "victim", f, Action::None, 0, "symptom explained by an upstream or resource root"});
        else
          out.events.push_back({t, n.name, "hold", f, Action::None, 0, "symptom began together with one on an unrelated node"});
      }
      continue;
    }
    st.pending = Fault::None;
    out.decisions.push_back(decide(n, f, st, t, out));
  }
  return out;
}

void Policy::forget(const std::string& node) {
  auto it = st_.find(node);
  if (it != st_.end()) it->second = State{};
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
      b.verify_from = t;
      b.last_fault = st.last_fault;
      b.last_action = Action::Fallback;
    }
    const bool has_backup = ns && !ns->fallback.empty();
    std::string detail = has_backup ? "-> " + ns->fallback : "no backup: SAFE_STOP";
    if (a == Action::Report) detail = "reported: SAFE_STOP";
    pending_events_.push_back({t, node, ok ? "fallback" : "fallback_failed", st.last_fault, a, 0, detail});
    return;
  }
  if (kind_ == PolicyKind::Agent && a != Action::None) {
    st.last_action = a;  // the ranked choice, which may differ from the rule default
    st.tried.push_back(a);
  }
  if (ok) {
    st.phase = Phase::Verifying;
    st.verify_deadline = t + p.verify_s;
    st.verify_from = t;
  } else {
    st.phase = Phase::Normal;
    pending_events_.push_back({t, node, "action_failed", st.last_fault, a, 0, ""});
    if (a == Action::LifecycleReset && kind_ != PolicyKind::Agent) st.escalate = true;
  }
}

}  // namespace helix_healer
