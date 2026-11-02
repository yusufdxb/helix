// Diagnosis and repair policy. Pure C++, no ROS: the node feeds it observation
// snapshots and executes the decisions it returns. A policy can only choose from
// the closed Action enum below; it never runs commands.
#pragma once

#include <deque>
#include <map>
#include <string>
#include <vector>

#include "helix_healer/spec.hpp"

namespace helix_healer {

enum class Fault { None, Crash, CrashLoop, Hang, EndpointLoss, CpuRunaway, MemoryLeak, Degraded, QosMismatch, Upstream, CommonMode };
// ParamRestore and RestartLKG are agent-only repairs. ParamRestore sets only the parameters that
// differ from the node's last-known-good snapshot; RestartLKG restarts it with the last-known-good
// copies of its config files (the user's files are never written).
enum class Action { None, Restart, LifecycleReset, Fallback, Report, ParamRestore, RestartLKG };
// Agent: Helix detection, diagnosis and verification, plus a per-incident repair search over
// ranked candidate actions (the ranking itself is done by the healer, see agent.hpp).
enum class PolicyKind { None, Respawn, Watchdog, Helix, Agent };

const char* to_string(Fault f);
const char* to_string(Action a);
Action action_from(const std::string& name);  // Action::None when unknown
PolicyKind policy_kind(const std::string& name);  // throws on unknown

struct TopicObs {
  std::string topic;
  double expected_hz = 0;  // 0 = not learned yet, not judged
  double age_s = 0;        // since the last message (or since spawn if none)
  double rate_hz = 0;
  bool own_endpoint = true;  // the node still advertises this topic in the graph
  // Adaptive envelope (0 = not ready, the fixed rule applies). Used by the helix policy only.
  double stale_limit_s = 0;
  double deficit = 0;
  double deficit_limit = 0;
};

struct NodeObs {
  std::string name;
  bool alive = false;
  double up_s = 0;    // since the current process was spawned
  double down_s = 0;  // since it died (0 while alive)
  double cpu = 0;     // cores
  double rss_mb = 0;
  double rss_slope = 0;  // MB/s over the recent window
  double rss_base = 0;   // MB shortly after start
  int quick_deaths = 0;  // consecutive exits within loop_s of spawn
  bool qos_incompatible = false;  // one of its subscriptions cannot match a publisher
  std::vector<std::string> upstream;  // managed nodes publishing topics it subscribes to
  std::vector<TopicObs> outputs;
  // Structured evidence for the agent policy, filled by the healer. Other policies ignore it.
  struct Evidence {
    int param_diff = 0;           // live parameters differing from the LKG snapshot
    bool config_changed = false;  // a config file on disk differs from its LKG copy
    bool config_ran = false;      // the current (or last) process was started with non-LKG config
    bool config_pinned = false;   // already running on the LKG copies after a rollback
    bool has_lkg_config = false;  // LKG copies exist, so RestartLKG is possible
  } ev;
};

struct Snapshot {
  double t = 0;
  std::vector<NodeObs> nodes;  // only running or once-started managed nodes
};

struct Decision {
  std::string node;
  Fault fault = Fault::None;
  Action action = Action::None;
  int attempt = 0;
  double delay_s = 0;
  std::string detail;
  // Agent policy: feasible, untried repairs in rule order. When non-empty, the healer ranks them
  // and executes the first; `action` holds the rule choice only as a default.
  std::vector<Action> candidates;
  int incident_action = 0;  // 1-based index of this action within its incident
};

struct Event {
  double t = 0;
  std::string node;
  std::string kind;  // detect, act, verified, verify_failed, escalate, fallback, given_up, victim, report
  Fault fault = Fault::None;
  Action action = Action::None;
  int attempt = 0;
  std::string detail;
};

struct StepOut {
  std::vector<Decision> decisions;
  std::vector<Event> events;
};

class Policy {
 public:
  Policy(PolicyKind kind, const Spec& spec);
  // Call at tick rate. Every returned decision must be answered with on_result().
  StepOut step(const Snapshot& s);
  // ok=false for a failed lifecycle reset escalates to a restart on the next step.
  void on_result(const std::string& node, Action a, bool ok, double t);
  PolicyKind kind() const { return kind_; }
  // Observable state for /diagnostics.
  std::string phase(const std::string& node) const;
  // True while the node is in normal operation with no pending or recent fault. The healer
  // learns timing envelopes only while this holds.
  bool healthy(const std::string& node) const;
  // The node left supervision (attach mode: its launcher exited). Drops its fault state so a
  // later re-adoption starts fresh. Call only when no decision for it is outstanding.
  void forget(const std::string& node);

  // Agent policy: per-incident repair limit (K in the pre-registration).
  static constexpr int kAgentK = 3;

  // Exposed for tests: the HELIX root-cause rule on one node, given the snapshot.
  Fault diagnose(const NodeObs& n, const Snapshot& s);

 private:
  enum class Phase { Normal, Acting, Verifying, Retired };
  struct State {
    Phase phase = Phase::Normal;
    Fault pending = Fault::None;
    double pending_since = -1;
    double cpu_high_since = -1;
    double resource_until = -1;  // this node counts as a resource root until then
    static constexpr double kNoOnset = -1e300;
    double onset = kNoOnset;     // when the current symptom started (may be negative early on)
    bool replaced = false;       // retired and a fallback took over successfully
    double healthy_since = 0;
    bool sick = false;
    bool sharp = false;          // last symptom came from a ready adaptive envelope
    bool victim_reported = false;
    std::deque<double> attempts;
    Fault last_fault = Fault::None;
    Action last_action = Action::None;
    double verify_deadline = 0;
    double verify_from = 0;      // a repair counts only once every judged output has published after this
    bool escalate = false;
    // Agent incident: one root-cause episode on this node, closed by a verified repair.
    bool incident_open = false;
    double incident_t = -1;      // time of its last action
    int incident_actions = 0;
    std::vector<Action> tried;   // actions already run in this incident
  };

  void observe(const NodeObs& n, const Snapshot& s, State& st);
  bool upstream_sick(const NodeObs& n, const State& self, double t) const;
  bool resource_root_elsewhere(const std::string& self, double t) const;
  bool input_silent_longer(const NodeObs& n, const Snapshot& s) const;
  bool linked(const std::string& a, const std::string& b, const Snapshot& s) const;
  bool common_mode(const NodeObs& n, const State& self, const Snapshot& s) const;
  // sharp: the symptom was flagged by a ready adaptive envelope (short confirmation applies).
  bool output_symptom(const NodeObs& n, bool* stale, bool* own_missing, double* oldest, bool* sharp = nullptr) const;
  bool leaking(const NodeObs& n) const;
  Decision decide(const NodeObs& n, Fault f, State& st, double t, StepOut& out);
  void decide_agent(const NodeObs& n, Fault f, State& st, double t, Decision& d, int used);
  bool diagnoses() const { return kind_ == PolicyKind::Helix || kind_ == PolicyKind::Agent; }
  Fault baseline_fault(const NodeObs& n, State& st, double t);
  int budget_used(State& st, double t);

  PolicyKind kind_;
  Spec spec_;
  std::map<std::string, State> st_;
  std::vector<Event> pending_events_;
};

}  // namespace helix_healer
