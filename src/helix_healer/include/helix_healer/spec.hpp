// Plain-text healer spec. One directive per line, '#' starts a comment.
//
//   healer  key=value ...                 global parameters (see HealerParams)
//   node    NAME key=value ...            a managed process; NAME is its ROS node name
//   out     NAME TOPIC [HZ]               an output the node must keep alive; HZ 0 or absent = learn it
//   in      NAME TOPIC                    optional declared input (the ROS graph adds the rest)
//   exec    NAME ARGV...                  command line; "@pkg/exe" resolves to <prefix>/lib/pkg/exe
//
// node keys: restart=1 critical=1 lifecycle=0 autostart=1 fallback=OTHER cpu_max=CORES rss_max_mb=MB
#pragma once

#include <string>
#include <vector>

namespace helix_healer {

struct HealerParams {
  double tick_hz = 100.0;        // v3 used 20
  double stale_periods = 4.0;     // an output is stale after this many expected periods...
  double stale_min_s = 0.25;      // ...but never sooner than this
  double degraded_ratio = 0.5;    // measured rate below ratio * expected = degraded
  double rate_window_s = 1.0;     // rate is counted over max(window, 3 periods)
  double confirm_s = 0.3;         // a non-exit symptom must persist this long before acting
  double verify_s = 5.0;          // a repaired node must be healthy within this long
  double startup_grace_s = 5.0;   // no output judgement this long after a spawn
  double backoff_s = 0.5;         // restart delay, doubled per attempt
  double backoff_max_s = 4.0;
  double window_s = 60.0;         // repair budget window
  int max_attempts = 3;           // repairs per node per window before the fallback
  double loop_s = 2.0;            // an exit within loop_s of its spawn is a quick death
  int loop_deaths = 2;            // consecutive quick deaths that make a crash loop
  double cpu_sustain_s = 1.5;     // CPU above cpu_max for this long = runaway
  double leak_horizon_s = 15.0;   // projected time to rss_max below this = leak
  double leak_min_growth_mb = 64; // ...and at least this much growth over the post-start baseline
  double victim_grace_s = 2.0;    // after upstream recovers, a downstream node gets this long to follow
  // Common-mode hold (helix only): a symptom that starts within common_window_s of a symptom on
  // another alive node with no data-flow path to or from it points at a shared cause (host or
  // middleware stall), not at either node. Both are held for up to common_hold_s from onset, then
  // judged individually. common_hold_s=0 disables the rule.
  double common_window_s = 0.5;
  double common_hold_s = 2.0;
  double lifecycle_timeout_s = 1.0;
  double term_grace_s = 1.0;      // SIGTERM -> SIGKILL
  double learn_s = 3.0;           // HZ 0 outputs: expected rate = rate measured over this window
  double resource_period_s = 0.5; // /proc sampling period
  // Fast repair (applies to every policy): SIGKILL at once for a hung or runaway process (SIGTERM
  // cannot be served by a blocked executor), no delay before the first restart, and post-repair
  // rates measured only over the new process's messages. fast_repair=0 restores v3 timing.
  double fast_repair = 1;
  // A process writing a core dump has already crashed, but the kernel keeps it alive until the dump
  // ends, and a piped core_pattern helper (apport, systemd-coredump) can take hundreds of ms. Every
  // policy sees the crash as soon as /proc reports CoreDumping, and after core_wait_s of dumping the
  // healer SIGKILLs the process so the exit is immediate. That truncates the core file: raise
  // core_wait_s to keep complete cores at the cost of repair latency.
  double core_wait_s = 0;
  // Adaptive timing envelope (helix policy only; the watchdog keeps the fixed stale timeout).
  // Per (node, output) it learns the inter-message gap online while the node is healthy, then
  // judges staleness against max(adaptive_floor_s, adaptive_stale_mult * decaying max gap) and
  // rate loss with a schedule-deficit CUSUM. adaptive=0 restores the v3 rule.
  double adaptive = 1;
  double adaptive_floor_s = 0.1;
  double adaptive_stale_mult = 3.0;
  double adaptive_min_gaps = 50;     // gaps observed before the envelope is trusted...
  double adaptive_min_s = 2.0;       // ...and seconds of healthy observation
  double adaptive_tau_s = 60.0;      // decay time of the learned maxima
  double adaptive_cusum_h_min = 4.0; // deficit alarm never below this many missed periods
  double adaptive_cusum_mult = 2.0;  // ...and never below this times the largest healthy deficit
  double adaptive_confirm_s = 0.1;   // confirmation for symptom faults when the envelope is ready
};

struct OutputSpec {
  std::string topic;
  double hz = 0.0;
};

struct NodeSpec {
  std::string name;
  std::vector<std::string> argv;
  std::vector<OutputSpec> outputs;
  std::vector<std::string> inputs;
  // Helpers run (not supervised) after every spawn of this node, e.g. ros2_control spawners.
  std::vector<std::vector<std::string>> then;
  // attach=PATTERN: adopt an already-running process whose cmdline contains PATTERN instead of
  // launching it; on repair it is re-executed with the captured argv, env and cwd.
  std::string attach;
  bool restart = true;
  bool critical = true;
  bool lifecycle = false;
  bool autostart = true;
  std::string fallback;
  double cpu_max = 0.9;     // cores
  double rss_max_mb = 512;  // MB
};

struct Spec {
  HealerParams p;
  std::vector<NodeSpec> nodes;
  const NodeSpec* find(const std::string& name) const;
};

// Throws std::runtime_error naming the offending line.
Spec parse_spec(const std::string& text);
Spec load_spec(const std::string& path);

}  // namespace helix_healer
