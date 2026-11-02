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
  double tick_hz = 20.0;
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
  double lifecycle_timeout_s = 1.0;
  double term_grace_s = 1.0;      // SIGTERM -> SIGKILL
  double learn_s = 3.0;           // HZ 0 outputs: expected rate = rate measured over this window
  double resource_period_s = 0.5; // /proc sampling period
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
