// Plain-text monitor configuration: one directive per line, `kind stream [stream2] key=value ...`.
// Lists are comma separated. '#' starts a comment.
//
//   rate     lidar_imu min_hz=200 max_hz=300 timeout_s=0.1
//   frozen   lidar_imu frozen_s=0.05
//   stamp    lidar_imu stall_s=0.05
//   range    lidar_imu ch=0,1,2 lo=-5,-5,-5 hi=5,5,5
//   residual low lidar_imu a_ch=0,1,2 b_ch=0,1,2 R=1,0,0,0,1,0,0,0,1 d=0,0,0 tau_s=0.1 threshold=0.1
//   group    joints low ch=0,1 scale=1,1 raw=1 max=0 detrend=0 quiet=0.1 moving=0.5
//   stop     window_s=0.5 deadline_s=2 min_groups=2 require_raw=1
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "helix_monitor/checks.hpp"
#include "helix_monitor/stop_verifier.hpp"

namespace helix_monitor {

struct Config {
  Monitor monitor;
  std::vector<EvidenceGroup> groups;
  StopVerifier::Params stop;
  bool has_stop = false;
};

// Throws std::runtime_error with the line number on malformed input.
std::unique_ptr<Config> parse_config(const std::string& text);
std::unique_ptr<Config> load_config(const std::string& path);

}  // namespace helix_monitor
