// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// ROS-free parameter contract of the helix_arbiter node, shared with the
// Python reference (arbiter_core.parse_config / check_topic_layout).
//
// Both nodes run with automatically_declare_parameters_from_overrides, so a
// YAML value keeps its YAML type. The node flattens every parameter into a
// ParamMap ("sources.teleop.topic" -> "/teleop/cmd_vel") and hands it here.
//
// Typing is strict on purpose. A priority of 200.5, a timeout given as the
// string "0.5", or a boolean where a number is expected is a configuration
// error, not something to coerce. Real-valued parameters accept YAML
// integers (rate_hz: 50) because YAML writers rarely add ".0".
#ifndef HELIX_ARBITER_CPP__ARBITER_CONFIG_HPP_
#define HELIX_ARBITER_CPP__ARBITER_CONFIG_HPP_

#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "helix_arbiter_cpp/arbiter_core.hpp"

namespace helix_arbiter_cpp
{

/// A parameter value whose type is not one of bool / integer / double /
/// string (arrays, byte arrays, NOT_SET).
struct Unsupported
{
};

using ParamValue = std::variant<Unsupported, bool, std::int64_t, double, std::string>;
using ParamMap = std::map<std::string, ParamValue>;

class ConfigError : public std::invalid_argument
{
public:
  using std::invalid_argument::invalid_argument;
};

/// Upper bound on rate_hz. The output is a 50 Hz command stream; a
/// kilohertz timer is a typo, not a configuration.
inline constexpr double kMaxRateHz = 1000.0;
/// Upper bound on shutdown_zero_count; the burst is published synchronously.
inline constexpr std::int64_t kMaxShutdownZeroCount = 1000;

struct ArbiterConfig
{
  std::string output_topic{"/cmd_vel"};
  std::string status_topic{"/helix/arbiter/status"};
  std::string hold_topic{"/helix/hold"};
  double rate_hz{50.0};
  double hold_timeout_sec{0.5};
  Limits limits{};
  std::int64_t shutdown_zero_count{10};
  bool autostart{false};
  std::vector<SourceSpec> sources;  // sorted by name
};

/// The node's declared defaults, as (name, value) pairs, in declaration order.
std::vector<std::pair<std::string, ParamValue>> default_parameters();

/// Parse and validate the full parameter set. Throws ConfigError. Does not
/// resolve topic names; see check_topic_layout.
ArbiterConfig parse_config(const ParamMap & params);

/// Parse the "sources.<name>.<field>" entries. Every source needs exactly
/// the fields topic (non-empty string), priority (integer) and timeout
/// (finite number > 0); any other field is an error so a misspelt or
/// unsupported key (e.g. "enabled: false") cannot be silently ignored.
std::vector<SourceSpec> parse_sources(const ParamMap & params);

/// Top-level parameter names this node does not use (typo guard; the node
/// logs them). sources.*, use_sim_time and qos_overrides.* are expected.
std::vector<std::string> unknown_parameters(const ParamMap & params);

/// Collision checks on RESOLVED (fully qualified) topic names: the output,
/// status and hold topics are pairwise distinct, and every source topic is
/// distinct from those three and from every other source. Throws ConfigError.
void check_topic_layout(
  const std::string & output, const std::string & status, const std::string & hold,
  const std::vector<std::pair<std::string, std::string>> & source_topics);

}  // namespace helix_arbiter_cpp

#endif  // HELIX_ARBITER_CPP__ARBITER_CONFIG_HPP_
