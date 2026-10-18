// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// LogMatcher: ROS-free core of the native log parser, ported from
// helix_core/log_parser.py with the same contract:
//
//   * rules come from a YAML file: rules: [{id, pattern, fault_type,
//     severity, detail_template}]; a rule missing a field is skipped
//   * patterns are case-insensitive regex searches (std::regex ECMAScript;
//     the shipped rules only use literals, '.*' and '|', on which Python re
//     and ECMAScript agree)
//   * only log level >= 40 (ERROR) is considered
//   * every matching rule fires, deduplicated per (rule id, logger name)
//     within dedup_window_sec on the wall clock
//   * detail = "<detail_template> | Log: <first 120 code points of msg>"
//   * context: matched_rule_id, log_severity, log_name, log_file
#ifndef HELIX_SENSING_CPP__LOG_MATCHER_HPP_
#define HELIX_SENSING_CPP__LOG_MATCHER_HPP_

#include <cstdint>
#include <map>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#include "helix_sensing_cpp/anomaly_core.hpp"

namespace helix_sensing_cpp
{

constexpr std::uint8_t kMinLogSeverity = 40;   // ERROR
constexpr std::size_t kLogTruncateCodePoints = 120;

struct LogRule
{
  std::string id;
  std::string pattern;
  std::regex re;
  std::string fault_type;
  std::int64_t severity{0};
  std::string detail_template;
};

/// First @p n Unicode code points of a UTF-8 string (Python str[:n]).
std::string utf8_prefix(const std::string & s, std::size_t n);

class LogMatcher
{
public:
  explicit LogMatcher(double dedup_window_sec = 5.0)
  : dedup_window_sec_(dedup_window_sec) {}

  /// Load rules from @p path. Throws on an unreadable file or bad YAML or
  /// regex; returns one warning per skipped rule.
  std::vector<std::string> load(const std::string & path);
  void set_rules(std::vector<LogRule> rules) {rules_ = std::move(rules);}
  std::size_t size() const noexcept {return rules_.size();}
  void clear() {rules_.clear(); dedup_.clear();}

  /// Faults for one /rosout record, at wall time @p wall_now.
  std::vector<FaultRecord> process(
    std::uint8_t level, const std::string & name, const std::string & msg,
    const std::string & file, double wall_now);

private:
  double dedup_window_sec_;
  std::vector<LogRule> rules_;
  std::map<std::pair<std::string, std::string>, double> dedup_;
};

}  // namespace helix_sensing_cpp

#endif  // HELIX_SENSING_CPP__LOG_MATCHER_HPP_
