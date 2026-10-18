// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.

#include "helix_sensing_cpp/log_matcher.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace helix_sensing_cpp
{

std::string utf8_prefix(const std::string & s, std::size_t n)
{
  std::size_t i = 0;
  std::size_t count = 0;
  while (i < s.size() && count < n) {
    const auto c = static_cast<unsigned char>(s[i]);
    std::size_t len = 1;
    if (c >= 0xF0) {
      len = 4;
    } else if (c >= 0xE0) {
      len = 3;
    } else if (c >= 0xC0) {
      len = 2;
    }
    i = std::min(s.size(), i + len);
    ++count;
  }
  return s.substr(0, i);
}

std::vector<std::string> LogMatcher::load(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  std::vector<std::string> warnings;
  std::vector<LogRule> rules;
  const YAML::Node list = root["rules"];
  if (!list) {
    rules_.clear();
    return warnings;
  }
  for (const auto & r : list) {
    static const char * kRequired[] = {"id", "pattern", "fault_type", "severity",
      "detail_template"};
    std::string missing;
    for (const char * k : kRequired) {
      if (!r[k]) {
        missing += missing.empty() ? k : std::string(", ") + k;
      }
    }
    if (!missing.empty()) {
      warnings.push_back(
        "Rule '" + (r["id"] ? r["id"].as<std::string>() : std::string("?")) +
        "' missing fields {" + missing + "}, skipping");
      continue;
    }
    LogRule rule;
    rule.id = r["id"].as<std::string>();
    rule.pattern = r["pattern"].as<std::string>();
    rule.re = std::regex(
      rule.pattern, std::regex::ECMAScript | std::regex::icase | std::regex::optimize);
    rule.fault_type = r["fault_type"].as<std::string>();
    rule.severity = r["severity"].as<std::int64_t>();
    rule.detail_template = r["detail_template"].as<std::string>();
    rules.push_back(std::move(rule));
  }
  rules_ = std::move(rules);
  return warnings;
}

std::vector<FaultRecord> LogMatcher::process(
  std::uint8_t level, const std::string & name, const std::string & msg,
  const std::string & file, double wall_now)
{
  std::vector<FaultRecord> out;
  if (level < kMinLogSeverity) {
    return out;
  }
  for (const auto & rule : rules_) {
    if (!std::regex_search(msg, rule.re)) {
      continue;
    }
    const auto key = std::make_pair(rule.id, name);
    const auto it = dedup_.find(key);
    const double last = it == dedup_.end() ? 0.0 : it->second;
    if (wall_now - last < dedup_window_sec_) {
      continue;
    }
    dedup_[key] = wall_now;
    FaultRecord f;
    f.node_name = name;
    f.fault_type = rule.fault_type;
    f.severity = static_cast<std::int8_t>(rule.severity);
    f.detail = rule.detail_template + " | Log: " + utf8_prefix(msg, kLogTruncateCodePoints);
    f.timestamp = wall_now;
    f.context_keys = {"matched_rule_id", "log_severity", "log_name", "log_file"};
    f.context_values = {rule.id, std::to_string(level), name, file};
    out.push_back(std::move(f));
  }
  return out;
}

}  // namespace helix_sensing_cpp
