// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// See include/helix_arbiter_cpp/arbiter_config.hpp.

#include "helix_arbiter_cpp/arbiter_config.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace helix_arbiter_cpp
{

namespace
{

constexpr const char * kSourcesPrefix = "sources.";

const std::set<std::string> & known_top_level()
{
  static const std::set<std::string> names{
    "output_topic", "status_topic", "hold_topic", "rate_hz", "hold_timeout_sec",
    "max_abs_linear", "max_abs_angular", "shutdown_zero_count", "autostart",
    "use_sim_time", "arbiter_backend"};
  return names;
}

bool starts_with(const std::string & s, const std::string & prefix)
{
  return s.size() >= prefix.size() && s.compare(0, prefix.size(), prefix) == 0;
}

const ParamValue & require(const ParamMap & params, const std::string & name)
{
  auto it = params.find(name);
  if (it == params.end()) {
    throw ConfigError("missing parameter '" + name + "'");
  }
  return it->second;
}

std::string as_string(const ParamValue & v, const std::string & name)
{
  if (const auto * s = std::get_if<std::string>(&v)) {
    if (s->empty()) {
      throw ConfigError("'" + name + "' must be a non-empty string");
    }
    return *s;
  }
  throw ConfigError("'" + name + "' must be a string");
}

std::int64_t as_integer(const ParamValue & v, const std::string & name)
{
  if (const auto * i = std::get_if<std::int64_t>(&v)) {
    return *i;
  }
  throw ConfigError("'" + name + "' must be an integer");
}

// Integers are accepted where a real number is expected; booleans are not.
double as_number(const ParamValue & v, const std::string & name)
{
  if (const auto * d = std::get_if<double>(&v)) {
    return *d;
  }
  if (const auto * i = std::get_if<std::int64_t>(&v)) {
    return static_cast<double>(*i);
  }
  throw ConfigError("'" + name + "' must be a number");
}

bool as_bool(const ParamValue & v, const std::string & name)
{
  if (const auto * b = std::get_if<bool>(&v)) {
    return *b;
  }
  throw ConfigError("'" + name + "' must be a boolean");
}

}  // namespace

std::vector<std::pair<std::string, ParamValue>> default_parameters()
{
  const ArbiterConfig d;
  return {
    {"output_topic", d.output_topic},
    {"status_topic", d.status_topic},
    {"hold_topic", d.hold_topic},
    {"rate_hz", d.rate_hz},
    {"hold_timeout_sec", d.hold_timeout_sec},
    {"max_abs_linear", d.limits.max_abs_linear},
    {"max_abs_angular", d.limits.max_abs_angular},
    {"shutdown_zero_count", d.shutdown_zero_count},
    {"autostart", d.autostart},
  };
}

std::vector<SourceSpec> parse_sources(const ParamMap & params)
{
  // name -> field -> value, ordered by name like the Python node's sorted().
  std::map<std::string, std::map<std::string, const ParamValue *>> raw;
  const std::string prefix(kSourcesPrefix);
  for (const auto & [key, value] : params) {
    if (!starts_with(key, prefix)) {
      continue;
    }
    const std::string rest = key.substr(prefix.size());
    const auto dot = rest.find('.');
    const std::string name = rest.substr(0, dot);
    const std::string field = dot == std::string::npos ? std::string() : rest.substr(dot + 1);
    if (name.empty()) {
      throw ConfigError("source parameter '" + key + "' has an empty source name");
    }
    raw[name][field] = &value;
  }
  if (raw.empty()) {
    throw ConfigError("no sources configured (expected sources.<name>.topic/priority/timeout)");
  }
  std::vector<SourceSpec> specs;
  for (const auto & [name, fields] : raw) {
    for (const auto & [field, value] : fields) {
      (void)value;
      if (field != "topic" && field != "priority" && field != "timeout") {
        throw ConfigError(
                "source '" + name + "' has unsupported field '" + field +
                "' (allowed: topic, priority, timeout)");
      }
    }
    for (const char * f : {"topic", "priority", "timeout"}) {
      if (fields.find(f) == fields.end()) {
        throw ConfigError("source '" + name + "' is missing '" + f + "'");
      }
    }
    const std::string base = std::string(kSourcesPrefix) + name + ".";
    SourceSpec s;
    s.name = name;
    s.topic = as_string(*fields.at("topic"), base + "topic");
    s.priority = as_integer(*fields.at("priority"), base + "priority");
    s.timeout_sec = as_number(*fields.at("timeout"), base + "timeout");
    if (!std::isfinite(s.timeout_sec) || s.timeout_sec <= 0.0) {
      throw ConfigError("'" + base + "timeout' must be finite and > 0");
    }
    specs.push_back(std::move(s));
  }
  return specs;
}

ArbiterConfig parse_config(const ParamMap & params)
{
  ArbiterConfig c;
  c.output_topic = as_string(require(params, "output_topic"), "output_topic");
  c.status_topic = as_string(require(params, "status_topic"), "status_topic");
  c.hold_topic = as_string(require(params, "hold_topic"), "hold_topic");

  c.rate_hz = as_number(require(params, "rate_hz"), "rate_hz");
  if (!std::isfinite(c.rate_hz) || c.rate_hz <= 0.0 || c.rate_hz > kMaxRateHz) {
    throw ConfigError("'rate_hz' must be finite, > 0 and <= 1000");
  }
  c.hold_timeout_sec = as_number(require(params, "hold_timeout_sec"), "hold_timeout_sec");
  if (!std::isfinite(c.hold_timeout_sec) || c.hold_timeout_sec <= 0.0) {
    throw ConfigError("'hold_timeout_sec' must be finite and > 0");
  }
  c.limits.max_abs_linear = as_number(require(params, "max_abs_linear"), "max_abs_linear");
  c.limits.max_abs_angular = as_number(require(params, "max_abs_angular"), "max_abs_angular");
  for (const auto & [name, v] : {std::pair<const char *, double>{"max_abs_linear",
        c.limits.max_abs_linear}, {"max_abs_angular", c.limits.max_abs_angular}})
  {
    if (!std::isfinite(v) || v < 0.0) {
      throw ConfigError(std::string("'") + name + "' must be finite and >= 0");
    }
  }
  c.shutdown_zero_count = as_integer(
    require(params, "shutdown_zero_count"),
    "shutdown_zero_count");
  if (c.shutdown_zero_count < 1 || c.shutdown_zero_count > kMaxShutdownZeroCount) {
    throw ConfigError("'shutdown_zero_count' must be between 1 and 1000");
  }
  c.autostart = as_bool(require(params, "autostart"), "autostart");
  c.sources = parse_sources(params);
  return c;
}

std::vector<std::string> unknown_parameters(const ParamMap & params)
{
  std::vector<std::string> out;
  for (const auto & [key, value] : params) {
    (void)value;
    if (known_top_level().count(key) != 0 || starts_with(key, kSourcesPrefix) ||
      starts_with(key, "qos_overrides."))
    {
      continue;
    }
    out.push_back(key);
  }
  return out;
}

void check_topic_layout(
  const std::string & output, const std::string & status, const std::string & hold,
  const std::vector<std::pair<std::string, std::string>> & source_topics)
{
  if (output == status || output == hold || status == hold) {
    throw ConfigError(
            "output_topic, status_topic and hold_topic must be distinct (resolved: " +
            output + ", " + status + ", " + hold + ")");
  }
  std::map<std::string, std::string> seen;
  for (const auto & [name, topic] : source_topics) {
    if (topic == output) {
      throw ConfigError("source " + name + " subscribes to the output topic " + output);
    }
    if (topic == status || topic == hold) {
      throw ConfigError("source " + name + " uses the arbiter's own topic " + topic);
    }
    auto [it, inserted] = seen.emplace(topic, name);
    if (!inserted) {
      throw ConfigError(
              "sources " + it->second + " and " + name + " share the topic " + topic);
    }
  }
}

}  // namespace helix_arbiter_cpp
