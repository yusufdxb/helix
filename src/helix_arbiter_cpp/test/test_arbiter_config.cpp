// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// The helix_arbiter parameter contract (no ROS). test_python_parity.py
// checks that the Python node accepts and refuses exactly the same maps.

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "helix_arbiter_cpp/arbiter_config.hpp"

using helix_arbiter_cpp::ConfigError;
using helix_arbiter_cpp::ParamMap;
using helix_arbiter_cpp::ParamValue;
using helix_arbiter_cpp::Unsupported;

namespace
{

// The shipped config/arbiter.yaml, as the node sees it after
// automatically_declare_parameters_from_overrides.
ParamMap shipped()
{
  return {
    {"output_topic", std::string("/cmd_vel")},
    {"status_topic", std::string("/helix/arbiter/status")},
    {"hold_topic", std::string("/helix/hold")},
    {"rate_hz", 50.0},
    {"hold_timeout_sec", 0.5},
    {"max_abs_linear", 1.0},
    {"max_abs_angular", 1.5},
    {"shutdown_zero_count", std::int64_t{10}},
    {"autostart", false},
    {"use_sim_time", false},
    {"arbiter_backend", std::string("cpp")},
    {"sources.teleop.topic", std::string("/teleop/cmd_vel")},
    {"sources.teleop.priority", std::int64_t{200}},
    {"sources.teleop.timeout", 0.5},
    {"sources.nav.topic", std::string("/nav/cmd_vel")},
    {"sources.nav.priority", std::int64_t{50}},
    {"sources.nav.timeout", 0.5},
  };
}

ParamMap with(ParamMap m, const std::string & key, ParamValue v)
{
  m[key] = std::move(v);
  return m;
}

ParamMap without(ParamMap m, const std::string & key)
{
  m.erase(key);
  return m;
}

}  // namespace

TEST(ArbiterConfig, ShippedConfigParses)
{
  const auto c = helix_arbiter_cpp::parse_config(shipped());
  EXPECT_EQ(c.output_topic, "/cmd_vel");
  EXPECT_EQ(c.rate_hz, 50.0);
  EXPECT_EQ(c.shutdown_zero_count, 10);
  ASSERT_EQ(c.sources.size(), 2u);
  EXPECT_EQ(c.sources[0].name, "nav");   // sorted by name
  EXPECT_EQ(c.sources[1].priority, 200);
  EXPECT_TRUE(helix_arbiter_cpp::unknown_parameters(shipped()).empty());
}

TEST(ArbiterConfig, DefaultsAreTheDocumentedOnes)
{
  ParamMap m;
  for (const auto & [name, value] : helix_arbiter_cpp::default_parameters()) {
    m[name] = value;
  }
  m["sources.nav.topic"] = std::string("/nav/cmd_vel");
  m["sources.nav.priority"] = std::int64_t{50};
  m["sources.nav.timeout"] = std::int64_t{1};   // YAML integer accepted for a real value
  const auto c = helix_arbiter_cpp::parse_config(m);
  EXPECT_EQ(c.status_topic, "/helix/arbiter/status");
  EXPECT_EQ(c.hold_topic, "/helix/hold");
  EXPECT_EQ(c.hold_timeout_sec, 0.5);
  EXPECT_EQ(c.limits.max_abs_linear, 1.0);
  EXPECT_EQ(c.limits.max_abs_angular, 1.5);
  EXPECT_FALSE(c.autostart);
  EXPECT_EQ(c.sources[0].timeout_sec, 1.0);
}

TEST(ArbiterConfig, TypesAreStrict)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::vector<std::pair<std::string, ParamValue>> bad{
    {"rate_hz", std::string("50")}, {"rate_hz", true}, {"rate_hz", Unsupported{}},
    {"rate_hz", 0.0}, {"rate_hz", 1000.5}, {"rate_hz", nan},
    {"hold_timeout_sec", 0.0}, {"hold_timeout_sec", nan},
    {"max_abs_linear", -0.1}, {"max_abs_linear", std::numeric_limits<double>::infinity()},
    {"max_abs_angular", nan}, {"shutdown_zero_count", 10.0},
    {"shutdown_zero_count", std::int64_t{0}}, {"shutdown_zero_count", std::int64_t{1001}},
    {"autostart", std::string("true")}, {"autostart", std::int64_t{1}},
    {"output_topic", std::string()}, {"output_topic", std::int64_t{5}},
    {"sources.nav.priority", 50.0}, {"sources.nav.priority", true},
    {"sources.nav.timeout", std::string("0.5")}, {"sources.nav.timeout", 0.0},
    {"sources.nav.timeout", nan}, {"sources.nav.topic", std::string()},
    {"sources.nav.enabled", false}, {"sources.nav", std::int64_t{5}},
  };
  for (const auto & [key, value] : bad) {
    EXPECT_THROW(helix_arbiter_cpp::parse_config(with(shipped(), key, value)), ConfigError)
      << key;
  }
  for (const char * key : {"output_topic", "rate_hz", "autostart", "sources.nav.timeout"}) {
    EXPECT_THROW(helix_arbiter_cpp::parse_config(without(shipped(), key)), ConfigError) << key;
  }
}

TEST(ArbiterConfig, UnknownTopLevelParametersAreReported)
{
  auto m = with(shipped(), "max_abs_linaer", 0.3);
  m["qos_overrides./cmd_vel.publisher.depth"] = std::int64_t{1};
  const auto unknown = helix_arbiter_cpp::unknown_parameters(m);
  ASSERT_EQ(unknown.size(), 1u);
  EXPECT_EQ(unknown[0], "max_abs_linaer");
}

TEST(ArbiterConfig, TopicLayoutRejectsAliasesAndSharing)
{
  using helix_arbiter_cpp::check_topic_layout;
  const std::string out = "/cmd_vel";
  const std::string st = "/helix/arbiter/status";
  const std::string hold = "/helix/hold";
  EXPECT_NO_THROW(check_topic_layout(out, st, hold, {{"nav", "/nav/cmd_vel"}}));
  EXPECT_THROW(check_topic_layout(out, st, hold, {{"nav", out}}), ConfigError);
  EXPECT_THROW(check_topic_layout(out, st, hold, {{"nav", hold}}), ConfigError);
  EXPECT_THROW(check_topic_layout(out, st, hold, {{"a", "/x"}, {"b", "/x"}}), ConfigError);
  EXPECT_THROW(check_topic_layout(out, out, hold, {{"a", "/x"}}), ConfigError);
}
