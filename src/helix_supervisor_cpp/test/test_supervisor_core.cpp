// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// SupervisorCore: one test per audited weakness of the Python
// diagnosis/recovery chain, then the acknowledgment and clearance paths.

#include <gtest/gtest.h>

#include <cstdint>

#include <string>
#include <vector>

#include "helix_supervisor_cpp/supervisor_core.hpp"

using helix_supervisor_cpp::AuditEvent;
using helix_supervisor_cpp::Config;
using helix_supervisor_cpp::SinkFeedback;
using helix_supervisor_cpp::State;
using helix_supervisor_cpp::SupervisorCore;

namespace
{

constexpr const char * kCloud = "rate_hz/utlidar_cloud";
constexpr const char * kDetector = "helix_anomaly_detector";
constexpr const char * kRates = "helix_topic_rate_monitor";

Config base_config()
{
  Config c;
  c.expected_nodes = {kDetector, kRates};
  c.critical_nodes = {kDetector, kRates};
  c.actuation_enabled = true;
  c.auto_resume = true;
  c.require_controller_ack = true;
  return c;
}

/// Beat every expected node from t0 to t1 at 10 Hz, ticking at 20 Hz.
void run(SupervisorCore & s, double t0, double t1, const std::vector<std::string> & beating)
{
  for (double t = t0; t <= t1 + 1e-9; t += 0.05) {
    if (static_cast<std::int64_t>(t * 20.0 + 0.5) % 2 == 0) {
      for (const auto & n : beating) {
        s.on_heartbeat(n, t);
      }
    }
    s.tick(t);
  }
}

SupervisorCore started(Config c = base_config())
{
  SupervisorCore s(c);
  s.start(0.0);
  run(s, 0.0, 0.5, {kDetector, kRates});
  return s;
}

bool has_action(
  const std::vector<AuditEvent> & a, const std::string & action,
  const std::string & status)
{
  for (const auto & e : a) {
    if (e.kind == "ACTION" && e.action == action && e.status == status) {
      return true;
    }
  }
  return false;
}

/// Drive a hold through arbiter ack, controller ack and measured stop.
void confirm_stop(SupervisorCore & s, double t)
{
  s.on_arbiter_status("HELIX_HOLD", s.hold_id(), t);
  SinkFeedback fb;
  fb.feedback_available = true;
  fb.stop_acked = true;
  fb.stop_ack_sent_at = t + 0.001;
  fb.stop_acked_at = t + 0.01;
  fb.stopped = true;
  fb.state_at = t + 0.02;
  s.on_sink_feedback(fb, t + 0.02);
}

}  // namespace

// --- startup -----------------------------------------------------------------

TEST(SupervisorStartup, HoldsUntilCriticalNodesBeat)
{
  SupervisorCore s(base_config());
  s.start(0.0);
  EXPECT_EQ(s.state(), State::kStarting);
  EXPECT_TRUE(s.hold());
  run(s, 0.0, 0.2, {kDetector});            // one critical node still silent
  EXPECT_EQ(s.state(), State::kStarting);
  EXPECT_TRUE(s.hold());
  run(s, 0.25, 0.5, {kDetector, kRates});
  EXPECT_EQ(s.state(), State::kNominal);
  EXPECT_FALSE(s.hold());
}

// Weakness: a node that never starts was invisible (registry learned nodes
// only from their first heartbeat).
TEST(SupervisorStartup, NeverStartedNodeRaisesStopWorthyCrash)
{
  SupervisorCore s(base_config());
  s.start(0.0);
  run(s, 0.0, 5.5, {kDetector});            // rate monitor never beats
  const auto * f = s.fault("CRASH:helix_topic_rate_monitor");
  ASSERT_NE(f, nullptr);
  EXPECT_TRUE(f->active);
  EXPECT_TRUE(f->stop_worthy);
  EXPECT_EQ(s.state(), State::kStopUnconfirmed);   // no arbiter in this test
  EXPECT_TRUE(s.hold());
  EXPECT_EQ(s.hold_id(), kRates);
  auto missing = s.missing_nodes(5.5);
  ASSERT_EQ(missing.size(), 1u);
  EXPECT_EQ(missing[0], kRates);
}

// --- crash -------------------------------------------------------------------

// Weakness: R4 mapped every CRASH to LOG_ONLY.
TEST(SupervisorCrash, CriticalNodeCrashStops)
{
  auto s = started();
  run(s, 0.55, 1.0, {kRates});              // detector stops beating at 0.5
  EXPECT_TRUE(s.hold());
  EXPECT_EQ(s.hold_id(), kDetector);
  auto a = s.take_audit();
  EXPECT_TRUE(has_action(a, "STOP_AND_HOLD", "ACCEPTED"));
}

TEST(SupervisorCrash, NonCriticalCrashIsLogOnly)
{
  auto c = base_config();
  c.critical_nodes = {kDetector};
  SupervisorCore s(c);
  s.start(0.0);
  run(s, 0.0, 0.5, {kDetector, kRates});
  run(s, 0.55, 2.0, {kDetector});
  const auto * f = s.fault("CRASH:helix_topic_rate_monitor");
  ASSERT_NE(f, nullptr);
  EXPECT_TRUE(f->active);
  EXPECT_FALSE(f->stop_worthy);
  EXPECT_FALSE(s.hold());
  EXPECT_EQ(s.state(), State::kNominal);
}

TEST(SupervisorCrash, ClearsOnlyAfterContinuousHeartbeat)
{
  auto s = started();
  run(s, 0.55, 1.5, {kRates});
  ASSERT_TRUE(s.hold());
  // Beats resume, but a 0.6 s gap at t=2.0 restarts the clearance clock.
  run(s, 1.55, 2.0, {kDetector, kRates});
  run(s, 2.05, 2.6, {kRates});
  EXPECT_TRUE(s.fault("CRASH:helix_anomaly_detector")->active);
  run(s, 2.65, 3.6, {kDetector, kRates});   // 0.95 s continuous: not yet
  EXPECT_TRUE(s.fault("CRASH:helix_anomaly_detector")->active);
  run(s, 3.65, 3.8, {kDetector, kRates});
  EXPECT_FALSE(s.fault("CRASH:helix_anomaly_detector")->active);
}

// --- clearance ---------------------------------------------------------------

// Weakness: RESUME after 3 s of fault silence, with no proof of clearance.
TEST(SupervisorClear, SilenceNeverReleases)
{
  auto s = started();
  s.on_fault("ANOMALY", kCloud, 2, "stale", 1.0);
  confirm_stop(s, 1.01);
  run(s, 1.05, 30.0, {kDetector, kRates});
  EXPECT_TRUE(s.hold());
  EXPECT_EQ(s.state(), State::kStopConfirmed);
}

TEST(SupervisorClear, DetectorClearThenHoldTimeThenAutoRelease)
{
  auto s = started();
  s.on_fault("ANOMALY", kCloud, 2, "stale", 1.0);
  confirm_stop(s, 1.01);
  run(s, 1.05, 4.95, {kDetector, kRates});
  s.take_audit();
  s.on_fault_clear("ANOMALY", kCloud, 5.0);
  EXPECT_EQ(s.state(), State::kClearing);
  EXPECT_TRUE(s.hold());
  run(s, 5.05, 6.9, {kDetector, kRates});
  EXPECT_TRUE(s.hold());                    // clear_hold_s = 2.0
  run(s, 6.95, 7.1, {kDetector, kRates});
  EXPECT_FALSE(s.hold());
  EXPECT_EQ(s.state(), State::kNominal);
  EXPECT_TRUE(has_action(s.take_audit(), "RESUME", "ACCEPTED"));
}

TEST(SupervisorClear, RefaultDuringClearingKeepsSameEpisode)
{
  auto s = started();
  s.on_fault("ANOMALY", kCloud, 2, "stale", 1.0);
  confirm_stop(s, 1.01);
  s.on_fault_clear("ANOMALY", kCloud, 2.0);
  run(s, 2.05, 3.0, {kDetector, kRates});
  s.on_fault("ANOMALY", kCloud, 2, "z", 3.0);
  EXPECT_EQ(s.state(), State::kStopConfirmed);
  EXPECT_EQ(s.hold_episodes(), 1u);
  run(s, 3.05, 10.0, {kDetector, kRates});
  EXPECT_TRUE(s.hold());
}

TEST(SupervisorClear, NoAutoResumeWaitsForOperator)
{
  auto c = base_config();
  c.auto_resume = false;
  auto s = started(c);
  s.on_fault("ANOMALY", kCloud, 2, "stale", 1.0);
  run(s, 1.05, 1.95, {kDetector, kRates});
  s.on_fault_clear("ANOMALY", kCloud, 2.0);
  run(s, 2.05, 5.0, {kDetector, kRates});
  EXPECT_EQ(s.state(), State::kAwaitRelease);
  EXPECT_TRUE(s.hold());
  std::string why;
  EXPECT_TRUE(s.operator_release(5.0, why)) << why;
  EXPECT_FALSE(s.hold());
}

TEST(SupervisorClear, CriticalLogIsLatchedUntilOperator)
{
  auto s = started();
  s.on_fault("LOG_PATTERN", "rosout/lidar_driver", 3, "FATAL", 1.0);
  EXPECT_TRUE(s.hold());
  s.on_fault_clear("LOG_PATTERN", "rosout/lidar_driver", 2.0);   // ignored
  run(s, 1.05, 20.0, {kDetector, kRates});
  EXPECT_TRUE(s.hold());
  std::string why;
  EXPECT_TRUE(s.operator_release(20.0, why)) << why;
  EXPECT_FALSE(s.hold());
}

TEST(SupervisorClear, OperatorReleaseRefusedWhileFaultActive)
{
  auto s = started();
  s.on_fault("ANOMALY", kCloud, 2, "stale", 1.0);
  std::string why;
  EXPECT_FALSE(s.operator_release(2.0, why));
  EXPECT_NE(why.find(kCloud), std::string::npos);
  EXPECT_TRUE(s.hold());
}

TEST(SupervisorClear, LogOnlyFaultExpiresWithoutHolding)
{
  auto s = started();
  s.on_fault("ANOMALY", "rate_hz/gnss", 2, "z", 1.0);
  EXPECT_FALSE(s.hold());
  EXPECT_TRUE(s.fault("ANOMALY:rate_hz/gnss")->active);
  run(s, 1.05, 11.2, {kDetector, kRates});
  EXPECT_FALSE(s.fault("ANOMALY:rate_hz/gnss")->active);
}

// --- one state machine, one enable flag --------------------------------------

// Weakness: diagnosis assumed a stop was in force even when recovery
// suppressed it; enable was read only at configure.
TEST(SupervisorEnable, DisabledRunsSameMachineButPublishesNoHold)
{
  auto c = base_config();
  c.actuation_enabled = false;
  auto s = started(c);
  s.on_fault("ANOMALY", kCloud, 2, "stale", 1.0);
  EXPECT_TRUE(s.would_hold());
  EXPECT_FALSE(s.hold());
  EXPECT_EQ(s.state(), State::kHoldRequested);
  auto a = s.take_audit();
  EXPECT_TRUE(has_action(a, "STOP_AND_HOLD", "SUPPRESSED_DISABLED"));
  run(s, 1.05, 5.0, {kDetector, kRates});   // advisory: no ack timeout
  EXPECT_EQ(s.state(), State::kHoldRequested);
}

TEST(SupervisorEnable, EnablingDuringActiveFaultAssertsImmediately)
{
  auto c = base_config();
  c.actuation_enabled = false;
  auto s = started(c);
  s.on_fault("ANOMALY", kCloud, 2, "stale", 1.0);
  s.take_audit();
  s.set_actuation_enabled(true, 2.0);
  EXPECT_TRUE(s.hold());
  EXPECT_DOUBLE_EQ(s.hold_requested_at(), 2.0);
  EXPECT_TRUE(has_action(s.take_audit(), "STOP_AND_HOLD", "ACCEPTED"));
  s.set_actuation_enabled(false, 3.0);
  EXPECT_FALSE(s.hold());
  EXPECT_TRUE(s.would_hold());
}

// --- acknowledgments ---------------------------------------------------------

TEST(SupervisorAck, FullChainReachesStopConfirmed)
{
  auto s = started();
  s.on_fault("ANOMALY", kCloud, 2, "stale", 1.0);
  EXPECT_EQ(s.state(), State::kHoldRequested);
  s.on_arbiter_status("HELIX_HOLD", kCloud, 1.01);
  EXPECT_EQ(s.state(), State::kHoldApplied);
  SinkFeedback fb;
  fb.feedback_available = true;
  fb.stop_acked = true;
  fb.stop_ack_sent_at = 1.012;
  fb.stop_acked_at = 1.03;
  fb.stopped = false;
  fb.state_at = 1.03;
  s.on_sink_feedback(fb, 1.03);
  EXPECT_EQ(s.state(), State::kStopAcked);
  fb.stopped = true;
  fb.state_at = 1.4;
  s.on_sink_feedback(fb, 1.4);
  EXPECT_EQ(s.state(), State::kStopConfirmed);
  EXPECT_DOUBLE_EQ(s.controller_ack_at(), 1.03);
  EXPECT_DOUBLE_EQ(s.motion_stopped_at(), 1.4);
}

TEST(SupervisorAck, ArbiterStatusForOtherHoldIsNotAnAck)
{
  auto s = started();
  s.on_fault("ANOMALY", kCloud, 2, "stale", 1.0);
  s.on_arbiter_status("HELIX_HOLD", "something_else", 1.01);
  EXPECT_EQ(s.state(), State::kHoldRequested);
  run(s, 1.05, 1.4, {kDetector, kRates});
  EXPECT_EQ(s.state(), State::kStopUnconfirmed);
}

TEST(SupervisorAck, StopMoveSentBeforeHoldIsNotAnAck)
{
  auto s = started();
  s.on_fault("ANOMALY", kCloud, 2, "stale", 1.0);
  s.on_arbiter_status("HELIX_HOLD", kCloud, 1.01);
  SinkFeedback fb;
  fb.feedback_available = true;
  fb.stop_acked = true;
  fb.stop_ack_sent_at = 0.9;    // repeat StopMove from before the fault
  fb.stop_acked_at = 0.95;
  fb.stopped = true;
  fb.state_at = 1.02;
  s.on_sink_feedback(fb, 1.02);
  EXPECT_EQ(s.state(), State::kHoldApplied);
}

TEST(SupervisorAck, MissingControllerAckTimesOutThenLateAckAdvances)
{
  auto s = started();
  s.on_fault("ANOMALY", kCloud, 2, "stale", 1.0);
  s.on_arbiter_status("HELIX_HOLD", kCloud, 1.01);
  run(s, 1.05, 2.6, {kDetector, kRates});
  EXPECT_EQ(s.state(), State::kStopUnconfirmed);
  EXPECT_TRUE(s.hold());
  SinkFeedback fb;
  fb.feedback_available = true;
  fb.stop_acked = true;
  fb.stop_ack_sent_at = 2.5;
  fb.stop_acked_at = 2.62;
  fb.stopped = true;
  fb.state_at = 2.63;
  s.on_sink_feedback(fb, 2.63);
  EXPECT_EQ(s.state(), State::kStopConfirmed);
}

TEST(SupervisorAck, MotionWhileConfirmedIsUnconfirmed)
{
  auto s = started();
  s.on_fault("ANOMALY", kCloud, 2, "stale", 1.0);
  confirm_stop(s, 1.01);
  ASSERT_EQ(s.state(), State::kStopConfirmed);
  SinkFeedback fb = s.last_feedback();
  fb.stopped = false;
  fb.state_at = 2.0;
  s.on_sink_feedback(fb, 2.0);
  EXPECT_EQ(s.state(), State::kStopUnconfirmed);
  fb.stopped = true;
  fb.state_at = 2.5;
  s.on_sink_feedback(fb, 2.5);
  EXPECT_EQ(s.state(), State::kStopConfirmed);
}

TEST(SupervisorAck, ArbiterPassingSourceDuringHoldIsUnconfirmed)
{
  auto s = started();
  s.on_fault("ANOMALY", kCloud, 2, "stale", 1.0);
  s.on_arbiter_status("HELIX_HOLD", kCloud, 1.01);
  s.on_arbiter_status("SOURCE", "", 1.5);
  EXPECT_EQ(s.state(), State::kStopUnconfirmed);
  s.on_arbiter_status("HELIX_HOLD", kCloud, 1.6);
  EXPECT_EQ(s.state(), State::kHoldApplied);
}

TEST(SupervisorAck, WithoutControllerAckHoldAppliedIsTerminal)
{
  auto c = base_config();
  c.require_controller_ack = false;
  auto s = started(c);
  s.on_fault("ANOMALY", kCloud, 2, "stale", 1.0);
  s.on_arbiter_status("HELIX_HOLD", kCloud, 1.01);
  run(s, 1.05, 10.0, {kDetector, kRates});
  EXPECT_EQ(s.state(), State::kHoldApplied);
}

TEST(SupervisorConfig, RejectsUnsafeTimeouts)
{
  Config c = base_config();
  c.heartbeat_timeout_s = 0.0;
  EXPECT_THROW(SupervisorCore{c}, std::invalid_argument);
  c = base_config();
  c.clear_hold_s = -1.0;
  EXPECT_THROW(SupervisorCore{c}, std::invalid_argument);
}

TEST(SupervisorAudit, TransitionsAreRecordedInOrder)
{
  auto s = started();
  s.take_audit();
  s.on_fault("ANOMALY", kCloud, 2, "stale", 1.0);
  confirm_stop(s, 1.01);
  std::vector<std::string> states;
  for (const auto & e : s.take_audit()) {
    if (e.kind == "TRANSITION") {
      states.push_back(e.action);
    }
  }
  const std::vector<std::string> want{
    "HOLD_REQUESTED", "HOLD_APPLIED", "STOP_CONFIRMED"};
  EXPECT_EQ(states, want);
}
