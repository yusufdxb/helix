// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// Mirrors helix_arbiter/test/test_sport_sink_core.py and
// test_sink_feedback.py for the C++ core.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "helix_go2_sink_cpp/sink_core.hpp"

using helix_go2_sink_cpp::kApiMove;
using helix_go2_sink_cpp::kApiStopMove;
using helix_go2_sink_cpp::Mode;
using helix_go2_sink_cpp::SinkFeedback;
using helix_go2_sink_cpp::SinkLogic;
using helix_go2_sink_cpp::StillThresholds;

TEST(SinkLogic, ZeroSendsStopMoveOnceThenRateLimited)
{
  SinkLogic s(Mode::kArmed);
  EXPECT_EQ(s.on_command(0, 0, 0, 0.0)->api_id, kApiStopMove);
  EXPECT_FALSE(s.on_command(0, 0, 0, 0.02));
  EXPECT_EQ(s.on_command(0, 0, 0, 0.6)->api_id, kApiStopMove);
}

TEST(SinkLogic, ArmedMovePayload)
{
  SinkLogic s(Mode::kArmed);
  auto r = s.on_command(0.15, 0.0, 0.1, 0.0);
  ASSERT_TRUE(r);
  EXPECT_EQ(r->api_id, kApiMove);
  EXPECT_EQ(r->parameter, "{\"x\": 0.15, \"y\": 0.0, \"z\": 0.1}");
}

TEST(SinkLogic, StopOnlyNeverMoves)
{
  SinkLogic s(Mode::kStopOnly);
  for (int i = 0; i < 100; ++i) {
    auto r = s.on_command(0.2, 0, 0, i * 0.02);
    EXPECT_TRUE(!r || r->api_id == kApiStopMove);
  }
}

TEST(SinkLogic, MovingThenZeroStopsImmediately)
{
  SinkLogic s(Mode::kArmed);
  s.on_command(0.2, 0, 0, 0.0);
  auto r = s.on_command(0.0, 0, 0, 0.01);
  ASSERT_TRUE(r);
  EXPECT_EQ(r->api_id, kApiStopMove);
  EXPECT_EQ(r->reason, "ZERO");
}

TEST(SinkLogic, OverLimitStopsNotClamps)
{
  SinkLogic s(Mode::kArmed);
  s.on_command(0.2, 0, 0, 0.0);
  auto r = s.on_command(0.9, 0, 0, 0.01);
  ASSERT_TRUE(r);
  EXPECT_EQ(r->reason, "REJECT_OVER_SINK_LIMIT");
}

TEST(SinkLogic, NonFiniteStops)
{
  for (double bad : {std::nan(""), std::numeric_limits<double>::infinity()}) {
    SinkLogic s(Mode::kArmed);
    s.on_command(0.2, 0, 0, 0.0);
    EXPECT_EQ(s.on_command(bad, 0, 0, 0.001)->api_id, kApiStopMove);
  }
}

TEST(SinkLogic, DeadmanOnInputSilence)
{
  SinkLogic s(Mode::kArmed, {}, 0.25);
  s.on_command(0.2, 0, 0, 0.0);
  EXPECT_FALSE(s.on_tick(0.2));
  auto r = s.on_tick(0.26);
  ASSERT_TRUE(r);
  EXPECT_EQ(r->reason, "DEADMAN");
  EXPECT_FALSE(s.on_tick(0.3));
  EXPECT_EQ(s.on_tick(0.8)->api_id, kApiStopMove);
}

TEST(SinkLogic, DeadmanBeforeAnyInput)
{
  SinkLogic s(Mode::kArmed);
  EXPECT_EQ(s.on_tick(0.0)->reason, "DEADMAN");
}

TEST(SinkLogic, MoveRateLimited)
{
  SinkLogic s(Mode::kArmed, {}, 0.25, 2.0, 20.0);
  EXPECT_EQ(s.on_command(0.2, 0, 0, 0.0)->api_id, kApiMove);
  EXPECT_FALSE(s.on_command(0.2, 0, 0, 0.02));
  EXPECT_EQ(s.on_command(0.2, 0, 0, 0.05)->api_id, kApiMove);
}

TEST(SinkLogic, BadModeRefused)
{
  EXPECT_THROW(helix_go2_sink_cpp::parse_mode("normal"), std::invalid_argument);
}

TEST(SinkFeedback, StopAckRequiresOwnRequestAndCodeZero)
{
  SinkFeedback fb;
  fb.on_sent(10, kApiStopMove, "ZERO", 1.0);
  EXPECT_FALSE(fb.on_response(99, kApiStopMove, 0, 1.01));
  EXPECT_TRUE(fb.on_response(10, kApiStopMove, 3, 1.02));
  EXPECT_FALSE(fb.stop_acked);
  fb.on_sent(11, kApiStopMove, "ZERO", 1.5);
  EXPECT_TRUE(fb.on_response(11, kApiStopMove, 0, 1.52));
  EXPECT_TRUE(fb.stop_acked);
  EXPECT_DOUBLE_EQ(fb.stop_ack_sent_at, 1.5);
}

TEST(SinkFeedback, MoveClearsAckAndStaleStopAckIgnored)
{
  SinkFeedback fb;
  fb.on_sent(10, kApiStopMove, "ZERO", 1.0);
  fb.on_sent(11, kApiMove, "MOVE", 1.1);
  fb.on_response(10, kApiStopMove, 0, 1.2);
  EXPECT_FALSE(fb.stop_acked);
  fb.on_sent(12, kApiStopMove, "DEADMAN", 1.3);
  fb.on_response(12, kApiStopMove, 0, 1.33);
  EXPECT_TRUE(fb.stop_acked);
  fb.on_sent(13, kApiMove, "MOVE", 2.0);
  EXPECT_FALSE(fb.stop_acked);
}

TEST(SinkFeedback, StoppedNeedsStillSAndFreshState)
{
  SinkFeedback fb(StillThresholds{0.05, 0.1, 0.3}, 1.0);
  fb.on_state(0.2, 0, 0, 3, 0.0);
  fb.on_state(0.01, 0.01, 0.02, 3, 0.1);
  fb.on_state(0.01, 0, 0, 3, 0.3);
  EXPECT_FALSE(fb.stopped);
  fb.on_state(0, 0, 0, 3, 0.41);
  EXPECT_TRUE(fb.stopped);
  EXPECT_TRUE(fb.stopped_now(1.0));
  EXPECT_FALSE(fb.stopped_now(1.5));
  fb.on_state(0, 0, 0.3, 3, 1.6);
  EXPECT_FALSE(fb.stopped);
}

TEST(SinkFeedback, TrackingIsBounded)
{
  SinkFeedback fb;
  for (int i = 0; i < static_cast<int>(SinkFeedback::kMaxTracked) + 50; ++i) {
    fb.on_sent(i, kApiStopMove, "ZERO", i);
  }
  EXPECT_FALSE(fb.on_response(0, kApiStopMove, 0, 1e4));
}
