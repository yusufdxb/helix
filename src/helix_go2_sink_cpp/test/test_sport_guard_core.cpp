// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
#include <gtest/gtest.h>

#include <limits>
#include <string>

#include "helix_go2_sink_cpp/sport_guard_core.hpp"

using helix_go2_sink_cpp::kApiMove;
using helix_go2_sink_cpp::kApiStopMove;
using helix_go2_sink_cpp::Request;
using helix_go2_sink_cpp::SinkFeedback;
using helix_go2_sink_cpp::SportGuard;

namespace
{
Request move()
{
  Request request;
  request.api_id = kApiMove;
  request.parameter = "{\"x\":0.2,\"y\":0,\"z\":0}";
  return request;
}

void release(SportGuard & guard, const std::string & challenge = "arbiter:1", double now = 0.0)
{
  ASSERT_TRUE(guard.on_status(1, challenge, true, true, now));
  EXPECT_FALSE(guard.on_request(1, move(), now + 0.01));
  ASSERT_TRUE(guard.on_status(2, challenge, false, false, now + 0.02));
  ASSERT_TRUE(guard.latched());
  ASSERT_TRUE(guard.on_operator_release(guard.release_token(), now + 0.02));
  ASSERT_FALSE(guard.latched());
}
}  // namespace

TEST(SportGuard, StartupRepeatsStopWithoutInput)
{
  SportGuard guard("test-guard");
  EXPECT_TRUE(guard.latched());
  EXPECT_TRUE(guard.fault_required());
  EXPECT_EQ(guard.on_tick(0.0)->api_id, kApiStopMove);
  EXPECT_FALSE(guard.on_tick(0.01));
  EXPECT_EQ(guard.on_tick(0.06)->api_id, kApiStopMove);
  EXPECT_FALSE(guard.on_request(1, move(), 0.07));
}

TEST(SportGuard, OnlyObservedCurrentOperatorReleaseOpensGate)
{
  SportGuard guard("test-guard");
  EXPECT_FALSE(guard.on_status(9, "arbiter:1", false, false, 0.0));
  EXPECT_FALSE(guard.on_request(1, move(), 0.01));
  EXPECT_TRUE(guard.latched());
  EXPECT_TRUE(guard.on_status(10, "arbiter:2", true, true, 0.02));
  EXPECT_TRUE(guard.on_status(11, "arbiter:2", false, false, 0.03));
  EXPECT_TRUE(guard.latched());
  EXPECT_TRUE(guard.on_operator_release(guard.release_token(), 0.03));
  EXPECT_FALSE(guard.latched());
  // No cached Move is replayed on release.
  EXPECT_FALSE(guard.on_tick(0.04));
  EXPECT_FALSE(guard.on_request(1, move(), 0.04));
  EXPECT_EQ(guard.on_request(2, move(), 0.05)->api_id, kApiMove);
}

TEST(SportGuard, SinkDeathStopsControllerThatNeverExpiresMove)
{
  SportGuard guard("test-guard");
  release(guard);
  bool controller_moving = false;
  if (guard.on_request(2, move(), 0.03)) {
    controller_moving = true;
  }
  ASSERT_TRUE(controller_moving);
  // The controller never stops by itself. Only guard output changes it.
  for (int i = 1; i <= 10; ++i) {
    const double now = 0.03 + i * 0.04;
    guard.on_status(2 + i, "arbiter:1", false, false, now);
    if (const auto request = guard.on_tick(now)) {
      if (request->api_id == kApiStopMove) {
        controller_moving = false;
      }
    }
  }
  EXPECT_FALSE(controller_moving);
  EXPECT_TRUE(guard.latched());
  EXPECT_TRUE(guard.fault_required());
}

TEST(SportGuard, LateMoveChecksGapBeforeRefreshingProgress)
{
  SportGuard guard("test-guard");
  release(guard);
  ASSERT_TRUE(guard.on_request(2, move(), 0.03));
  guard.on_status(3, "arbiter:1", false, false, 0.20);
  EXPECT_FALSE(guard.on_request(3, move(), 0.29));
  EXPECT_TRUE(guard.latched());
  EXPECT_EQ(guard.reason(), "SINK_PROGRESS_TIMEOUT");
}

TEST(SportGuard, DuplicateAndOldRequestsCannotRefreshProgress)
{
  SportGuard guard("test-guard");
  release(guard);
  ASSERT_TRUE(guard.on_request(3, move(), 0.03));
  EXPECT_FALSE(guard.on_request(3, move(), 0.20));
  EXPECT_FALSE(guard.on_request(2, move(), 0.21));
  guard.on_status(3, "arbiter:1", false, false, 0.22);
  ASSERT_TRUE(guard.on_tick(0.29));
  EXPECT_TRUE(guard.latched());
}

TEST(SportGuard, TimeoutNeedsNewChallengeAndNewRelease)
{
  SportGuard guard("test-guard");
  release(guard);
  guard.on_request(2, move(), 0.03);
  guard.on_tick(0.29);
  // New sink progress cannot release either latch by itself.
  EXPECT_FALSE(guard.on_request(3, move(), 0.30));
  guard.on_status(3, "arbiter:1", false, false, 0.31);
  guard.on_status(4, "arbiter:1", true, true, 0.32);
  guard.on_status(5, "arbiter:1", false, false, 0.33);
  EXPECT_TRUE(guard.latched());
  EXPECT_TRUE(guard.fault_required());
  guard.on_status(6, "arbiter:2", true, true, 0.34);
  EXPECT_FALSE(guard.fault_required());
  guard.on_request(4, move(), 0.345);
  guard.on_status(7, "arbiter:2", false, false, 0.35);
  EXPECT_TRUE(guard.on_operator_release(guard.release_token(), 0.35));
  EXPECT_FALSE(guard.latched());
  EXPECT_FALSE(guard.on_request(4, move(), 0.36));
  EXPECT_TRUE(guard.on_request(5, move(), 0.36));
}

TEST(SportGuard, PrematureReleaseIsNeverBankedForLaterInput)
{
  SportGuard guard("test-guard");
  guard.on_status(1, "arbiter:1", true, true, 0.0);
  guard.on_status(2, "arbiter:1", false, false, 0.01);
  EXPECT_FALSE(guard.on_operator_release(guard.release_token(), 0.01));
  EXPECT_TRUE(guard.latched());
  guard.on_request(1, move(), 0.02);
  guard.on_status(3, "arbiter:1", false, false, 0.03);
  EXPECT_TRUE(guard.latched());
  guard.on_status(4, "arbiter:2", true, true, 0.04);
  guard.on_request(2, move(), 0.045);
  guard.on_status(5, "arbiter:2", false, false, 0.05);
  EXPECT_TRUE(guard.on_operator_release(guard.release_token(), 0.05));
  EXPECT_FALSE(guard.latched());
}

TEST(SportGuard, DuplicateStatusCannotRefreshArbiterHealth)
{
  SportGuard guard("test-guard");
  release(guard);
  guard.on_request(2, move(), 0.1);
  EXPECT_FALSE(guard.on_status(2, "arbiter:1", false, false, 0.25));
  guard.on_request(3, move(), 0.30);
  ASSERT_TRUE(guard.on_tick(0.33));
  EXPECT_TRUE(guard.latched());
  EXPECT_EQ(guard.reason(), "ARBITER_STATUS_TIMEOUT");
}

TEST(SportGuard, StatusGapIsCheckedBeforeReceiptTimeIsReplaced)
{
  SportGuard guard("test-guard");
  release(guard);
  guard.on_request(2, move(), 0.2);
  guard.on_status(3, "arbiter:1", false, false, 0.33);
  EXPECT_TRUE(guard.latched());
}

TEST(SportGuard, GuardRestartsStayLatchedUntilNewProtectiveHandshake)
{
  for (int restart = 0; restart < 20; ++restart) {
    SportGuard guard("test-guard");
    EXPECT_FALSE(guard.on_status(50, "arbiter:1", false, false, 0.0));
    EXPECT_FALSE(guard.on_request(100 + restart, move(), 0.01));
    EXPECT_TRUE(guard.latched());
    guard.on_status(51, "arbiter:2", true, true, 0.02);
    guard.on_request(200 + restart, move(), 0.025);
    guard.on_status(52, "arbiter:2", false, false, 0.03);
    EXPECT_TRUE(guard.on_operator_release(guard.release_token(), 0.03));
    EXPECT_FALSE(guard.latched());
  }
}

TEST(SportGuard, ArbiterRestartsRejectRetiredSessionsAndStaleReleases)
{
  SportGuard guard("test-guard");
  release(guard);
  guard.on_status(1, "new_arbiter:1", true, true, 0.03);
  EXPECT_TRUE(guard.latched());
  EXPECT_FALSE(guard.on_status(500, "arbiter:1", false, false, 0.04));
  EXPECT_FALSE(guard.on_status(500, "arbiter:2", true, true, 0.05));
  guard.on_request(2, move(), 0.055);
  EXPECT_TRUE(guard.on_status(2, "new_arbiter:1", false, false, 0.06));
  EXPECT_TRUE(guard.on_operator_release(guard.release_token(), 0.06));
  EXPECT_FALSE(guard.latched());
}

TEST(SportGuard, NewFalseGenerationCannotReleaseWithoutObservedHold)
{
  SportGuard guard("test-guard");
  release(guard);
  guard.on_status(3, "arbiter:2", false, false, 0.03);
  EXPECT_TRUE(guard.latched());
  EXPECT_TRUE(guard.fault_required());
}

TEST(SportGuard, ProgressBeforeAProtectionEventCannotAuthorizeItsRelease)
{
  SportGuard guard("test-guard");
  release(guard);
  guard.on_request(2, move(), 0.03);
  guard.on_tick(0.29);
  guard.on_status(3, "arbiter:2", true, true, 0.30);
  guard.on_status(4, "arbiter:2", false, false, 0.31);
  EXPECT_FALSE(guard.on_operator_release(guard.release_token(), 0.31));
  EXPECT_TRUE(guard.latched());
  EXPECT_TRUE(guard.fault_required());
  guard.on_request(3, move(), 0.32);
  guard.on_status(5, "arbiter:2", false, false, 0.33);
  EXPECT_TRUE(guard.latched());
}

TEST(SportGuard, StopIntentsPassWhileHeldAndIdleReadinessAllowsSinkRepeatPeriod)
{
  SportGuard guard("test-guard");
  Request stop;
  EXPECT_EQ(guard.on_request(17, stop, 0.0)->api_id, kApiStopMove);
  guard.on_status(1, "arbiter:1", true, true, 0.45);
  guard.on_status(2, "arbiter:1", false, false, 0.5);
  EXPECT_TRUE(guard.on_operator_release(guard.release_token(), 0.5));
  EXPECT_FALSE(guard.latched());
  guard.on_status(3, "arbiter:1", false, false, 0.7);
  EXPECT_FALSE(guard.on_tick(0.7));
  EXPECT_TRUE(guard.on_tick(0.76));
  EXPECT_TRUE(guard.latched());
}

TEST(SportGuard, ForbiddenApiAndNonpositiveIdsNeverPass)
{
  SportGuard guard("test-guard");
  release(guard);
  EXPECT_FALSE(guard.on_request(-17, move(), 0.03));
  Request damp;
  damp.api_id = 1001;
  EXPECT_FALSE(guard.on_request(2, damp, 0.04));
  EXPECT_TRUE(guard.latched());
  EXPECT_TRUE(guard.fault_required());
}

TEST(SportGuard, AckIsNotMeasuredStopAndRejectedOrLostStopRemainsUnproven)
{
  SinkFeedback feedback;
  feedback.on_sent(-200, kApiStopMove, "GUARD", 0.0);
  EXPECT_FALSE(feedback.stop_acked);
  EXPECT_FALSE(feedback.stopped_now(0.1));
  EXPECT_TRUE(feedback.on_response(-200, kApiStopMove, -1, 0.1));
  EXPECT_FALSE(feedback.stop_acked);
  EXPECT_FALSE(feedback.stopped_now(0.1));
  feedback.on_sent(-199, kApiStopMove, "GUARD", 0.2);
  feedback.on_response(-199, kApiStopMove, 0, 0.21);
  EXPECT_TRUE(feedback.stop_acked);
  EXPECT_FALSE(feedback.stopped_now(0.21));
}

TEST(SportGuard, InvalidTimeoutsRejected)
{
  EXPECT_THROW(SportGuard("test", 0.0), std::invalid_argument);
  EXPECT_THROW(SportGuard("test", -1.0), std::invalid_argument);
  EXPECT_THROW(SportGuard("test", std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
  EXPECT_THROW(SportGuard(""), std::invalid_argument);
}

TEST(SportGuard, ReplayedHeldReleasedPairCannotOpenARestartedGuard)
{
  SportGuard old_guard("old-random-instance");
  const auto old_token = old_guard.release_token();
  release(old_guard);
  SportGuard restarted("new-random-instance");
  // A valid-looking delayed pair has no authority over the new guard instance.
  restarted.on_status(40, "arbiter:7", true, true, 1.0);
  restarted.on_request(99, move(), 1.01);
  restarted.on_status(41, "arbiter:7", false, false, 1.02);
  EXPECT_TRUE(restarted.ready_for_release(1.02));
  EXPECT_TRUE(restarted.latched());
  EXPECT_FALSE(restarted.on_operator_release(old_token, 1.03));
  EXPECT_FALSE(restarted.on_request(100, move(), 1.04));
  EXPECT_TRUE(restarted.latched());
  EXPECT_TRUE(restarted.on_operator_release(restarted.release_token(), 1.05));
  EXPECT_TRUE(restarted.on_request(101, move(), 1.06));
}

TEST(SportGuard, EveryTripInvalidatesEarlierGuardOperatorAuthorization)
{
  SportGuard guard("unique-instance");
  const auto startup_token = guard.release_token();
  release(guard);
  guard.on_request(2, move(), 0.03);
  guard.on_tick(0.29);
  const auto tripped_token = guard.release_token();
  EXPECT_NE(startup_token, tripped_token);
  guard.on_status(3, "arbiter:2", true, true, 0.30);
  guard.on_request(3, move(), 0.31);
  guard.on_status(4, "arbiter:2", false, false, 0.32);
  EXPECT_FALSE(guard.on_operator_release(startup_token, 0.33));
  EXPECT_FALSE(guard.on_operator_release(tripped_token, 0.33));
  EXPECT_TRUE(guard.latched());
  EXPECT_TRUE(guard.on_operator_release(guard.release_token(), 0.34));
}

TEST(SportGuard, PrehealthOperatorTokenIsRejectedAndNeverBanked)
{
  SportGuard guard("unique-instance");
  const auto premature_token = guard.release_token();
  EXPECT_FALSE(guard.on_operator_release(premature_token, 0.0));
  EXPECT_NE(premature_token, guard.release_token());
  guard.on_status(1, "arbiter:1", true, true, 0.01);
  guard.on_request(1, move(), 0.02);
  guard.on_status(2, "arbiter:1", false, false, 0.03);
  EXPECT_TRUE(guard.latched());
  EXPECT_FALSE(guard.on_operator_release(premature_token, 0.04));
  EXPECT_TRUE(guard.on_operator_release(guard.release_token(), 0.05));
}

TEST(SportGuard, DelayedPrehealthTokenCannotAuthorizeLaterReadiness)
{
  SportGuard guard("unique-instance");
  const auto early_token = guard.release_token();
  guard.on_status(1, "arbiter:1", true, true, 0.0);
  guard.on_request(1, move(), 0.01);
  guard.on_status(2, "arbiter:1", false, false, 0.02);
  ASSERT_TRUE(guard.ready_for_release(0.02));
  const auto ready_token = guard.release_token();
  EXPECT_NE(early_token, ready_token);
  EXPECT_FALSE(guard.on_operator_release(early_token, 0.03));
  EXPECT_TRUE(guard.latched());
  guard.on_status(3, "arbiter:1", false, false, 0.10);
  guard.on_request(2, move(), 0.12);
  guard.on_tick(0.15);
  EXPECT_EQ(ready_token, guard.release_token());  // Continuous readiness is stable.
  EXPECT_TRUE(guard.on_operator_release(ready_token, 0.16));
}

TEST(SportGuard, ExpiredReadyRequestTokenCannotBeRevivedByFreshProgress)
{
  SportGuard guard("unique-instance");
  guard.on_status(1, "arbiter:1", true, true, 0.0);
  guard.on_request(1, move(), 0.01);
  guard.on_status(2, "arbiter:1", false, false, 0.02);
  const auto expired_token = guard.release_token();
  guard.on_status(3, "arbiter:1", false, false, 0.22);
  guard.on_status(4, "arbiter:1", false, false, 0.42);
  guard.on_status(5, "arbiter:1", false, false, 0.62);
  guard.on_request(2, move(), 0.78);  // Check the request gap before receipt update.
  EXPECT_TRUE(guard.latched());
  EXPECT_FALSE(guard.ready_for_release(0.78));
  EXPECT_NE(expired_token, guard.release_token());
  guard.on_status(6, "arbiter:1", false, false, 0.79);
  EXPECT_FALSE(guard.on_operator_release(expired_token, 0.80));
  guard.on_status(7, "arbiter:2", true, true, 0.81);
  guard.on_request(3, move(), 0.82);
  guard.on_status(8, "arbiter:2", false, false, 0.83);
  EXPECT_TRUE(guard.ready_for_release(0.83));
  EXPECT_FALSE(guard.on_operator_release(expired_token, 0.84));
  EXPECT_TRUE(guard.on_operator_release(guard.release_token(), 0.85));
}

TEST(SportGuard, ExpiredReadyStatusTokenCannotBeRevivedByFreshStatus)
{
  SportGuard guard("unique-instance");
  guard.on_status(1, "arbiter:1", true, true, 0.0);
  guard.on_request(1, move(), 0.01);
  guard.on_status(2, "arbiter:1", false, false, 0.02);
  const auto expired_token = guard.release_token();
  guard.on_request(2, move(), 0.20);
  guard.on_status(3, "arbiter:1", false, false, 0.33);
  EXPECT_NE(expired_token, guard.release_token());
  EXPECT_FALSE(guard.ready_for_release(0.33));
  guard.on_request(3, move(), 0.34);
  guard.on_status(4, "arbiter:1", false, false, 0.35);
  EXPECT_FALSE(guard.on_operator_release(expired_token, 0.36));
  EXPECT_TRUE(guard.latched());
}
