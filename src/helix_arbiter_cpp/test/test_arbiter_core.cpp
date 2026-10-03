// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// Absolute expectations for the native arbiter core (no ROS). Behavioural
// equality with the Python reference is covered separately and exhaustively
// by test_python_parity.py; these tests pin the safety properties themselves,
// so a drift that affected both implementations would still fail here.

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "helix_arbiter_cpp/arbiter_core.hpp"

using helix_arbiter_cpp::Arbiter;
using helix_arbiter_cpp::Command;
using helix_arbiter_cpp::kZero;
using helix_arbiter_cpp::Limits;
using helix_arbiter_cpp::Reason;
using helix_arbiter_cpp::Rejection;
using helix_arbiter_cpp::SourceSpec;
using helix_arbiter_cpp::Twist6;

namespace
{

constexpr std::uint64_t kU64Max = std::numeric_limits<std::uint64_t>::max();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

const SourceSpec kTeleop{"teleop", "/teleop/cmd_vel", 200, 0.5};
const SourceSpec kNav{"nav", "/nav/cmd_vel", 50, 0.5};

Twist6 vx(double v, double wz = 0.0) {return Twist6{v, 0.0, 0.0, 0.0, 0.0, wz};}

struct Rig
{
  Arbiter arb{{kTeleop, kNav}, 0.5};
  std::uint64_t seq{0};
  double t{0.0};

  bool hold(bool h, std::uint64_t epoch = 1000)
  {
    return arb.on_hold(h, h ? "f" : "", epoch, ++seq, t);
  }
  bool src(const char * name, double v) {return arb.on_source(*arb.index_of(name), vx(v), t);}
  helix_arbiter_cpp::Decision step(double dt)
  {
    t += dt;
    return arb.decide(t);
  }
};

}  // namespace

TEST(ArbiterCore, MissingHoldStateFailsClosed)
{
  Rig r;
  r.src("teleop", 0.4);
  const auto d = r.arb.decide(0.0);
  EXPECT_EQ(d.command, kZero);
  EXPECT_EQ(d.reason, Reason::kMissing);
  EXPECT_TRUE(d.helix_forced());
}

TEST(ArbiterCore, HoldBeatsHighestPriorityAndKeepsFaultId)
{
  Rig r;
  r.hold(false);
  r.src("teleop", 0.5);
  EXPECT_EQ(r.step(0.02).command.vx, 0.5);
  r.hold(true);
  r.src("teleop", 0.5);
  const auto d = r.step(0.02);
  EXPECT_EQ(d.command, kZero);
  EXPECT_EQ(d.reason, Reason::kHold);
  EXPECT_EQ(d.hold_fault_id, "f");
}

TEST(ArbiterCore, ResumeNeverReplaysAPreHoldCommand)
{
  Rig r;
  r.hold(false);
  r.src("nav", 0.3);
  r.step(0.02);
  r.hold(true);
  r.src("nav", 0.3);  // received during the hold
  r.step(0.02);
  r.hold(false);
  const auto d = r.step(0.02);
  EXPECT_EQ(d.command, kZero);
  EXPECT_EQ(d.reason, Reason::kNoInput);
  r.src("nav", 0.3);
  EXPECT_EQ(r.step(0.02).command.vx, 0.3);
}

TEST(ArbiterCore, StaleBoundaryIsInclusive)
{
  Rig r;
  r.src("nav", 0.2);
  r.arb.on_hold(false, "", 1, 1, 0.5);
  r.arb.on_source(1, vx(0.2), 0.0);
  EXPECT_EQ(r.arb.decide(0.5).command.vx, 0.2);   // age == timeout: live
  r.arb.on_hold(false, "", 1, 2, std::nextafter(0.5, 1.0));
  EXPECT_EQ(r.arb.decide(std::nextafter(0.5, 1.0)).command, kZero);  // just past
}

TEST(ArbiterCore, StaleHoldFailsClosedAndClearsSources)
{
  Rig r;
  r.hold(false);
  r.src("nav", 0.3);
  EXPECT_EQ(r.step(0.02).command.vx, 0.3);
  const auto d = r.step(0.49);  // hold age 0.51 > 0.5, source age 0.49
  EXPECT_EQ(d.reason, Reason::kStale);
  EXPECT_EQ(d.command, kZero);
  r.hold(false);  // recovery returns: the stored 0.3 must not come back
  EXPECT_EQ(r.arb.decide(r.t).reason, Reason::kNoInput);
}

TEST(ArbiterCore, HoldOrderingUsesFullWidthUnsignedIntegers)
{
  Arbiter a({kNav}, 0.5);
  // 2^53 + 1 is not representable as a double; a double comparison would
  // call these two equal and drop the newer state as a duplicate.
  const std::uint64_t e = (std::uint64_t{1} << 53);
  ASSERT_TRUE(a.on_hold(true, "f", e, e, 0.0));
  EXPECT_TRUE(a.on_hold(false, "", e, e + 1, 0.0));
  EXPECT_FALSE(a.on_hold(true, "f", e, e + 1, 0.0));     // duplicate
  EXPECT_FALSE(a.on_hold(true, "f", e - 1, kU64Max, 0.0));  // older epoch
  EXPECT_TRUE(a.on_hold(false, "", e + 1, 0, 0.0));       // newer epoch, seq restarts
  EXPECT_TRUE(a.on_hold(false, "", kU64Max, kU64Max, 0.0));
  EXPECT_FALSE(a.on_hold(true, "f", kU64Max, kU64Max, 0.0));
  EXPECT_EQ(a.counters().hold_reordered, 3u);
  EXPECT_EQ(a.decide(0.0).reason, Reason::kNoInput);
  // Once stale, any epoch is accepted: a restarted publisher can recover.
  EXPECT_TRUE(a.on_hold(false, "", 0, 0, 0.6));
}

TEST(ArbiterCore, PriorityThenMostRecentReceipt)
{
  Arbiter a({{"a", "/a", 100, 0.5}, {"b", "/b", 100, 0.5}, {"c", "/c", 10, 0.5}}, 0.5);
  a.on_hold(false, "", 1, 1, 0.0);
  a.on_source(2, vx(0.1), 0.0);
  EXPECT_EQ(a.decide(0.0).source, "c");
  a.on_source(0, vx(0.2), 0.0);
  a.on_source(1, vx(0.3), 0.0);   // same instant, later receipt
  EXPECT_EQ(a.decide(0.0).source, "b");
  a.on_source(0, vx(0.2), 0.0);
  EXPECT_EQ(a.decide(0.0).source, "a");
}

TEST(ArbiterCore, EveryAxisMustBeFiniteAndRejectionDiscardsTheOldValue)
{
  for (int axis = 0; axis < 6; ++axis) {
    for (double bad : {kNaN, kInf, -kInf}) {
      Rig r;
      r.hold(false);
      r.src("teleop", 0.4);
      ASSERT_EQ(r.step(0.01).command.vx, 0.4);
      double v[6] = {0.1, 0.0, 0.0, 0.0, 0.0, 0.0};
      v[axis] = bad;
      Rejection why = Rejection::kNone;
      EXPECT_FALSE(r.arb.on_source(0, Twist6{v[0], v[1], v[2], v[3], v[4], v[5]}, r.t, &why));
      EXPECT_EQ(why, Rejection::kNonFinite);
      const auto d = r.step(0.01);
      EXPECT_EQ(d.command, kZero) << "axis " << axis;
      EXPECT_EQ(d.reason, Reason::kNoInput) << "axis " << axis;
    }
  }
}

TEST(ArbiterCore, LimitsAreRejectionBoundsNotClamps)
{
  const Limits lim{1.0, 1.5};
  Rejection why = Rejection::kNone;
  EXPECT_TRUE(helix_arbiter_cpp::validate_twist(Twist6{1.0, -1.0, 0, 0, 0, -1.5}, lim, &why));
  EXPECT_FALSE(
    helix_arbiter_cpp::validate_twist(
      Twist6{std::nextafter(1.0, 2.0), 0, 0, 0, 0, 0}, lim, &why));
  EXPECT_EQ(why, Rejection::kLinearOverLimit);
  EXPECT_FALSE(helix_arbiter_cpp::validate_twist(Twist6{0, 0, 0, 0, 0, 1.6}, lim, &why));
  EXPECT_EQ(why, Rejection::kAngularOverLimit);
  // Unactuated axes only need to be finite and are dropped from the output.
  const auto cmd = helix_arbiter_cpp::validate_twist(Twist6{0.1, 0, 9.0, 0.5, 0.5, 0.2}, lim);
  ASSERT_TRUE(cmd);
  EXPECT_EQ(*cmd, (Command{0.1, 0.0, 0.2}));
}

TEST(ArbiterCore, AcceptedZeroIsALiveSourceAndNegativeZeroIsNormalised)
{
  Rig r;
  r.hold(false);
  ASSERT_TRUE(r.arb.on_source(1, Twist6{-0.0, -0.0, 0, 0, 0, -0.0}, r.t));
  const auto d = r.step(0.01);
  EXPECT_EQ(d.reason, Reason::kSource);
  EXPECT_TRUE(d.command.is_zero());
  EXPECT_FALSE(std::signbit(d.command.vx));
  EXPECT_FALSE(std::signbit(d.command.wz));
}

TEST(ArbiterCore, HoldTransitionsAreCountedOnce)
{
  Rig r;
  r.hold(false);
  const auto base = r.arb.counters().hold_transitions;
  for (int i = 0; i < 20; ++i) {
    r.hold(true);
    r.src("nav", 0.3);
    EXPECT_EQ(r.step(0.02).command, kZero);
  }
  EXPECT_EQ(r.arb.counters().hold_transitions - base, 1u);
}

TEST(ArbiterCore, ResetForgetsHoldAndSources)
{
  Rig r;
  r.hold(false);
  r.src("nav", 0.3);
  r.arb.reset();
  EXPECT_EQ(r.arb.decide(r.t).reason, Reason::kMissing);
  r.hold(false);
  EXPECT_EQ(r.arb.decide(r.t).reason, Reason::kNoInput);
}

TEST(ArbiterCore, ConstructionRefusesUnsafeConfigurations)
{
  using V = std::vector<SourceSpec>;
  EXPECT_THROW(Arbiter(V{}, 0.5), std::invalid_argument);
  EXPECT_THROW(Arbiter(V{kNav, kNav}, 0.5), std::invalid_argument);
  EXPECT_THROW(Arbiter(V{{"", "/x", 1, 0.5}}, 0.5), std::invalid_argument);
  for (double bad : {0.0, -1.0, kNaN, kInf}) {
    EXPECT_THROW(Arbiter(V{{"x", "/x", 1, bad}}, 0.5), std::invalid_argument);
    EXPECT_THROW(Arbiter(V{kNav}, bad), std::invalid_argument);
  }
  for (double bad : {-0.1, kNaN, kInf}) {
    EXPECT_THROW(Arbiter(V{kNav}, 0.5, Limits{bad, 1.5}), std::invalid_argument);
    EXPECT_THROW(Arbiter(V{kNav}, 0.5, Limits{1.0, bad}), std::invalid_argument);
  }
  EXPECT_NO_THROW(Arbiter(V{kNav}, 0.5, Limits{0.0, 0.0}));
}
