#include <gtest/gtest.h>

#include <cmath>

#include "helix_monitor/config.hpp"

using namespace helix_monitor;

namespace {
Sample S(double t, std::vector<double> v, double stamp = -1.0) { return {t, stamp < 0 ? t : stamp, std::move(v)}; }
int raises(const std::vector<Event>& ev, const std::string& check) {
  int n = 0;
  for (const auto& e : ev) n += (e.check == check && e.raised);
  return n;
}
}  // namespace

TEST(Latch, HoldAndClear) {
  Latch l(0.2, 0.5);
  EXPECT_EQ(l.update(0.0, true), 0);
  EXPECT_EQ(l.update(0.1, true), 0);
  EXPECT_EQ(l.update(0.2, true), 1);
  EXPECT_EQ(l.update(0.3, false), 0);
  EXPECT_EQ(l.update(0.5, true), 0);    // interruption restarts the clear timer
  EXPECT_EQ(l.update(0.6, false), 0);
  EXPECT_EQ(l.update(1.1, false), -1);
}

TEST(RateCheck, TimeoutAndBand) {
  RateCheck c("x", {1.0, 90.0, 110.0, 0.1, 0.5});
  std::vector<Event> ev;
  for (int i = 0; i < 200; ++i) c.on_sample("x", S(i * 0.01, {0}), ev);
  EXPECT_TRUE(ev.empty());
  c.tick(1.99 + 0.15, ev);               // silence longer than timeout
  EXPECT_EQ(raises(ev, "rate"), 1);
  ev.clear();
  RateCheck half("x", {1.0, 90.0, 110.0, 1.0, 0.5});
  for (int i = 0; i < 300; ++i) half.on_sample("x", S(i * 0.02, {0}), ev);  // 50 Hz
  EXPECT_EQ(raises(ev, "rate"), 1);
}

TEST(FrozenCheck, IdenticalPayload) {
  FrozenCheck c("x", {0.05, 3});
  std::vector<Event> ev;
  for (int i = 0; i < 20; ++i) c.on_sample("x", S(i * 0.004, {double(i), 1.0}), ev);
  EXPECT_TRUE(ev.empty());
  for (int i = 20; i < 50; ++i) c.on_sample("x", S(i * 0.004, {7.0, 1.0}), ev);
  EXPECT_EQ(raises(ev, "frozen"), 1);
  EXPECT_NEAR(ev.front().t, 20 * 0.004 + 0.05, 0.005);
}

TEST(StampCheck, StallWhileArriving) {
  StampCheck c("x", {0.05});
  std::vector<Event> ev;
  for (int i = 0; i < 20; ++i) c.on_sample("x", S(i * 0.004, {0}), ev);
  for (int i = 20; i < 50; ++i) c.on_sample("x", S(i * 0.004, {0}, 19 * 0.004), ev);
  EXPECT_EQ(raises(ev, "stamp"), 1);
}

TEST(RangeCheck, BoundsAndNaN) {
  RangeCheck c("x", {{1}, {-1.0}, {1.0}, 0.0, 0.1});
  std::vector<Event> ev;
  c.on_sample("x", S(0.0, {99.0, 0.5}), ev);   // channel 0 not monitored
  EXPECT_TRUE(ev.empty());
  c.on_sample("x", S(0.01, {0.0, NAN}), ev);
  EXPECT_EQ(raises(ev, "range"), 1);
}

TEST(ResidualCheck, DetectsBiasNotRotation) {
  ResidualCheck::Params p;
  // b is a rotated by 90 deg about z: a = R * b with R mapping (bx,by) -> (-by,bx).
  p.R = {0, -1, 0, 1, 0, 0, 0, 0, 1};
  p.tau_s = 0.05;
  p.threshold = 0.1;
  p.hold_s = 0.1;
  ResidualCheck c("a", "b", p);
  std::vector<Event> ev;
  double t = 0.0;
  auto step = [&](double bias) {
    const double w = std::sin(3.0 * t);
    const double bx = 0.5 * w, by = -0.3 * w, bz = 0.2;
    c.on_sample("b", S(t, {bx, by, bz + bias}), ev);
    c.on_sample("a", S(t + 0.001, {-by, bx, bz}), ev);
    t += 0.004;
  };
  for (int i = 0; i < 1000; ++i) step(0.0);
  EXPECT_TRUE(ev.empty()) << "clean rotated pair must not alarm";
  EXPECT_LT(c.statistic(), 0.05);
  for (int i = 0; i < 200; ++i) step(0.3);
  ASSERT_EQ(raises(ev, "residual"), 1);
  for (int i = 0; i < 500; ++i) step(0.0);
  EXPECT_FALSE(ev.back().raised) << "residual must clear once the fault goes away";
}

TEST(ResidualCheck, StaleSideDoesNotAlarm) {
  ResidualCheck::Params p;
  p.threshold = 0.1;
  ResidualCheck c("a", "b", p);
  std::vector<Event> ev;
  c.on_sample("b", S(0.0, {5, 5, 5}), ev);
  for (int i = 1; i < 200; ++i) c.on_sample("a", S(i * 0.004, {0, 0, 0}), ev);
  EXPECT_TRUE(ev.empty());
}

namespace {
StopVerifier make_sv(int min_groups = 2, bool require_raw = true) {
  std::vector<EvidenceGroup> g(3);
  g[0] = {"estimator", "est", {0}, {1.0}, false, true, false, 0.05, 0.2};
  g[1] = {"joints", "enc", {0}, {1.0}, true, false, false, 0.1, 0.5};
  g[2] = {"inertial", "imu", {0}, {1.0}, true, false, false, 0.1, 0.5};
  return StopVerifier(g, {0.5, 2.0, 0.2, min_groups, require_raw});
}
// Feeds the three groups at 100 Hz with the given amplitudes, ticks, returns the resolved verdict.
Verdict run(StopVerifier& sv, double est, double enc, double imu, bool drop_raw = false) {
  sv.command(1.0);
  StopResult r;
  for (int i = 0; i < 400; ++i) {
    const double t = i * 0.01, sgn = (i % 2) ? 1.0 : -1.0;
    const bool after = t >= 1.0;
    sv.on_sample("est", S(t, {after ? est : 1.0}));
    if (!drop_raw) {
      sv.on_sample("enc", S(t, {sgn * (after ? enc : 1.0)}));
      sv.on_sample("imu", S(t, {sgn * (after ? imu : 1.0)}));
    }
    if (sv.tick(t, r)) return r.verdict;
  }
  return sv.state();
}
}  // namespace

TEST(StopVerifier, GenuineStopVerified) {
  auto sv = make_sv();
  EXPECT_EQ(run(sv, 0.0, 0.01, 0.01), Verdict::kVerified);
}

TEST(StopVerifier, EstimatorLiesFails) {
  auto sv = make_sv();
  EXPECT_EQ(run(sv, 0.0, 1.0, 1.0), Verdict::kFailed);
}

TEST(StopVerifier, EstimatorOnlyIsUnverified) {
  auto sv = make_sv();
  EXPECT_EQ(run(sv, 0.0, 0, 0, /*drop_raw=*/true), Verdict::kUnverified);
  auto single = make_sv(1, false);    // B-est mode: one estimator group suffices
  EXPECT_EQ(run(single, 0.0, 0, 0, true), Verdict::kVerified);
}

TEST(StopVerifier, EvidenceMustPostdateCommand) {
  auto sv = make_sv();
  sv.command(1.0);
  StopResult r;
  bool resolved = false;
  for (int i = 0; i < 200; ++i) {
    const double t = 0.5 + i * 0.01;     // robot already still before the command
    sv.on_sample("est", S(t, {0.0}));
    sv.on_sample("enc", S(t, {0.0}));
    sv.on_sample("imu", S(t, {0.0}));
    if ((resolved = sv.tick(t, r))) break;
  }
  ASSERT_TRUE(resolved);
  EXPECT_EQ(r.verdict, Verdict::kVerified);
  EXPECT_GE(r.t, 1.5 - 1e-9);
}

TEST(Config, ParsesAndRejects) {
  auto c = parse_config(
      "rate imu min_hz=200 max_hz=300 timeout_s=0.1\n"
      "frozen imu frozen_s=0.05  # comment\n"
      "residual low imu a_ch=0,1,2 b_ch=0,1,2 threshold=0.2\n"
      "group joints low ch=0,1 scale=1,1 quiet=0.1 moving=0.5\n"
      "stop deadline_s=2.0\n");
  EXPECT_EQ(c->monitor.checks().size(), 3u);
  EXPECT_EQ(c->groups.size(), 1u);
  EXPECT_TRUE(c->has_stop);
  EXPECT_THROW(parse_config("bogus x\n"), std::runtime_error);
  EXPECT_THROW(parse_config("residual a b a_ch=0,1 b_ch=0,1,2\n"), std::runtime_error);
  EXPECT_THROW(parse_config("rate imu min_hz=abc\n"), std::runtime_error);
}
