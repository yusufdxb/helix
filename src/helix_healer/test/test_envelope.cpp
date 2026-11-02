#include <gtest/gtest.h>

#include <random>

#include "helix_healer/envelope.hpp"
#include "helix_healer/policy.hpp"

using namespace helix_healer;

namespace {

Envelope trained(double hz, double secs, double t0 = 0) {
  Envelope e;
  for (double t = t0; t <= t0 + secs + 1e-9; t += 1.0 / hz) e.arrival(t, true);
  return e;
}

}  // namespace

TEST(Envelope, NotReadyUntilEnoughHealthyGaps) {
  Envelope e;
  for (int i = 0; i < 50; ++i) e.arrival(i * 0.05, true);  // 49 gaps, 2.45 s
  EXPECT_FALSE(e.ready());
  EXPECT_EQ(e.stale_limit(), 0.0);
  EXPECT_EQ(e.deficit_limit(), 0.0);
  e.arrival(50 * 0.05, true);
  EXPECT_TRUE(e.ready());
}

TEST(Envelope, StaleLimitTracksTheLearnedGap) {
  Envelope e = trained(20, 5);
  EXPECT_NEAR(e.mean_gap(), 0.05, 1e-6);
  EXPECT_NEAR(e.stale_limit(), 0.15, 1e-6);  // 3 x max gap, above the 0.1 s floor
  Envelope fast = trained(500, 5);
  EXPECT_NEAR(fast.stale_limit(), 0.1, 1e-9);  // floor
}

TEST(Envelope, UnlearnedArrivalsDoNotMoveTheBaseline) {
  Envelope e = trained(20, 5);
  const double mean = e.mean_gap();
  const double last = 5.0;
  for (int i = 1; i <= 20; ++i) e.arrival(last + i * 0.5, false);  // a sick 2 Hz producer
  EXPECT_NEAR(e.mean_gap(), mean, 1e-12);
}

TEST(Envelope, CusumCatchesARateDropWithinAFewGaps) {
  Envelope e = trained(20, 5);
  double t = 5.0, hit = -1;
  for (int i = 0; i < 20 && hit < 0; ++i) {
    t += 0.15;  // 20 Hz -> 6.7 Hz
    e.arrival(t, false);
    if (e.deficit(t) > e.deficit_limit()) hit = t - 5.0;
  }
  ASSERT_GT(hit, 0);
  EXPECT_LE(hit, 0.5);
}

TEST(Envelope, OpenGapCountsBeforeTheNextMessage) {
  Envelope e = trained(20, 5);
  // 0.05 s mean, k = 1.5: an open gap of 0.3 s already contributes 4.5 > h = 4
  EXPECT_LT(e.deficit(5.0 + 0.2), e.deficit_limit());
  EXPECT_GT(e.deficit(5.0 + 0.3), e.deficit_limit());
}

TEST(Envelope, MildSlowdownAboveTheDegradedRatioIsIgnored) {
  Envelope e = trained(20, 5);
  double t = 5.0;
  for (int i = 0; i < 200; ++i) {
    t += 1.0 / 15.0;  // 75% of nominal; degraded_ratio is 50%
    e.arrival(t, false);
    ASSERT_LT(e.deficit(t), e.deficit_limit()) << "at gap " << i;
  }
}

TEST(Envelope, NoAlarmOnHumanoidJointStateJitter) {
  // Timing seen in a public Unitree G1 bag: 500 Hz, 0.1 ms jitter, rare 6 ms gaps.
  std::mt19937 rng(7);
  std::normal_distribution<double> gap(0.002, 0.0001);
  std::uniform_real_distribution<double> u(0, 1);
  Envelope e;
  double t = 0;
  int alarms = 0;
  for (int i = 0; i < 60 * 500; ++i) {
    t += u(rng) < 0.002 ? 0.006 : std::max(0.0005, gap(rng));
    e.arrival(t, true);
    if (e.ready() && (e.deficit(t) > e.deficit_limit())) ++alarms;
  }
  EXPECT_EQ(alarms, 0);
}

TEST(Envelope, RespawnForgetsTheOpenGap) {
  Envelope e = trained(20, 5);
  e.reset_process();
  EXPECT_EQ(e.deficit(100.0), 0.0);
  EXPECT_TRUE(e.ready());  // the baseline survives a respawn
}

namespace {

const char* kSpec = R"(
healer confirm_s=0.3 verify_s=2 startup_grace_s=1 max_attempts=3 victim_grace_s=1
node a
out a /a 20
exec a x
node b
in b /a
out b /b 20
exec b x
)";

NodeObs healthy(const std::string& name, const std::string& topic) {
  NodeObs n;
  n.name = name;
  n.alive = true;
  n.up_s = 100;
  n.rss_mb = 50;
  n.rss_base = 50;
  n.outputs.push_back({topic, 20, 0.02, 20, true});
  return n;
}

// Steps the policy at 100 Hz; returns the first decision for `node` and its time.
std::pair<double, Decision> first_decision(Policy& p, double t1, const std::function<void(NodeObs&, NodeObs&, double)>& f,
                                           const std::string& node = "a") {
  NodeObs a = healthy("a", "/a"), b = healthy("b", "/b");
  b.upstream = {"a"};
  for (double t = 0; t < t1; t += 0.01) {
    f(a, b, t);
    for (auto& d : p.step({t, {a, b}}).decisions) {
      p.on_result(d.node, d.action, false, t);
      if (d.node == node) return {t, d};
    }
  }
  return {-1, {}};
}

}  // namespace

TEST(Adaptive, EnvelopeLimitCutsHangDetectionTime) {
  Policy p(PolicyKind::Helix, parse_spec(kSpec));
  auto [t, d] = first_decision(p, 2, [](NodeObs& a, NodeObs&, double t) {
    a.outputs[0].age_s = 0.02 + t;
    a.outputs[0].stale_limit_s = 0.15;
  });
  ASSERT_GE(t, 0);
  EXPECT_EQ(d.fault, Fault::Hang);
  EXPECT_EQ(d.action, Action::Restart);
  EXPECT_LE(t, 0.3);  // stale at 0.13 s, sharp confirm 0.1 s
  EXPECT_EQ(d.delay_s, 0.0);
}

TEST(Adaptive, FixedRuleWithoutEnvelope) {
  Policy p(PolicyKind::Helix, parse_spec(kSpec));
  auto [t, d] = first_decision(p, 2, [](NodeObs& a, NodeObs&, double t) { a.outputs[0].age_s = 0.02 + t; });
  ASSERT_GE(t, 0);
  EXPECT_GE(t, 0.5);  // fixed limit 0.25 s + confirm 0.3 s
}

TEST(Adaptive, WatchdogIgnoresTheEnvelope) {
  Policy p(PolicyKind::Watchdog, parse_spec(kSpec));
  auto [t, d] = first_decision(p, 2, [](NodeObs& a, NodeObs&, double t) {
    a.outputs[0].age_s = 0.02 + t;
    a.outputs[0].stale_limit_s = 0.15;
  });
  ASSERT_GE(t, 0);
  EXPECT_GE(t, 0.5);
}

TEST(Adaptive, AdaptiveOffRestoresTheV3Rule) {
  Policy p(PolicyKind::Helix, parse_spec(std::string(kSpec) + "healer adaptive=0\n"));
  auto [t, d] = first_decision(p, 2, [](NodeObs& a, NodeObs&, double t) {
    a.outputs[0].age_s = 0.02 + t;
    a.outputs[0].stale_limit_s = 0.15;
  });
  ASSERT_GE(t, 0);
  EXPECT_GE(t, 0.5);
}

TEST(Adaptive, DeficitFlagsDegradeBeforeTheRateWindow) {
  Policy p(PolicyKind::Helix, parse_spec(kSpec));
  auto [t, d] = first_decision(p, 2, [](NodeObs& a, NodeObs&, double t) {
    a.outputs[0].deficit = t > 0.2 ? 5 : 0;  // rate window still reads nominal
    a.outputs[0].deficit_limit = 4;
  });
  ASSERT_GE(t, 0);
  EXPECT_EQ(d.fault, Fault::Degraded);
  EXPECT_LE(t, 0.35);
}

TEST(Adaptive, StarvedConsumerIsAVictimEvenBeforeUpstreamIsJudged) {
  // a goes silent first but has a long fixed limit; b's adaptive limit trips first.
  Policy p(PolicyKind::Helix, parse_spec(kSpec));
  auto [t, d] = first_decision(
      p, 0.6,
      [](NodeObs& a, NodeObs& b, double t) {
        a.outputs[0].age_s = 0.02 + t;
        b.outputs[0].age_s = std::max(0.02, t - 0.03);
        b.outputs[0].stale_limit_s = 0.1;
      },
      "b");
  EXPECT_LT(t, 0) << "b was repaired as " << to_string(d.fault);
}

TEST(FastRepair, FirstRestartHasNoDelayInEveryPolicy) {
  for (auto k : {PolicyKind::Respawn, PolicyKind::Watchdog, PolicyKind::Helix}) {
    Policy p(k, parse_spec(kSpec));
    auto [t, d] = first_decision(p, 1, [](NodeObs& a, NodeObs&, double t) {
      if (t > 0.1) a.alive = false, a.down_s = t - 0.1;
    });
    ASSERT_GE(t, 0);
    EXPECT_EQ(d.action, Action::Restart);
    EXPECT_EQ(d.delay_s, 0.0);
  }
}

TEST(FastRepair, OffKeepsTheV3Backoff) {
  Policy p(PolicyKind::Respawn, parse_spec(std::string(kSpec) + "healer fast_repair=0\n"));
  auto [t, d] = first_decision(p, 1, [](NodeObs& a, NodeObs&, double t) {
    if (t > 0.1) a.alive = false, a.down_s = t - 0.1;
  });
  ASSERT_GE(t, 0);
  EXPECT_EQ(d.delay_s, 0.5);
}

namespace {

// Two crashes of `a`: the first at 0.5 s, the second `second_qd` quick deaths later at 2.5 s
// (after the first repair was verified). Returns the delay of the second restart, -1 if none.
double second_restart_delay(const std::string& spec, int second_qd) {
  Policy p(PolicyKind::Helix, parse_spec(spec));
  NodeObs a = healthy("a", "/a"), b = healthy("b", "/b");
  b.upstream = {"a"};
  int n = 0;
  double up_at = -1;
  for (double t = 0; t < 4; t += 0.01) {
    const bool dead = (t >= 0.5 && up_at < 0) || (t >= 2.5 && n < 2);
    a.alive = !dead;
    a.quick_deaths = dead && t >= 2.5 ? second_qd : 0;
    a.down_s = dead ? 0.01 : 0;
    a.outputs[0].age_s = dead ? 0.5 : 0.02;
    for (auto& d : p.step({t, {a, b}}).decisions) {
      if (d.node != "a") continue;
      if (++n == 2) return d.delay_s;
      up_at = t;
      p.on_result(d.node, d.action, true, t);
    }
  }
  return -1;
}

}  // namespace

TEST(FastRepair, OnlyAYoungDeathWithoutABackupWaits) {
  const std::string backed = std::string(kSpec) + "node a_b autostart=0\nout a_b /a 20\nexec a_b x\n";
  std::string with_backup = backed;
  with_backup.replace(with_backup.find("node a\n"), 7, "node a fallback=a_b\n");
  EXPECT_EQ(second_restart_delay(kSpec, 1), 1.0);         // young death, no backup: backoff(1)
  EXPECT_EQ(second_restart_delay(kSpec, 0), 0.0);         // ran past loop_s: no reason to wait
  EXPECT_EQ(second_restart_delay(with_backup, 1), 0.0);   // the crash-loop detector bounds it
}
