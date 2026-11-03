#include <gtest/gtest.h>

#include <cmath>
#include <functional>

#include "helix_healer/policy.hpp"

using namespace helix_healer;

namespace {

// a (driver) -> b (filter); c is a lifecycle node; a falls back to a_b.
const char* kSpec = R"(
healer confirm_s=0.3 verify_s=2 startup_grace_s=1 max_attempts=3 victim_grace_s=1 cpu_sustain_s=1
node a fallback=a_b
out a /a 20
exec a x
node a_b autostart=0
out a_b /a 20
exec a_b x
node b
in b /a
out b /b 20
exec b x
node c lifecycle=1 critical=0
out c /c 20
exec c x
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

struct World {
  NodeObs a = healthy("a", "/a"), b = healthy("b", "/b"), c = healthy("c", "/c");
  World() { b.upstream = {"a"}; }
  Snapshot at(double t) const { return {t, {a, b, c}}; }
};

struct Trace {
  std::vector<Decision> decisions;
  std::vector<Event> events;
  int count(const std::string& node, Action a) const {
    int k = 0;
    for (const auto& d : decisions) k += d.node == node && d.action == a;
    return k;
  }
  bool has(const std::string& node, const std::string& kind) const {
    for (const auto& e : events)
      if (e.node == node && e.kind == kind) return true;
    return false;
  }
};

// Step the policy from t0 to t1. `world` mutates the observation at each t; `ok` says
// whether an action succeeds (results are delivered on the next step).
void run(Policy& p, World& w, double t0, double t1, Trace& tr, const std::function<void(World&, double)>& world,
         const std::function<bool(const Decision&)>& ok = [](const Decision&) { return true; }) {
  for (double t = t0; t < t1; t += 0.05) {
    world(w, t);
    auto out = p.step(w.at(t));
    for (auto& e : out.events) tr.events.push_back(e);
    for (auto& d : out.decisions) {
      tr.decisions.push_back(d);
      p.on_result(d.node, d.action, ok(d), t);
    }
  }
}

}  // namespace

TEST(Policy, CrashIsRestartedImmediatelyAndVerified) {
  Policy p(PolicyKind::Helix, parse_spec(kSpec));
  World w;
  Trace tr;
  run(p, w, 0, 1, tr, [](World&, double) {});
  EXPECT_TRUE(tr.decisions.empty());
  bool restarted = false;
  run(p, w, 1, 4, tr, [&](World& w, double t) {
    restarted = !tr.decisions.empty();
    w.a.alive = restarted;
    w.a.down_s = restarted ? 0 : t - 1;
    w.a.up_s = restarted ? 0.5 : 0;
  });
  ASSERT_EQ(tr.decisions.size(), 1u);
  EXPECT_EQ(tr.decisions[0].fault, Fault::Crash);
  EXPECT_EQ(tr.decisions[0].action, Action::Restart);
  EXPECT_TRUE(tr.has("a", "verified"));
  EXPECT_EQ(tr.count("b", Action::Restart), 0);  // b's stale output is a's fault, not b's
}

TEST(Policy, VerificationWaitsForAFreshMessage) {
  // a is respawned but the new process never publishes /a: alive is not fixed.
  Policy p(PolicyKind::Helix, parse_spec(kSpec));
  World w;
  Trace tr;
  run(p, w, 0, 1, tr, [](World&, double) {});
  double up_at = -1;
  run(p, w, 1, 4, tr, [&](World& w, double t) {
    if (up_at < 0 && !tr.decisions.empty()) up_at = t;
    w.a.alive = up_at >= 0;
    w.a.down_s = up_at >= 0 ? 0 : t - 1;
    w.a.up_s = up_at >= 0 ? t - up_at : 0;
    w.a.outputs[0].age_s = t - 0.98;  // last message just before the crash
  });
  EXPECT_FALSE(tr.has("a", "verified"));
  EXPECT_TRUE(tr.has("a", "verify_failed"));
  for (const auto& e : tr.events)
    if (e.kind == "verify_failed") {
      EXPECT_EQ(e.detail, "still_silent");
      EXPECT_LE(e.t - up_at, 1.2);  // startup grace 1 s, not the 2 s verify window
      break;  // the policy then repairs again; only the first verdict is under test
    }
}

TEST(Policy, CrashLoopGoesToFallback) {
  Policy p(PolicyKind::Helix, parse_spec(kSpec));
  World w;
  Trace tr;
  run(p, w, 0, 2, tr, [](World& w, double t) {
    if (t > 0.5) w.a.alive = false, w.a.quick_deaths = 2, w.a.down_s = 0.1;
  });
  ASSERT_FALSE(tr.decisions.empty());
  EXPECT_EQ(tr.decisions[0].fault, Fault::CrashLoop);
  EXPECT_EQ(tr.decisions[0].action, Action::Fallback);
  EXPECT_TRUE(tr.has("a", "fallback"));
  EXPECT_EQ(p.phase("a"), "retired");
}

TEST(Policy, HangNeedsConfirmationThenRestarts) {
  Policy p(PolicyKind::Helix, parse_spec(kSpec));
  World w;
  Trace tr;
  // stale for less than confirm_s past the stale limit: no action
  run(p, w, 0, 0.28, tr, [](World& w, double t) { w.a.outputs[0].age_s = 0.25 + t; });
  EXPECT_TRUE(tr.decisions.empty());
  run(p, w, 0.28, 1.0, tr, [](World& w, double t) { w.a.outputs[0].age_s = 0.25 + t; });
  ASSERT_EQ(tr.count("a", Action::Restart), 1);
  EXPECT_EQ(tr.decisions[0].fault, Fault::Hang);
}

TEST(Policy, FlickeringSlowOutputStillConfirms) {
  // A slowed output whose age crosses the stale limit every period: the diagnosis alternates
  // Hang and Degraded, each run shorter than the confirmation. It is one symptom and must confirm.
  Policy p(PolicyKind::Helix,
           parse_spec("healer confirm_s=0.3 verify_s=5 startup_grace_s=3\nnode a\nout a /a 83\nexec a x\n"));
  NodeObs a = healthy("a", "/a");
  TopicObs& o = a.outputs[0];
  o.expected_hz = 83;
  o.rate_hz = 1 / 0.18;
  o.stale_limit_s = 0.09;
  o.deficit = 100;
  o.deficit_limit = 4;
  std::vector<Decision> ds;
  for (double t = 0; t < 10 && ds.empty(); t += 0.02) {
    a.outputs[0].age_s = std::fmod(t, 0.18);
    for (auto& d : p.step({t, {a}}).decisions) ds.push_back(d);
  }
  ASSERT_EQ(ds.size(), 1u);
  EXPECT_EQ(ds[0].node, "a");
  EXPECT_EQ(ds[0].action, Action::Restart);
}

TEST(Policy, MissingOwnEndpointIsEndpointLoss) {
  Policy p(PolicyKind::Helix, parse_spec(kSpec));
  World w;
  Trace tr;
  run(p, w, 0, 1, tr, [](World& w, double t) {
    w.c.outputs[0].age_s = 0.3 + t;
    w.c.outputs[0].own_endpoint = false;
  });
  ASSERT_FALSE(tr.decisions.empty());
  EXPECT_EQ(tr.decisions[0].fault, Fault::EndpointLoss);
  EXPECT_EQ(tr.decisions[0].action, Action::LifecycleReset);  // c is a lifecycle node
}

TEST(Policy, FailedLifecycleResetEscalatesToRestart) {
  Policy p(PolicyKind::Helix, parse_spec(kSpec));
  World w;
  Trace tr;
  run(
      p, w, 0, 1, tr,
      [](World& w, double t) {
        w.c.outputs[0].age_s = 0.3 + t;
        w.c.outputs[0].own_endpoint = false;
      },
      [](const Decision& d) { return d.action != Action::LifecycleReset; });
  ASSERT_GE(tr.decisions.size(), 2u);
  EXPECT_EQ(tr.decisions[0].action, Action::LifecycleReset);
  EXPECT_EQ(tr.decisions[1].action, Action::Restart);
  EXPECT_TRUE(tr.has("c", "escalate"));
}

TEST(Policy, HangOnLifecycleNodeRestartsDirectly) {
  // A stuck executor cannot answer lifecycle services, so a reset would only cost a timeout.
  Policy p(PolicyKind::Helix, parse_spec(kSpec));
  World w;
  Trace tr;
  run(p, w, 0, 1, tr, [](World& w, double t) { w.c.outputs[0].age_s = 0.3 + t; });
  ASSERT_FALSE(tr.decisions.empty());
  EXPECT_EQ(tr.decisions[0].fault, Fault::Hang);
  EXPECT_EQ(tr.decisions[0].action, Action::Restart);
}

TEST(Policy, DownstreamOfStaleUpstreamIsAVictim) {
  Policy p(PolicyKind::Helix, parse_spec(kSpec));
  World w;
  Trace tr;
  // a hangs at t=0; b's output goes stale 0.1 s later because its input stopped.
  run(
      p, w, 0, 1.5, tr,
      [](World& w, double t) {
        w.a.outputs[0].age_s = 0.02 + t;
        w.b.outputs[0].age_s = std::max(0.02, t - 0.1);
      },
      [](const Decision&) { return false; });  // keep a broken
  EXPECT_GE(tr.count("a", Action::Restart), 1);
  EXPECT_EQ(tr.count("b", Action::Restart), 0);
  EXPECT_TRUE(tr.has("b", "victim"));
}

TEST(Policy, WatchdogBaselineRestartsTheVictimToo) {
  Policy p(PolicyKind::Watchdog, parse_spec(kSpec));
  World w;
  Trace tr;
  run(
      p, w, 0, 1.5, tr,
      [](World& w, double t) {
        w.a.outputs[0].age_s = 0.02 + t;
        w.b.outputs[0].age_s = std::max(0.02, t - 0.1);
      },
      [](const Decision&) { return false; });
  EXPECT_GE(tr.count("a", Action::Restart), 1);
  EXPECT_GE(tr.count("b", Action::Restart), 1);
}

TEST(Policy, RespawnBaselineIgnoresHang) {
  Policy p(PolicyKind::Respawn, parse_spec(kSpec));
  World w;
  Trace tr;
  run(p, w, 0, 3, tr, [](World& w, double t) { w.a.outputs[0].age_s = 0.02 + t; });
  EXPECT_TRUE(tr.decisions.empty());
}

TEST(Policy, CpuRunawayIsTheRootAndStarvedNeighbourIsNot) {
  Policy p(PolicyKind::Helix, parse_spec(kSpec));
  World w;
  Trace tr;
  run(p, w, 0, 1.5, tr, [](World& w, double t) {
    w.a.cpu = 2.0;                                  // a spins
    w.c.outputs[0].age_s = std::max(0.02, t - 0.2);  // c starves on the shared cores
  });
  EXPECT_EQ(tr.count("a", Action::Restart), 1);
  EXPECT_EQ(tr.decisions.at(0).fault, Fault::CpuRunaway);
  EXPECT_EQ(tr.count("c", Action::LifecycleReset) + tr.count("c", Action::Restart), 0);
  EXPECT_TRUE(tr.has("c", "victim"));
}

TEST(Policy, QosMismatchSkipsRestartsAndFallsBack) {
  Policy p(PolicyKind::Helix, parse_spec(kSpec));
  World w;
  Trace tr;
  run(p, w, 0, 1, tr, [](World& w, double t) {
    w.b.qos_incompatible = true;
    w.b.outputs[0].age_s = 0.3 + t;
  });
  ASSERT_EQ(tr.decisions.size(), 1u);
  EXPECT_EQ(tr.decisions[0].fault, Fault::QosMismatch);
  EXPECT_EQ(tr.decisions[0].action, Action::Fallback);  // b is critical with no backup: safe stop
}

TEST(Policy, DegradedRateIsDetected) {
  Policy p(PolicyKind::Helix, parse_spec(kSpec));
  World w;
  Trace tr;
  run(p, w, 0, 1, tr, [](World& w, double) { w.c.outputs[0].rate_hz = 6; });
  ASSERT_EQ(tr.decisions.size(), 1u);
  EXPECT_EQ(tr.decisions[0].fault, Fault::Degraded);
}

TEST(Policy, SimultaneousStallOnUnrelatedNodesIsHeld) {
  Policy p(PolicyKind::Helix, parse_spec(kSpec));
  World w;
  Trace tr;
  // a and c share no data path. Both lose rate at the same moment for 1 s (a host or
  // middleware stall), then recover: neither is restarted.
  run(p, w, 0, 3, tr, [](World& w, double t) {
    const double hz = t < 1.0 ? 6 : 20;
    w.a.outputs[0].rate_hz = hz;
    w.c.outputs[0].rate_hz = hz;
  });
  EXPECT_TRUE(tr.decisions.empty());
  EXPECT_TRUE(tr.has("a", "hold"));
  EXPECT_TRUE(tr.has("c", "hold"));
}

TEST(Policy, CommonModeHoldExpires) {
  Policy p(PolicyKind::Helix, parse_spec(kSpec));
  World w;
  Trace tr;
  // The same shared symptom, but it never clears: after common_hold_s both are judged alone.
  auto slow = [](World& w, double) {
    w.a.outputs[0].rate_hz = 6;
    w.c.outputs[0].rate_hz = 6;
  };
  run(p, w, 0, 1.9, tr, slow);
  EXPECT_TRUE(tr.decisions.empty());
  run(p, w, 1.9, 3, tr, slow);
  EXPECT_GE(tr.count("a", Action::Restart), 1);
  EXPECT_GE(tr.count("c", Action::LifecycleReset) + tr.count("c", Action::Restart), 1);
}

TEST(Policy, LinkedNodesAreNotACommonMode) {
  Policy p(PolicyKind::Helix, parse_spec(kSpec));
  World w;
  Trace tr;
  // b reads a: a simultaneous symptom on both is a root and its victim, not a shared cause.
  run(p, w, 0, 1, tr, [](World& w, double) {
    w.a.outputs[0].rate_hz = 6;
    w.b.outputs[0].rate_hz = 6;
  });
  EXPECT_FALSE(tr.has("a", "hold"));
  EXPECT_EQ(tr.count("a", Action::Restart), 1);
  EXPECT_EQ(tr.count("b", Action::Restart), 0);
}

TEST(Policy, CommonModeHoldCanBeDisabled) {
  Policy p(PolicyKind::Helix, parse_spec(std::string(kSpec) + "healer common_hold_s=0\n"));
  World w;
  Trace tr;
  run(p, w, 0, 1, tr, [](World& w, double) {
    w.a.outputs[0].rate_hz = 6;
    w.c.outputs[0].rate_hz = 6;
  });
  EXPECT_EQ(tr.count("a", Action::Restart), 1);
}

TEST(Policy, LeakIsCaughtBeforeTheHardLimit) {
  Policy p(PolicyKind::Helix, parse_spec(kSpec));
  World w;
  Trace tr;
  run(p, w, 0, 1, tr, [](World& w, double) {
    w.a.rss_mb = 300;
    w.a.rss_slope = 30;  // (512 - 300) / 30 = 7 s to the limit
  });
  ASSERT_FALSE(tr.decisions.empty());
  EXPECT_EQ(tr.decisions[0].fault, Fault::MemoryLeak);
  Policy wd(PolicyKind::Watchdog, parse_spec(kSpec));
  World w2;
  Trace tr2;
  run(wd, w2, 0, 1, tr2, [](World& w, double) { w.a.rss_mb = 300, w.a.rss_slope = 30; });
  EXPECT_TRUE(tr2.decisions.empty());  // the watchdog only acts at the hard limit
}

TEST(Policy, BudgetExhaustionFallsBack) {
  Policy p(PolicyKind::Helix, parse_spec(kSpec));
  World w;
  Trace tr;
  // a crashes again right after every restart (but not quickly enough to be a crash loop)
  run(p, w, 0, 30, tr, [](World& w, double) {
    w.a.alive = false;
    w.a.down_s = 0.1;
  });
  EXPECT_EQ(tr.count("a", Action::Restart), 3);
  EXPECT_EQ(tr.count("a", Action::Fallback), 1);
}

TEST(Policy, NoneNeverActs) {
  Policy p(PolicyKind::None, parse_spec(kSpec));
  World w;
  Trace tr;
  run(p, w, 0, 3, tr, [](World& w, double) { w.a.alive = false; });
  EXPECT_TRUE(tr.decisions.empty());
}

namespace {
// a hangs over [1, 2.5) and recovers by itself at 2.5; b stops with it. From 2.5 b publishes at
// `rate` while a deficit run up during the outage (50 periods, over its limit of 4) drains at
// `drain` periods/s. Returns the number of restarts of b.
int downstream_after_recovery(double rate, double drain) {
  Policy p(PolicyKind::Helix, parse_spec(kSpec));
  World w;
  Trace tr;
  run(
      p, w, 0, 12, tr,
      [&](World& w, double t) {
        const bool out = t >= 1 && t < 2.5;
        w.a.outputs[0].age_s = out ? 0.02 + (t - 1) : 0.02;
        w.b.outputs[0].age_s = out ? std::max(0.02, t - 1.1) : 0.02;
        w.b.outputs[0].deficit_limit = 4;
        w.b.outputs[0].stale_limit_s = 0.15;
        w.b.outputs[0].deficit = t < 2.5 ? 0 : std::max(0.0, 50 - drain * (t - 2.5));
        w.b.outputs[0].rate_hz = t < 2.5 ? 20 : rate;
      },
      [](const Decision&) { return false; });  // a's restart "fails": it recovers on its own
  return tr.count("b", Action::Restart);
}
}  // namespace

// The deficit b ran up while its upstream was out drains for 50/15 = 3.3 s after a recovers,
// longer than victim_grace_s = 1. b's rate is back, so it is not blamed meanwhile.
TEST(Policy, DrainingDeficitAfterUpstreamRecoveryIsNotBlamed) { EXPECT_EQ(downstream_after_recovery(20, 15), 0); }

// Control: the same deficit with b still at half rate is a real slowdown of b, blamed after the grace.
TEST(Policy, SlowDownstreamAfterUpstreamRecoveryIsStillBlamed) { EXPECT_GE(downstream_after_recovery(10, 15), 1); }

// Bound: a deficit that has not drained 4 x victim_grace_s after recovery is blamed even at full rate.
TEST(Policy, DrainGraceIsBounded) { EXPECT_GE(downstream_after_recovery(20, 0), 1); }

// Observe-only (act=0 / observe_only): the healer reports every decision, including the give-up
// Fallback, as a successful no-op (Action::None). The node must stay watched, never retired, decisions
// must not repeat faster than the verification window, and an outside recovery reads as verified.
TEST(Policy, ObserveNoOpKeepsNodeWatchedAndSeesOutsideRecovery) {
  Policy p(PolicyKind::Helix, parse_spec(kSpec));
  World w;
  Trace tr;
  int decided = 0;
  auto observe = [&](double t0, double t1, const std::function<void(World&, double)>& world) {
    for (double t = t0; t < t1; t += 0.05) {
      world(w, t);
      auto out = p.step(w.at(t));
      for (auto& e : out.events) tr.events.push_back(e);
      for (auto& d : out.decisions) {
        tr.decisions.push_back(d);
        ++decided;
        p.on_result(d.node, Action::None, true, t);
      }
    }
  };
  observe(0, 1, [](World&, double) {});
  observe(1, 11, [](World& w, double t) {  // a is dead for 10 s; nothing restarts it
    w.a.alive = false;
    w.a.down_s = t - 1;
    w.a.up_s = 0;
    w.a.outputs[0].age_s = t - 0.98;
  });
  EXPECT_GE(decided, 1);
  EXPECT_LE(decided, 6);  // at most one per verify_s (2 s) after the first
  EXPECT_NE(p.phase("a"), "retired");
  EXPECT_FALSE(tr.has("a", "fallback"));
  EXPECT_FALSE(tr.has("a", "verified"));
  const double back = 11;
  observe(11, 14, [&](World& w, double t) {  // an outside respawn (launcher, operator) brings it back
    w.a.alive = true;
    w.a.down_s = 0;
    w.a.up_s = t - back;
    w.a.outputs[0].age_s = 0.02;
  });
  EXPECT_TRUE(tr.has("a", "verified"));
  EXPECT_NE(p.phase("a"), "retired");
}
