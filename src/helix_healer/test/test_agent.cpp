#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <functional>

#include "helix_healer/agent.hpp"
#include "helix_healer/policy.hpp"

using namespace helix_healer;

namespace {

const char* kSpec = R"(
healer confirm_s=0.3 verify_s=2 startup_grace_s=1 max_attempts=5 victim_grace_s=1 cpu_sustain_s=1
node a
out a /a 20
exec a x
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
  NodeObs a = healthy("a", "/a"), c = healthy("c", "/c");
  Snapshot at(double t) const { return {t, {a, c}}; }
};

struct Trace {
  std::vector<Decision> d;
  std::vector<Event> ev;
  int kinds(const std::string& node, const std::string& kind) const {
    int k = 0;
    for (const auto& e : ev) k += e.node == node && e.kind == kind;
    return k;
  }
};

// The healer contract for the agent: a Decision with candidates is ranked by the healer, which
// runs one action and reports it through on_result. `pick` plays the ranker.
void run(Policy& p, World& w, double t0, double t1, Trace& tr, const std::function<void(World&, double)>& world,
         const std::function<bool(Action)>& ok,
         const std::function<Action(const Decision&)>& pick = [](const Decision& d) { return d.action; }) {
  for (double t = t0; t < t1; t += 0.05) {
    world(w, t);
    auto out = p.step(w.at(t));
    for (auto& e : out.events) tr.ev.push_back(e);
    for (auto& d : out.decisions) {
      tr.d.push_back(d);
      const Action a = d.candidates.empty() ? d.action : pick(d);
      p.on_result(d.node, a, ok(a), t);
    }
  }
}

}  // namespace

TEST(Agent, ParamDiffPutsParamRestoreFirstAndOpensAnIncident) {
  Policy p(PolicyKind::Agent, parse_spec(kSpec));
  World w;
  Trace tr;
  run(p, w, 0, 1, tr, [](World&, double) {}, [](Action) { return true; });
  run(
      p, w, 1, 3, tr,
      [&](World& w, double) {
        w.a.outputs[0].rate_hz = tr.d.empty() ? 2 : 20;
        w.a.ev.param_diff = tr.d.empty() ? 1 : 0;
      },
      [](Action) { return true; });
  ASSERT_FALSE(tr.d.empty());
  EXPECT_EQ(tr.d[0].fault, Fault::Degraded);
  ASSERT_EQ(tr.d[0].candidates.size(), 2u);
  EXPECT_EQ(tr.d[0].candidates[0], Action::ParamRestore);
  EXPECT_EQ(tr.d[0].candidates[1], Action::Restart);
  EXPECT_EQ(tr.kinds("a", "incident"), 1);
  EXPECT_EQ(tr.kinds("a", "act"), 0);  // the healer logs "act" after ranking
}

TEST(Agent, ConfigTheProcessLoadedIsRolledBackBeforeAPlainRestart) {
  Policy p(PolicyKind::Agent, parse_spec(kSpec));
  World w;
  Trace tr;
  run(p, w, 0, 1, tr, [](World&, double) {}, [](Action) { return true; });
  run(
      p, w, 1, 2, tr,
      [](World& w, double) {
        w.a.alive = false;
        w.a.down_s = 0.1;
        w.a.ev.has_lkg_config = w.a.ev.config_changed = w.a.ev.config_ran = true;
      },
      [](Action) { return true; });
  ASSERT_FALSE(tr.d.empty());
  EXPECT_EQ(tr.d[0].fault, Fault::Crash);
  ASSERT_EQ(tr.d[0].candidates.size(), 2u);
  EXPECT_EQ(tr.d[0].candidates[0], Action::RestartLKG);
  EXPECT_EQ(tr.d[0].candidates[1], Action::Restart);
}

TEST(Agent, ChangedButUnloadedConfigIsTriedAsDeployedFirst) {
  Policy p(PolicyKind::Agent, parse_spec(kSpec));
  World w;
  Trace tr;
  run(p, w, 0, 1, tr, [](World&, double) {}, [](Action) { return true; });
  run(
      p, w, 1, 2, tr,
      [](World& w, double) {
        w.a.alive = false;
        w.a.down_s = 0.1;
        w.a.ev.has_lkg_config = w.a.ev.config_changed = true;
      },
      [](Action) { return true; });
  ASSERT_FALSE(tr.d.empty());
  ASSERT_EQ(tr.d[0].candidates.size(), 2u);
  EXPECT_EQ(tr.d[0].candidates[0], Action::Restart);
  EXPECT_EQ(tr.d[0].candidates[1], Action::RestartLKG);
}

TEST(Agent, FailedOneShotActionsAreNotRetriedAndTheIncidentIsBounded) {
  Policy p(PolicyKind::Agent, parse_spec(kSpec));
  World w;
  Trace tr;
  run(p, w, 0, 1, tr, [](World&, double) {}, [](Action) { return true; });
  // c is a degraded lifecycle node with a parameter diff; nothing ever fixes it.
  run(
      p, w, 1, 40, tr,
      [](World& w, double) {
        w.c.outputs[0].rate_hz = 2;
        w.c.ev.param_diff = 1;
      },
      [](Action) { return false; });
  int repairs = 0, param = 0, reset = 0;
  for (const auto& d : tr.d) {
    if (d.node != "c" || d.candidates.empty()) continue;
    ++repairs;
    param += d.action == Action::ParamRestore;
    reset += d.action == Action::LifecycleReset;
  }
  EXPECT_EQ(repairs, Policy::kAgentK);
  EXPECT_EQ(param, 1);
  EXPECT_EQ(reset, 1);
  EXPECT_GE(tr.kinds("c", "given_up"), 1);
  EXPECT_EQ(tr.kinds("c", "escalate"), 0);
}

TEST(Agent, RankedChoiceIsWhatTheIncidentRemembers) {
  // The ranker runs Restart instead of the rule's ParamRestore; the failed Restart may repeat,
  // but ParamRestore must stay available for the next decision.
  Policy p(PolicyKind::Agent, parse_spec(kSpec));
  World w;
  Trace tr;
  run(p, w, 0, 1, tr, [](World&, double) {}, [](Action) { return true; });
  bool first = true;
  run(
      p, w, 1, 12, tr,
      [](World& w, double) {
        w.a.outputs[0].rate_hz = 2;
        w.a.ev.param_diff = 1;
      },
      [](Action) { return false; },
      [&](const Decision& d) {
        Action a = first ? Action::Restart : d.action;
        first = false;
        return a;
      });
  std::vector<Decision> ad;
  for (const auto& d : tr.d)
    if (d.node == "a" && !d.candidates.empty()) ad.push_back(d);
  ASSERT_GE(ad.size(), 2u);
  EXPECT_EQ(ad[1].candidates[0], Action::ParamRestore);
}

TEST(Agent, QosMismatchGivesUpWithoutActing) {
  Policy p(PolicyKind::Agent, parse_spec(kSpec));
  World w;
  Trace tr;
  run(p, w, 0, 1, tr, [](World&, double) {}, [](Action) { return true; });
  run(
      p, w, 1, 3, tr,
      [](World& w, double t) {
        w.a.outputs[0].age_s = t;
        w.a.qos_incompatible = true;
      },
      [](Action) { return true; });
  ASSERT_FALSE(tr.d.empty());
  EXPECT_TRUE(tr.d[0].candidates.empty());
  EXPECT_EQ(tr.kinds("a", "incident"), 0);
}

TEST(Agent, HelixIgnoresTheNewEvidence) {
  Policy p(PolicyKind::Helix, parse_spec(kSpec));
  World w;
  Trace tr;
  run(p, w, 0, 1, tr, [](World&, double) {}, [](Action) { return true; });
  run(
      p, w, 1, 3, tr,
      [&](World& w, double) {
        w.a.outputs[0].rate_hz = tr.d.empty() ? 2 : 20;
        w.a.ev.param_diff = 1;
      },
      [](Action) { return true; });
  ASSERT_FALSE(tr.d.empty());
  EXPECT_TRUE(tr.d[0].candidates.empty());
  EXPECT_EQ(tr.d[0].action, Action::Restart);
}

TEST(Ranker, ActionNamesRoundTrip) {
  for (Action a : repair_actions()) EXPECT_EQ(action_from(to_string(a)), a);
  EXPECT_EQ(action_from("rm -rf"), Action::None);
}

TEST(Ranker, OrderByIsStableOnTies) {
  std::vector<Action> c = {Action::ParamRestore, Action::Restart, Action::LifecycleReset};
  auto o = order_by(c, {0.5, 0.9, 0.5});
  EXPECT_EQ(o, (std::vector<Action>{Action::Restart, Action::ParamRestore, Action::LifecycleReset}));
}

TEST(Ranker, RandomIsAPermutationAndSeeded) {
  std::vector<Action> c = {Action::ParamRestore, Action::Restart, Action::LifecycleReset, Action::RestartLKG};
  RandomRanker r1(7), r2(7);
  AgentEvidence e;
  for (int i = 0; i < 5; ++i) {
    auto a = r1.rank("x", e, c), b = r2.rank("x", e, c);
    EXPECT_EQ(a, b);
    auto s = a;
    std::sort(s.begin(), s.end());
    auto cs = c;
    std::sort(cs.begin(), cs.end());
    EXPECT_EQ(s, cs);
  }
}

TEST(Ranker, OraclePutsTheNamedActionFirstOnlyForItsNode) {
  OracleRanker o("rsp:restart_lkg,controller_manager:param_restore");
  std::vector<Action> c = {Action::Restart, Action::RestartLKG};
  AgentEvidence e;
  EXPECT_EQ(o.rank("rsp", e, c)[0], Action::RestartLKG);
  EXPECT_EQ(o.rank("other", e, c)[0], Action::Restart);
  EXPECT_THROW(OracleRanker("rsp:shell"), std::runtime_error);
}

TEST(Ranker, HistoryLearnsOnlineAndPersists) {
  const std::string path = ::testing::TempDir() + "helix_history_test.txt";
  std::remove(path.c_str());
  AgentEvidence e;
  e.fault = Fault::Crash;
  e.alive = false;
  e.config_changed = e.config_ran = true;
  std::vector<Action> c = {Action::Restart, Action::RestartLKG};
  {
    HistoryRanker h(path);
    EXPECT_EQ(h.rank("n", e, c)[0], Action::Restart);  // both untried: rule order
    h.record("n", e, Action::Restart, false);
    h.record("n", e, Action::Restart, false);
    EXPECT_EQ(h.rank("n", e, c)[0], Action::RestartLKG);  // untried 0.6 beats 1/4
  }
  HistoryRanker h2(path);  // reload from disk
  EXPECT_EQ(h2.size(), 2u);
  EXPECT_EQ(h2.rank("n", e, c)[0], Action::RestartLKG);
  // A different signature with the same fault backs off to the fault-level counts.
  AgentEvidence e2 = e;
  e2.config_ran = false;
  EXPECT_NEAR(h2.score(e2, Action::Restart), 0.25, 1e-9);
  std::remove(path.c_str());
}

TEST(Ranker, MlpLoadsWeightsAndScoresByAction) {
  const std::string path = ::testing::TempDir() + "helix_mlp_test.txt";
  const int in = kFeatures + static_cast<int>(repair_actions().size());
  {
    // One hidden unit that fires only for the RestartLKG one-hot input.
    std::ofstream f(path);
    f << in << " 1\n";
    for (int i = 0; i < in; ++i) f << (i == kFeatures + action_index(Action::RestartLKG) ? 1 : 0) << ' ';
    f << "\n0\n2\n-1\n";
  }
  MlpRanker m(path);
  AgentEvidence e;
  EXPECT_NEAR(m.logit(e, Action::RestartLKG), 1.0, 1e-6);
  EXPECT_NEAR(m.logit(e, Action::Restart), -1.0, 1e-6);
  EXPECT_EQ(m.rank("n", e, {Action::Restart, Action::RestartLKG})[0], Action::RestartLKG);
  {
    std::ofstream f(path);
    f << "3 1\n0 0 0\n";
  }
  EXPECT_THROW(MlpRanker bad(path), std::runtime_error);
  std::remove(path.c_str());
}

TEST(Ranker, FeaturesHaveFixedLength) {
  AgentEvidence e;
  EXPECT_EQ(features(e).size(), static_cast<size_t>(kFeatures));
  e.fault = Fault::CommonMode;
  e.log_errors = 100000;
  for (float x : features(e)) {
    EXPECT_GE(x, 0.f);
    EXPECT_LE(x, 1.f);
  }
}
