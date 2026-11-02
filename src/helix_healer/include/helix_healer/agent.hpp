// Repair rankers for the agent policy. The policy proposes the feasible, untried repairs in
// rule order; a ranker only reorders them. Rankers never add actions, so every arm shares the
// same closed action set, budget and verifier.
#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include "helix_healer/policy.hpp"

namespace helix_healer {

// Structured evidence for one incident decision, collected deterministically by the healer.
struct AgentEvidence {
  Fault fault = Fault::None;
  bool alive = true;
  int quick_deaths = 0;
  int param_diff = 0;
  bool config_changed = false;
  bool config_ran = false;
  int log_errors = 0;
  bool lifecycle = false;
  int incident_action = 1;  // 1-based index of this action within the incident
  std::string log_tail;     // last lines of the node's log, read only by the LLM ranker
};

// Coarse evidence key for the repair history: fault plus the discrete evidence flags.
std::string signature(const AgentEvidence& e);
// Fixed-length numeric features (kFeatures) for the MLP.
constexpr int kFeatures = 20;
std::vector<float> features(const AgentEvidence& e);
// Repair actions a ranker may order, in a fixed index order for one-hot encodings.
const std::vector<Action>& repair_actions();
int action_index(Action a);  // -1 for a non-repair action

class Ranker {
 public:
  virtual ~Ranker() = default;
  virtual const char* name() const = 0;
  // Returns a permutation of cands. cands is non-empty and in rule order.
  virtual std::vector<Action> rank(const std::string& node, const AgentEvidence& e, const std::vector<Action>& cands) = 0;
  // Outcome of an executed repair (true = verified). Default: ignored.
  virtual void record(const std::string& /*node*/, const AgentEvidence& /*e*/, Action /*a*/, bool /*ok*/) {}
  // Extra text for the act event after the last rank() call (for example model timings).
  virtual std::string detail() const { return {}; }
};

// The hand-written rule order produced by Policy::decide_agent, unchanged.
class RuleRanker : public Ranker {
 public:
  const char* name() const override { return "rule"; }
  std::vector<Action> rank(const std::string&, const AgentEvidence&, const std::vector<Action>& c) override { return c; }
};

// Sham: uniformly random order, seeded.
class RandomRanker : public Ranker {
 public:
  explicit RandomRanker(uint64_t seed) : rng_(seed) {}
  const char* name() const override { return "random"; }
  std::vector<Action> rank(const std::string&, const AgentEvidence&, const std::vector<Action>& c) override;

 private:
  std::mt19937_64 rng_;
};

// Ceiling: "node:action,node:action" puts that node's correct action first, rest in rule order.
class OracleRanker : public Ranker {
 public:
  explicit OracleRanker(const std::string& spec);
  const char* name() const override { return "oracle"; }
  std::vector<Action> rank(const std::string& node, const AgentEvidence&, const std::vector<Action>& c) override;

 private:
  std::map<std::string, Action> best_;
};

// Repair-history posterior: Beta(1+s, 1+f) mean per (signature, action), backing off to
// (fault, action) when the signature has no data, untried actions scored kUntried. Ties keep
// rule order. Outcomes append to a plain-text log ("sig fault action ok" per line) and are read
// back at start, so the ranker learns online across episodes.
class HistoryRanker : public Ranker {
 public:
  static constexpr double kUntried = 0.6;
  explicit HistoryRanker(std::string path, bool append = true);
  const char* name() const override { return "history"; }
  std::vector<Action> rank(const std::string&, const AgentEvidence& e, const std::vector<Action>& c) override;
  void record(const std::string&, const AgentEvidence& e, Action a, bool ok) override;
  double score(const AgentEvidence& e, Action a) const;
  size_t size() const { return n_; }

 private:
  struct Count { int s = 0, f = 0; };
  void add(const std::string& sig, const std::string& fault, Action a, bool ok);
  std::string path_;
  bool append_;
  std::map<std::pair<std::string, int>, Count> by_sig_, by_fault_;
  size_t n_ = 0;
  mutable std::mutex mu_;
};

// Two-layer MLP over [features, action one-hot] -> repair-success logit. Weights come from a
// plain-text file written by the training script: "in hidden" then W1 (hidden x in), b1,
// W2 (hidden), b2, whitespace separated, row major.
class MlpRanker : public Ranker {
 public:
  explicit MlpRanker(const std::string& path);
  const char* name() const override { return "mlp"; }
  std::vector<Action> rank(const std::string&, const AgentEvidence& e, const std::vector<Action>& c) override;
  float logit(const AgentEvidence& e, Action a) const;

 private:
  int in_ = 0, hid_ = 0;
  std::vector<float> w1_, b1_, w2_;
  float b2_ = 0;
};

// Asks a local Ollama model for the first action, constrained by a JSON schema to an enum over
// cands; the rest keep rule order. arg is "model" or "model@host:port" (default 127.0.0.1:11434).
// No connection, timeout (timeout_s) or an answer outside cands throws, and the healer then uses
// the rule order and logs ranker_error.
class LlmRanker : public Ranker {
 public:
  explicit LlmRanker(const std::string& arg, double timeout_s = 5.0);
  const char* name() const override { return "llm"; }
  std::vector<Action> rank(const std::string& node, const AgentEvidence& e, const std::vector<Action>& c) override;
  std::string detail() const override { return detail_; }
  // Prompt and request body, exposed for tests.
  std::string prompt(const std::string& node, const AgentEvidence& e, const std::vector<Action>& c) const;
  std::string request(const std::string& node, const AgentEvidence& e, const std::vector<Action>& c) const;
  // Extracts the chosen action from an Ollama /api/chat response body; Action::None if absent.
  static Action parse(const std::string& body, const std::vector<Action>& c);

 private:
  std::string model_, host_ = "127.0.0.1";
  int port_ = 11434;
  double timeout_s_;
  std::string detail_;
};

// Stable sort of cands by descending score; equal scores keep the incoming (rule) order.
std::vector<Action> order_by(const std::vector<Action>& cands, const std::vector<double>& score);

// name: rule | random | oracle | history | mlp | llm. arg: oracle spec, history path, weights
// path or model.
std::unique_ptr<Ranker> make_ranker(const std::string& name, const std::string& arg, uint64_t seed);

}  // namespace helix_healer
