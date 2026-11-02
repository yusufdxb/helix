#include "helix_healer/agent.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace helix_healer {

namespace {
const std::vector<Fault> kFaults = {Fault::Crash,       Fault::CrashLoop, Fault::Hang,     Fault::EndpointLoss,
                                    Fault::CpuRunaway,  Fault::MemoryLeak, Fault::Degraded, Fault::Upstream,
                                    Fault::CommonMode};
}

const std::vector<Action>& repair_actions() {
  static const std::vector<Action> a = {Action::Restart, Action::LifecycleReset, Action::ParamRestore, Action::RestartLKG};
  return a;
}

int action_index(Action a) {
  const auto& r = repair_actions();
  auto it = std::find(r.begin(), r.end(), a);
  return it == r.end() ? -1 : static_cast<int>(it - r.begin());
}

std::string signature(const AgentEvidence& e) {
  std::ostringstream o;
  o << to_string(e.fault) << "|a" << e.alive << "|q" << (e.quick_deaths > 0) << "|p" << (e.param_diff > 0) << "|c"
    << e.config_changed << "|r" << e.config_ran << "|l" << (e.log_errors > 0) << "|lc" << e.lifecycle;
  return o.str();
}

std::vector<float> features(const AgentEvidence& e) {
  std::vector<float> f(kFeatures, 0.f);
  int i = 0;
  for (Fault k : kFaults) f[i++] = e.fault == k ? 1.f : 0.f;  // 9
  f[i++] = e.alive;
  f[i++] = std::min(e.quick_deaths, 5) / 5.f;
  f[i++] = e.param_diff > 0;
  f[i++] = std::min(e.param_diff, 5) / 5.f;
  f[i++] = e.config_changed;
  f[i++] = e.config_ran;
  f[i++] = e.log_errors > 0;
  f[i++] = std::log1p(static_cast<float>(std::min(e.log_errors, 1000))) / std::log1p(1000.f);
  f[i++] = e.lifecycle;
  f[i++] = std::min(e.incident_action, 4) / 4.f;
  f[i++] = 1.f;  // bias-like constant so an all-zero fault one-hot is still distinguishable
  return f;
}

std::vector<Action> order_by(const std::vector<Action>& cands, const std::vector<double>& score) {
  std::vector<size_t> idx(cands.size());
  std::iota(idx.begin(), idx.end(), 0);
  std::stable_sort(idx.begin(), idx.end(), [&](size_t a, size_t b) { return score[a] > score[b]; });
  std::vector<Action> out;
  for (size_t i : idx) out.push_back(cands[i]);
  return out;
}

std::vector<Action> RandomRanker::rank(const std::string&, const AgentEvidence&, const std::vector<Action>& c) {
  std::vector<Action> out = c;
  std::shuffle(out.begin(), out.end(), rng_);
  return out;
}

OracleRanker::OracleRanker(const std::string& spec) {
  std::stringstream ss(spec);
  std::string item;
  while (std::getline(ss, item, ',')) {
    auto colon = item.find(':');
    if (colon == std::string::npos) continue;
    Action a = action_from(item.substr(colon + 1));
    if (a == Action::None) throw std::runtime_error("oracle: unknown action in '" + item + "'");
    best_[item.substr(0, colon)] = a;
  }
}

std::vector<Action> OracleRanker::rank(const std::string& node, const AgentEvidence&, const std::vector<Action>& c) {
  auto it = best_.find(node);
  if (it == best_.end()) return c;
  std::vector<double> s;
  for (Action a : c) s.push_back(a == it->second ? 1.0 : 0.0);
  return order_by(c, s);
}

HistoryRanker::HistoryRanker(std::string path, bool append) : path_(std::move(path)), append_(append) {
  if (path_.empty()) return;
  std::ifstream in(path_);
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream ls(line);
    std::string sig, fault, act;
    int ok = 0;
    if (!(ls >> sig >> fault >> act >> ok)) continue;
    Action a = action_from(act);
    if (action_index(a) < 0) continue;
    add(sig, fault, a, ok != 0);
  }
}

void HistoryRanker::add(const std::string& sig, const std::string& fault, Action a, bool ok) {
  auto& c1 = by_sig_[{sig, action_index(a)}];
  auto& c2 = by_fault_[{fault, action_index(a)}];
  (ok ? c1.s : c1.f)++;
  (ok ? c2.s : c2.f)++;
  ++n_;
}

double HistoryRanker::score(const AgentEvidence& e, Action a) const {
  std::lock_guard<std::mutex> lk(mu_);
  const int ai = action_index(a);
  auto beta = [](const Count& c) { return (1.0 + c.s) / (2.0 + c.s + c.f); };
  if (auto it = by_sig_.find({signature(e), ai}); it != by_sig_.end()) return beta(it->second);
  if (auto it = by_fault_.find({to_string(e.fault), ai}); it != by_fault_.end()) return beta(it->second);
  return kUntried;
}

std::vector<Action> HistoryRanker::rank(const std::string&, const AgentEvidence& e, const std::vector<Action>& c) {
  std::vector<double> s;
  for (Action a : c) s.push_back(score(e, a));
  return order_by(c, s);
}

void HistoryRanker::record(const std::string&, const AgentEvidence& e, Action a, bool ok) {
  if (action_index(a) < 0) return;
  std::lock_guard<std::mutex> lk(mu_);
  add(signature(e), to_string(e.fault), a, ok);
  if (append_ && !path_.empty()) {
    std::ofstream out(path_, std::ios::app);
    out << signature(e) << ' ' << to_string(e.fault) << ' ' << to_string(a) << ' ' << (ok ? 1 : 0) << '\n';
  }
}

MlpRanker::MlpRanker(const std::string& path) {
  std::ifstream in(path);
  if (!in || !(in >> in_ >> hid_)) throw std::runtime_error("mlp: cannot read weights '" + path + "'");
  const int want = kFeatures + static_cast<int>(repair_actions().size());
  if (in_ != want || hid_ <= 0 || hid_ > 1024)
    throw std::runtime_error("mlp: shape " + std::to_string(in_) + "x" + std::to_string(hid_) + ", expected input " +
                             std::to_string(want));
  auto read = [&](std::vector<float>& v, size_t n) {
    v.resize(n);
    for (auto& x : v)
      if (!(in >> x)) throw std::runtime_error("mlp: truncated weights '" + path + "'");
  };
  read(w1_, static_cast<size_t>(in_) * hid_);
  read(b1_, hid_);
  read(w2_, hid_);
  if (!(in >> b2_)) throw std::runtime_error("mlp: truncated weights '" + path + "'");
}

float MlpRanker::logit(const AgentEvidence& e, Action a) const {
  std::vector<float> x = features(e);
  for (size_t k = 0; k < repair_actions().size(); ++k) x.push_back(action_index(a) == static_cast<int>(k) ? 1.f : 0.f);
  float out = b2_;
  for (int h = 0; h < hid_; ++h) {
    float z = b1_[h];
    for (int i = 0; i < in_; ++i) z += w1_[static_cast<size_t>(h) * in_ + i] * x[i];
    out += w2_[h] * std::max(0.f, z);
  }
  return out;
}

std::vector<Action> MlpRanker::rank(const std::string&, const AgentEvidence& e, const std::vector<Action>& c) {
  std::vector<double> s;
  for (Action a : c) s.push_back(logit(e, a));
  return order_by(c, s);
}

std::unique_ptr<Ranker> make_ranker(const std::string& name, const std::string& arg, uint64_t seed) {
  if (name.empty() || name == "rule") return std::make_unique<RuleRanker>();
  if (name == "random") return std::make_unique<RandomRanker>(seed);
  if (name == "oracle") return std::make_unique<OracleRanker>(arg);
  if (name == "history") return std::make_unique<HistoryRanker>(arg);
  if (name == "mlp") return std::make_unique<MlpRanker>(arg);
  if (name == "llm") return std::make_unique<LlmRanker>(arg);
  throw std::runtime_error("unknown agent ranker '" + name + "' (rule|random|oracle|history|mlp|llm)");
}

}  // namespace helix_healer
