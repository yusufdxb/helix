#include "helix_healer/spec.hpp"

#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace helix_healer {

const NodeSpec* Spec::find(const std::string& name) const {
  for (const auto& n : nodes)
    if (n.name == name) return &n;
  return nullptr;
}

namespace {

[[noreturn]] void fail(int no, const std::string& msg) {
  throw std::runtime_error("spec line " + std::to_string(no) + ": " + msg);
}

double num(int no, const std::string& s) {
  try {
    std::size_t used = 0;
    double v = std::stod(s, &used);
    if (used != s.size()) fail(no, "bad number '" + s + "'");
    return v;
  } catch (const std::logic_error&) {
    fail(no, "bad number '" + s + "'");
  }
}

void set_param(HealerParams& p, int no, const std::string& k, const std::string& v) {
  const std::map<std::string, double*> d = {
      {"tick_hz", &p.tick_hz}, {"stale_periods", &p.stale_periods}, {"stale_min_s", &p.stale_min_s},
      {"degraded_ratio", &p.degraded_ratio}, {"rate_window_s", &p.rate_window_s}, {"confirm_s", &p.confirm_s},
      {"verify_s", &p.verify_s}, {"startup_grace_s", &p.startup_grace_s}, {"backoff_s", &p.backoff_s},
      {"backoff_max_s", &p.backoff_max_s}, {"window_s", &p.window_s}, {"loop_s", &p.loop_s},
      {"cpu_sustain_s", &p.cpu_sustain_s}, {"leak_horizon_s", &p.leak_horizon_s},
      {"leak_min_growth_mb", &p.leak_min_growth_mb}, {"victim_grace_s", &p.victim_grace_s},
      {"common_window_s", &p.common_window_s}, {"common_hold_s", &p.common_hold_s},
      {"lifecycle_timeout_s", &p.lifecycle_timeout_s}, {"term_grace_s", &p.term_grace_s},
      {"learn_s", &p.learn_s}, {"resource_period_s", &p.resource_period_s}, {"fast_repair", &p.fast_repair},
      {"core_wait_s", &p.core_wait_s},
      {"adaptive", &p.adaptive}, {"adaptive_floor_s", &p.adaptive_floor_s},
      {"adaptive_stale_mult", &p.adaptive_stale_mult}, {"adaptive_min_gaps", &p.adaptive_min_gaps},
      {"adaptive_min_s", &p.adaptive_min_s}, {"adaptive_tau_s", &p.adaptive_tau_s},
      {"adaptive_cusum_h_min", &p.adaptive_cusum_h_min}, {"adaptive_cusum_mult", &p.adaptive_cusum_mult},
      {"adaptive_confirm_s", &p.adaptive_confirm_s}};
  if (k == "max_attempts") p.max_attempts = static_cast<int>(num(no, v));
  else if (k == "loop_deaths") p.loop_deaths = static_cast<int>(num(no, v));
  else if (auto it = d.find(k); it != d.end()) *it->second = num(no, v);
  else fail(no, "unknown healer key '" + k + "'");
}

bool flag(int no, const std::string& v) {
  if (v == "1" || v == "true") return true;
  if (v == "0" || v == "false") return false;
  fail(no, "expected 0/1, got '" + v + "'");
}

}  // namespace

Spec parse_spec(const std::string& text) {
  Spec spec;
  std::stringstream in(text);
  std::string raw;
  int no = 0;
  auto node = [&](const std::string& name) -> NodeSpec& {
    for (auto& n : spec.nodes)
      if (n.name == name) return n;
    fail(no, "node '" + name + "' used before its 'node' line");
  };
  while (std::getline(in, raw)) {
    ++no;
    if (auto h = raw.find('#'); h != std::string::npos) raw.resize(h);
    std::stringstream ls(raw);
    std::vector<std::string> tok;
    for (std::string t; ls >> t;) tok.push_back(t);
    if (tok.empty()) continue;
    const std::string& kind = tok[0];
    if (kind == "healer") {
      for (std::size_t i = 1; i < tok.size(); ++i) {
        auto eq = tok[i].find('=');
        if (eq == std::string::npos) fail(no, "expected key=value, got '" + tok[i] + "'");
        set_param(spec.p, no, tok[i].substr(0, eq), tok[i].substr(eq + 1));
      }
    } else if (kind == "node") {
      if (tok.size() < 2) fail(no, "node needs a name");
      if (spec.find(tok[1])) fail(no, "duplicate node '" + tok[1] + "'");
      NodeSpec n;
      n.name = tok[1];
      for (std::size_t i = 2; i < tok.size(); ++i) {
        auto eq = tok[i].find('=');
        if (eq == std::string::npos) fail(no, "expected key=value, got '" + tok[i] + "'");
        const std::string k = tok[i].substr(0, eq), v = tok[i].substr(eq + 1);
        if (k == "restart") n.restart = flag(no, v);
        else if (k == "critical") n.critical = flag(no, v);
        else if (k == "lifecycle") n.lifecycle = flag(no, v);
        else if (k == "autostart") n.autostart = flag(no, v);
        else if (k == "act") n.act = flag(no, v);
        else if (k == "fallback") n.fallback = v;
        else if (k == "cpu_max") n.cpu_max = num(no, v);
        else if (k == "rss_max_mb") n.rss_max_mb = num(no, v);
        else if (k == "attach") n.attach = v;
        else if (k == "ros_node") n.ros_node = v;
        else fail(no, "unknown node key '" + k + "'");
      }
      spec.nodes.push_back(std::move(n));
    } else if (kind == "out") {
      if (tok.size() < 3 || tok.size() > 4) fail(no, "out NAME TOPIC [HZ]");
      node(tok[1]).outputs.push_back({tok[2], tok.size() == 4 ? num(no, tok[3]) : 0.0});
    } else if (kind == "in") {
      if (tok.size() != 3) fail(no, "in NAME TOPIC");
      node(tok[1]).inputs.push_back(tok[2]);
    } else if (kind == "exec") {
      if (tok.size() < 3) fail(no, "exec NAME ARGV...");
      node(tok[1]).argv.assign(tok.begin() + 2, tok.end());
    } else if (kind == "then") {
      if (tok.size() < 3) fail(no, "then NAME ARGV...");
      node(tok[1]).then.emplace_back(tok.begin() + 2, tok.end());
    } else {
      fail(no, "unknown directive '" + kind + "'");
    }
  }
  for (const auto& n : spec.nodes) {
    if (n.argv.empty() && n.attach.empty())
      throw std::runtime_error("spec: node '" + n.name + "' has no exec line and no attach= pattern");
    if (!n.fallback.empty()) {
      const NodeSpec* f = spec.find(n.fallback);
      if (!f) throw std::runtime_error("spec: node '" + n.name + "' falls back to unknown '" + n.fallback + "'");
      if (f->autostart) throw std::runtime_error("spec: fallback '" + n.fallback + "' must have autostart=0");
    }
  }
  if (spec.p.tick_hz <= 0 || spec.p.max_attempts < 1 || spec.p.loop_deaths < 1)
    throw std::runtime_error("spec: tick_hz, max_attempts and loop_deaths must be positive");
  return spec;
}

Spec load_spec(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open spec " + path);
  std::stringstream ss;
  ss << f.rdbuf();
  return parse_spec(ss.str());
}

}  // namespace helix_healer
