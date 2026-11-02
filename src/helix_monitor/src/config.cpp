#include "helix_monitor/config.hpp"

#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace helix_monitor {
namespace {

using Kv = std::map<std::string, std::string>;

std::vector<double> nums(const std::string& s) {
  std::vector<double> out;
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ',')) out.push_back(std::stod(tok));
  return out;
}

struct Line {
  int no;
  Kv kv;
  double num(const std::string& k, double def) const {
    auto it = kv.find(k);
    return it == kv.end() ? def : std::stod(it->second);
  }
  std::vector<double> list(const std::string& k, std::size_t n = 0) const {
    auto it = kv.find(k);
    if (it == kv.end()) throw std::runtime_error("line " + std::to_string(no) + ": missing " + k);
    auto v = nums(it->second);
    if (n && v.size() != n)
      throw std::runtime_error("line " + std::to_string(no) + ": " + k + " needs " + std::to_string(n) + " values");
    return v;
  }
  std::vector<std::size_t> idx(const std::string& k, std::size_t n = 0) const {
    std::vector<std::size_t> out;
    for (double d : list(k, n)) out.push_back(static_cast<std::size_t>(d));
    return out;
  }
};

template <std::size_t N, typename T>
std::array<T, N> arr(const std::vector<T>& v) {
  std::array<T, N> a{};
  for (std::size_t i = 0; i < N; ++i) a[i] = v[i];
  return a;
}

}  // namespace

std::unique_ptr<Config> parse_config(const std::string& text) {
  auto cfg = std::make_unique<Config>();
  std::stringstream in(text);
  std::string raw;
  int no = 0;
  while (std::getline(in, raw)) {
    ++no;
    if (auto h = raw.find('#'); h != std::string::npos) raw.resize(h);
    std::stringstream ls(raw);
    std::vector<std::string> pos;
    Line L{no, {}};
    std::string tok;
    while (ls >> tok) {
      if (auto eq = tok.find('='); eq != std::string::npos) L.kv[tok.substr(0, eq)] = tok.substr(eq + 1);
      else pos.push_back(tok);
    }
    if (pos.empty()) continue;
    const std::string& kind = pos[0];
    auto need = [&](std::size_t n) {
      if (pos.size() < n) throw std::runtime_error("line " + std::to_string(no) + ": " + kind + " needs a stream name");
    };
    try {
      if (kind == "rate") {
        need(2);
        cfg->monitor.add(std::make_unique<RateCheck>(pos[1], RateCheck::Params{
            L.num("window_s", 1.0), L.num("min_hz", 0.0), L.num("max_hz", 1e9), L.num("timeout_s", 0.5), L.num("clear_s", 0.5)}));
      } else if (kind == "frozen") {
        need(2);
        cfg->monitor.add(std::make_unique<FrozenCheck>(pos[1], FrozenCheck::Params{
            L.num("frozen_s", 0.2), static_cast<std::size_t>(L.num("min_count", 3))}));
      } else if (kind == "stamp") {
        need(2);
        cfg->monitor.add(std::make_unique<StampCheck>(pos[1], StampCheck::Params{L.num("stall_s", 0.2)}));
      } else if (kind == "range") {
        need(2);
        RangeCheck::Params p;
        p.channels = L.idx("ch");
        p.lo = L.list("lo", p.channels.size());
        p.hi = L.list("hi", p.channels.size());
        p.hold_s = L.num("hold_s", 0.0);
        p.clear_s = L.num("clear_s", 0.5);
        cfg->monitor.add(std::make_unique<RangeCheck>(pos[1], p));
      } else if (kind == "residual") {
        need(3);
        ResidualCheck::Params p;
        p.a_channels = arr<3>(L.idx("a_ch", 3));
        p.b_channels = arr<3>(L.idx("b_ch", 3));
        if (L.kv.count("R")) p.R = arr<9>(L.list("R", 9));
        if (L.kv.count("d")) p.d = arr<3>(L.list("d", 3));
        p.tau_s = L.num("tau_s", p.tau_s);
        p.threshold = L.num("threshold", p.threshold);
        p.hold_s = L.num("hold_s", p.hold_s);
        p.clear_s = L.num("clear_s", p.clear_s);
        p.stale_s = L.num("stale_s", p.stale_s);
        cfg->monitor.add(std::make_unique<ResidualCheck>(pos[1], pos[2], p));
      } else if (kind == "group") {
        need(3);
        EvidenceGroup g;
        g.name = pos[1];
        g.stream = pos[2];
        g.channels = L.idx("ch");
        g.scale = L.kv.count("scale") ? L.list("scale", g.channels.size()) : std::vector<double>(g.channels.size(), 1.0);
        g.raw = L.num("raw", 1) != 0;
        g.use_max = L.num("max", 0) != 0;
        g.detrend = L.num("detrend", 0) != 0;
        g.quiet = L.num("quiet", 1.0);
        g.moving = L.num("moving", 3.0);
        cfg->groups.push_back(g);
      } else if (kind == "stop") {
        cfg->has_stop = true;
        cfg->stop = {L.num("window_s", 0.5), L.num("deadline_s", 2.0), L.num("stale_s", 0.2),
                     static_cast<int>(L.num("min_groups", 2)), L.num("require_raw", 1) != 0};
      } else {
        throw std::runtime_error("unknown directive '" + kind + "'");
      }
    } catch (const std::invalid_argument&) {
      throw std::runtime_error("line " + std::to_string(no) + ": bad number");
    } catch (const std::runtime_error& e) {
      const std::string m = e.what();
      throw std::runtime_error(m.rfind("line ", 0) == 0 ? m : "line " + std::to_string(no) + ": " + m);
    }
  }
  return cfg;
}

std::unique_ptr<Config> load_config(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open " + path);
  std::stringstream ss;
  ss << f.rdbuf();
  return parse_config(ss.str());
}

}  // namespace helix_monitor
