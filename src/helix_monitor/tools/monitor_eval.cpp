// Offline driver for the HELIX monitor: replays timestamped samples from stdin, prints JSONL.
//
// Input (one per line, time-ordered):
//   S,<stream>,<t>,<stamp>,<v0>,<v1>,...   sample
//   C,stop,<t>                             stop commanded
//   T,<t>                                  clock tick with no sample
//   R,<id>                                 start a new independent episode (fresh state)
// Output: one JSON object per alarm transition and per stop verdict.
//   --stats          also print the worst statistic per check at end of input (calibration)
//   --trace <dt>     also print evidence-group metrics every dt seconds (calibration)
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include "helix_monitor/config.hpp"

using namespace helix_monitor;

namespace {

void num(double x) {
  if (std::isfinite(x)) std::printf("%.6g", x);
  else std::printf("null");
}

void print_events(const std::vector<Event>& ev) {
  for (const auto& e : ev) {
    std::printf("{\"t\":%.6f,\"check\":\"%s\",\"stream\":\"%s\",\"raised\":%s,\"value\":", e.t, e.check.c_str(),
                e.stream.c_str(), e.raised ? "true" : "false");
    num(e.value);
    std::printf("}\n");
  }
}

void print_metrics(const StopVerifier& sv, const std::vector<double>& m) {
  std::printf("{");
  for (std::size_t g = 0; g < m.size(); ++g) {
    std::printf("%s\"%s\":", g ? "," : "", sv.groups()[g].name.c_str());
    num(m[g]);
  }
  std::printf("}");
}

}  // namespace

int main(int argc, char** argv) {
  std::string cfg_path;
  bool stats = false;
  double trace_dt = 0.0;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--stats")) stats = true;
    else if (!std::strcmp(argv[i], "--trace") && i + 1 < argc) trace_dt = std::atof(argv[++i]);
    else cfg_path = argv[i];
  }
  if (cfg_path.empty()) {
    std::fprintf(stderr, "usage: monitor_eval <config> [--stats] [--trace dt] < samples\n");
    return 2;
  }
  std::unique_ptr<Config> cfg;
  try {
    cfg = load_config(cfg_path);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "config error: %s\n", e.what());
    return 2;
  }
  auto sv = std::make_unique<StopVerifier>(cfg->groups, cfg->stop);

  std::string line;
  std::vector<Event> ev;
  Sample s;
  double next_trace = -1.0;
  std::ios::sync_with_stdio(false);
  while (std::getline(std::cin, line)) {
    if (line.size() < 3) continue;
    double t = 0.0;
    const char kind = line[0];
    const char* p = line.c_str() + 2;
    ev.clear();
    if (kind == 'R') {
      cfg = load_config(cfg_path);
      sv = std::make_unique<StopVerifier>(cfg->groups, cfg->stop);
      next_trace = -1.0;
      std::printf("{\"episode\":\"%s\"}\n", p);
      continue;
    }
    if (kind == 'S') {
      const char* comma = std::strchr(p, ',');
      if (!comma) continue;
      const std::string stream(p, comma);
      char* end = nullptr;
      s.t = std::strtod(comma + 1, &end);
      s.stamp = std::strtod(end + 1, &end);
      s.v.clear();
      while (*end == ',') s.v.push_back(std::strtod(end + 1, &end));
      t = s.t;
      cfg->monitor.on_sample(stream, s, ev);
      sv->on_sample(stream, s);
    } else if (kind == 'C') {
      const char* comma = std::strchr(p, ',');
      if (!comma) continue;
      t = std::strtod(comma + 1, nullptr);
      sv->command(t);
    } else if (kind == 'T') {
      t = std::strtod(p, nullptr);
    } else {
      continue;
    }
    cfg->monitor.tick(t, ev);
    print_events(ev);
    StopResult r;
    if (sv->tick(t, r)) {
      std::printf("{\"stop\":\"%s\",\"t_cmd\":%.6f,\"t\":%.6f,\"metrics\":", to_string(r.verdict), r.t_cmd, r.t);
      print_metrics(*sv, r.metric);
      std::printf("}\n");
    }
    if (trace_dt > 0.0 && !sv->groups().empty() && t >= next_trace) {
      std::vector<double> m(sv->groups().size());
      for (std::size_t g = 0; g < m.size(); ++g) m[g] = sv->metric(g, t);
      std::printf("{\"trace\":%.6f,\"metrics\":", t);
      print_metrics(*sv, m);
      std::printf("}\n");
      next_trace = t + trace_dt;
    }
  }
  if (stats) {
    for (const auto& c : cfg->monitor.checks()) {
      std::printf("{\"stat\":\"%s\",\"stream\":\"%s\",\"max\":", c->name().c_str(), c->stream().c_str());
      num(c->statistic());
      if (auto* r = dynamic_cast<const RateCheck*>(c.get())) {
        std::printf(",\"min_rate\":");
        num(r->min_rate());
        std::printf(",\"max_rate\":");
        num(r->max_rate());
        std::printf(",\"max_gap\":");
        num(r->max_gap());
      }
      std::printf("}\n");
    }
  }
  return 0;
}
