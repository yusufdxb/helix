// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// helix_anomaly_replay: drives AnomalyCore and the Python-compatible
// formatting helpers with a line protocol that tools/anomaly_replay.py also
// executes against the Python reference node's own methods. Identical input
// must give identical output text. No ROS involved.
//
// Usage:
//   helix_anomaly_replay < scenario.txt > results.txt
//   helix_anomaly_replay --bench scenario.txt [--repeat N] [--warmup N]
//
// Encoding as in helix_arbiter_replay: f64 = 16 hex digits of the IEEE-754
// bit pattern, str = "s:" + percent-encoded bytes, integers decimal.
// --bench times AnomalyCore::process for every sample line (in-process core
// cost only: no ROS, DDS, logging or message construction).

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "helix_sensing_cpp/anomaly_core.hpp"
#include "helix_sensing_cpp/pyfmt.hpp"
#include "helix_sensing_cpp/rolling_stats.hpp"

namespace
{

using helix_sensing_cpp::AnomalyCore;
using helix_sensing_cpp::AnomalyParams;

std::vector<std::string> split(const std::string & line)
{
  std::vector<std::string> out;
  std::istringstream is(line);
  std::string tok;
  while (is >> tok) {
    out.push_back(tok);
  }
  return out;
}

double f64(const std::string & tok)
{
  if (tok.size() != 16) {
    throw std::runtime_error("bad f64 token '" + tok + "'");
  }
  const std::uint64_t bits = std::strtoull(tok.c_str(), nullptr, 16);
  double d;
  std::memcpy(&d, &bits, sizeof(d));
  return d;
}

std::string f64(double d)
{
  std::uint64_t bits;
  std::memcpy(&bits, &d, sizeof(d));
  char buf[17];
  std::snprintf(buf, sizeof(buf), "%016" PRIx64, bits);
  return buf;
}

std::string dec(const std::string & tok)
{
  if (tok.rfind("s:", 0) != 0) {
    throw std::runtime_error("bad str token '" + tok + "'");
  }
  std::string out;
  for (std::size_t i = 2; i < tok.size(); ++i) {
    if (tok[i] == '%' && i + 2 < tok.size()) {
      out.push_back(static_cast<char>(std::strtol(tok.substr(i + 1, 2).c_str(), nullptr, 16)));
      i += 2;
    } else {
      out.push_back(tok[i]);
    }
  }
  return out;
}

std::string enc(const std::string & s)
{
  static const char * kSafe =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_./~{}-";
  std::string out = "s:";
  for (unsigned char c : s) {
    if (c != 0 && std::strchr(kSafe, c) != nullptr) {
      out.push_back(static_cast<char>(c));
    } else {
      char buf[4];
      std::snprintf(buf, sizeof(buf), "%%%02X", c);
      out += buf;
    }
  }
  return out;
}

class Session
{
public:
  std::string run(const std::vector<std::string> & t)
  {
    const std::string & cmd = t.at(0);
    if (cmd == "params") {
      AnomalyParams p;
      p.zscore_threshold = f64(t.at(1));
      p.consecutive_trigger = std::strtoll(t.at(2).c_str(), nullptr, 10);
      p.window_size = std::strtoll(t.at(3).c_str(), nullptr, 10);
      p.emit_cooldown_s = f64(t.at(4));
      p.min_anomaly_duration_s = f64(t.at(5));
      try {
        core_ = std::make_unique<AnomalyCore>(p);
        return "params ok";
      } catch (const std::invalid_argument &) {
        core_.reset();
        return "params error";
      }
    }
    if (cmd == "sample") {
      if (!core_) {
        throw std::runtime_error("no core configured");
      }
      const auto r = core_->process(dec(t.at(1)), f64(t.at(2)), f64(t.at(3)), f64(t.at(4)));
      if (!r.fault) {
        return "ok " + std::to_string(r.consecutive);
      }
      const auto & f = *r.fault;
      std::string out = "fault " + enc(f.node_name) + " " + enc(f.fault_type) + " " +
        std::to_string(static_cast<int>(f.severity)) + " " + enc(f.detail) + " " +
        f64(f.timestamp) + " " + std::to_string(f.context_keys.size());
      for (std::size_t i = 0; i < f.context_keys.size(); ++i) {
        out += " " + enc(f.context_keys[i]) + " " + enc(f.context_values[i]);
      }
      return out + " " + std::to_string(r.consecutive);
    }
    if (cmd == "square") {
      return "square " + f64(helix_sensing_cpp::python_square(f64(t.at(1))));
    }
    if (cmd == "parse") {
      const auto v = helix_sensing_cpp::parse_python_float(dec(t.at(1)));
      return v ? "parse " + f64(*v) : "parse none";
    }
    if (cmd == "repr") {
      return "repr " + enc(helix_sensing_cpp::python_repr(f64(t.at(1))));
    }
    if (cmd == "round") {
      return "round " + enc(
        helix_sensing_cpp::python_round_str(
          f64(t.at(1)), std::atoi(
            t.at(2).c_str())));
    }
    if (cmd == "fixed") {
      return "fixed " + enc(
        helix_sensing_cpp::python_fixed(
          f64(t.at(1)), std::atoi(
            t.at(2).c_str())));
    }
    throw std::runtime_error("unknown command '" + cmd + "'");
  }

  AnomalyCore & core()
  {
    if (!core_) {
      throw std::runtime_error("no core configured");
    }
    return *core_;
  }

private:
  std::unique_ptr<AnomalyCore> core_;
};

int replay(std::istream & in, std::ostream & out)
{
  Session s;
  std::string line;
  std::size_t lineno = 0;
  while (std::getline(in, line)) {
    ++lineno;
    const auto t = split(line);
    if (t.empty() || t[0][0] == '#') {
      continue;
    }
    try {
      out << s.run(t) << '\n';
    } catch (const std::exception & e) {
      std::cerr << "line " << lineno << ": " << e.what() << '\n';
      return 2;
    }
  }
  return 0;
}

std::string percentiles_json(std::vector<std::int64_t> v)
{
  if (v.empty()) {
    return "{\"count\": 0}";
  }
  std::sort(v.begin(), v.end());
  auto pct = [&v](double p) {
      std::size_t k = static_cast<std::size_t>(std::ceil(p / 100.0 * v.size()));
      k = std::max<std::size_t>(1, std::min(k, v.size()));
      return v[k - 1];
    };
  double sum = 0.0;
  for (auto x : v) {
    sum += static_cast<double>(x);
  }
  std::ostringstream os;
  os << "{\"count\": " << v.size() << ", \"p50_ns\": " << pct(50) << ", \"p95_ns\": " <<
    pct(95) << ", \"p99_ns\": " << pct(99) << ", \"max_ns\": " << v.back() <<
    ", \"mean_ns\": " << sum / static_cast<double>(v.size()) << "}";
  return os.str();
}

int bench(const std::string & path, int repeat, int warmup)
{
  std::ifstream f(path);
  if (!f) {
    std::cerr << "cannot open " << path << '\n';
    return 2;
  }
  struct Sample
  {
    std::string metric;
    double value, mono, wall;
  };
  std::vector<std::string> params_line;
  std::vector<Sample> samples;
  std::string line;
  while (std::getline(f, line)) {
    const auto t = split(line);
    if (t.empty() || t[0][0] == '#') {
      continue;
    }
    if (t[0] == "params") {
      params_line = t;
    } else if (t[0] == "sample") {
      samples.push_back({dec(t.at(1)), f64(t.at(2)), f64(t.at(3)), f64(t.at(4))});
    }
  }
  if (params_line.empty()) {
    std::cerr << "no params line\n";
    return 2;
  }
  using clock = std::chrono::steady_clock;
  std::uint64_t sink = 0;
  auto fresh = [&params_line]() {
      Session s;
      s.run(params_line);
      return s;
    };
  auto run_once = [&](AnomalyCore & core, std::vector<std::int64_t> * ns) {
      for (const Sample & smp : samples) {
        const auto t0 = ns ? clock::now() : clock::time_point{};
        const auto r = core.process(smp.metric, smp.value, smp.mono, smp.wall);
        sink += static_cast<std::uint64_t>(r.consecutive) + (r.fault ? 1 : 0);
        if (ns) {
          ns->push_back(
            std::chrono::duration_cast<std::chrono::nanoseconds>(clock::now() - t0).count());
        }
      }
    };
  for (int i = 0; i < warmup; ++i) {
    Session s = fresh();
    run_once(s.core(), nullptr);
  }
  std::vector<std::int64_t> ns;
  for (int i = 0; i < repeat; ++i) {
    Session s = fresh();
    run_once(s.core(), &ns);
  }
  const auto t0 = clock::now();
  for (int i = 0; i < repeat; ++i) {
    Session s = fresh();
    run_once(s.core(), nullptr);
  }
  const double elapsed = std::chrono::duration<double>(clock::now() - t0).count();
  std::vector<std::int64_t> overhead;
  for (int i = 0; i < 100000; ++i) {
    const auto a = clock::now();
    overhead.push_back(
      std::chrono::duration_cast<std::chrono::nanoseconds>(clock::now() - a).count());
  }
  std::cout << "{\"implementation\": \"cpp\", \"samples_per_replay\": " << samples.size() <<
    ", \"repeat\": " << repeat << ", \"warmup\": " << warmup <<
    ", \"throughput_samples_per_s\": " << static_cast<double>(samples.size()) * repeat / elapsed <<
    ", \"timer_overhead\": " << percentiles_json(overhead) <<
    ", \"process\": " << percentiles_json(ns) << ", \"checksum\": " << sink << "}\n";
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::ios::sync_with_stdio(false);
  if (argc >= 3 && std::string(argv[1]) == "--bench") {
    int repeat = 20;
    int warmup = 3;
    for (int i = 3; i + 1 < argc; i += 2) {
      const std::string flag = argv[i];
      if (flag == "--repeat") {
        repeat = std::max(1, std::atoi(argv[i + 1]));
      } else if (flag == "--warmup") {
        warmup = std::max(0, std::atoi(argv[i + 1]));
      }
    }
    return bench(argv[2], repeat, warmup);
  }
  if (argc != 1) {
    std::cerr << "usage: helix_anomaly_replay < scenario | --bench FILE [--repeat N] "
      "[--warmup N]\n";
    return 2;
  }
  return replay(std::cin, std::cout);
}
