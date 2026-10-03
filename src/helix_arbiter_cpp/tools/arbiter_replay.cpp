// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// helix_arbiter_replay: drives the native arbiter core with a line protocol
// that tools/arbiter_replay.py also executes against the Python reference
// (helix_arbiter.arbiter_core). Identical input gives identical output text
// when the two implementations agree, decision by decision. No ROS involved.
//
// Usage:
//   helix_arbiter_replay < scenario.txt > decisions.txt
//   helix_arbiter_replay --bench scenario.txt [--repeat N] [--warmup N]
//
// Encoding: f64 = 16 hex digits of the IEEE-754 bit pattern (exact, keeps
// -0.0 and NaN); str = "s:" + percent-encoded bytes; integers decimal;
// booleans 0/1. The protocol is documented in tools/arbiter_replay.py.
//
// --bench replays the scenario's runtime events (src, hold, decide, reset)
// against a freshly built arbiter, timing every core call with
// std::chrono::steady_clock, and prints one JSON object. In-process core cost
// only: no ROS, DDS, serialisation or scheduling latency is included.

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
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "helix_arbiter_cpp/arbiter_config.hpp"
#include "helix_arbiter_cpp/arbiter_core.hpp"

namespace
{

using helix_arbiter_cpp::Arbiter;
using helix_arbiter_cpp::ParamMap;
using helix_arbiter_cpp::ParamValue;

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

std::uint64_t u64(const std::string & tok)
{
  if (tok.empty() || tok[0] == '-') {
    throw std::runtime_error("bad u64 token '" + tok + "'");
  }
  return std::strtoull(tok.c_str(), nullptr, 10);
}

std::int64_t i64(const std::string & tok) {return std::strtoll(tok.c_str(), nullptr, 10);}

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

const char * rejection_token(helix_arbiter_cpp::Rejection why)
{
  switch (why) {
    case helix_arbiter_cpp::Rejection::kNonFinite: return "nonfinite";
    case helix_arbiter_cpp::Rejection::kLinearOverLimit: return "linear";
    case helix_arbiter_cpp::Rejection::kAngularOverLimit: return "angular";
    case helix_arbiter_cpp::Rejection::kNone: break;
  }
  return "none";
}

class Session
{
public:
  std::string run(const std::vector<std::string> & t)
  {
    const std::string & cmd = t.at(0);
    if (cmd == "new") {
      hold_timeout_ = f64(t.at(1));
      limits_ = helix_arbiter_cpp::Limits{f64(t.at(2)), f64(t.at(3))};
      specs_.clear();
      arb_.reset();
      return "new";
    }
    if (cmd == "spec") {
      specs_.push_back({dec(t.at(1)), "/unused", i64(t.at(2)), f64(t.at(3))});
      return "spec";
    }
    if (cmd == "build") {
      try {
        arb_ = std::make_unique<Arbiter>(specs_, hold_timeout_, limits_);
        return "build ok";
      } catch (const std::invalid_argument &) {
        arb_.reset();
        return "build error";
      }
    }
    if (cmd == "src") {
      const auto idx = arbiter().index_of(dec(t.at(1)));
      if (!idx) {
        return "src unknown";
      }
      helix_arbiter_cpp::Rejection why = helix_arbiter_cpp::Rejection::kNone;
      const helix_arbiter_cpp::Twist6 in{f64(t.at(2)), f64(t.at(3)), f64(t.at(4)),
        f64(t.at(5)), f64(t.at(6)), f64(t.at(7))};
      if (arbiter().on_source(*idx, in, f64(t.at(8)), &why)) {
        return "src 1";
      }
      return std::string("src 0 ") + rejection_token(why);
    }
    if (cmd == "hold") {
      const bool ok = arbiter().on_hold(
        t.at(1) == "1", dec(t.at(2)), u64(t.at(3)), u64(t.at(4)), f64(t.at(5)));
      return ok ? "hold 1" : "hold 0";
    }
    if (cmd == "decide") {
      const auto d = arbiter().decide(f64(t.at(1)));
      return std::string("decide ") + helix_arbiter_cpp::reason_name(d.reason) + " " +
             enc(d.source) + " " + enc(d.hold_fault_id) + " " +
             f64(d.command.vx) + " " + f64(d.command.vy) + " " + f64(d.command.wz) + " " +
             (d.helix_forced() ? "1" : "0");
    }
    if (cmd == "reset") {
      arbiter().reset();
      return "reset";
    }
    if (cmd == "counters") {
      const auto & c = arbiter().counters();
      std::string out = "counters " + std::to_string(c.rejected) + " " +
        std::to_string(c.hold_reordered) + " " + std::to_string(c.hold_transitions);
      for (std::size_t i = 0; i < arbiter().source_count(); ++i) {
        out += " " + enc(arbiter().spec(i).name) + "=" +
          std::to_string(arbiter().rejected_by_source(i));
      }
      return out;
    }
    if (cmd == "param") {
      const std::string name = dec(t.at(1));
      const std::string & kind = t.at(2);
      ParamValue v = helix_arbiter_cpp::Unsupported{};
      if (kind == "b") {
        v = t.at(3) == "1";
      } else if (kind == "i") {
        v = static_cast<std::int64_t>(i64(t.at(3)));
      } else if (kind == "d") {
        v = f64(t.at(3));
      } else if (kind == "s") {
        v = dec(t.at(3));
      }
      params_[name] = v;
      return "param";
    }
    if (cmd == "clearparams") {
      params_.clear();
      return "clearparams";
    }
    if (cmd == "config") {
      try {
        const auto c = helix_arbiter_cpp::parse_config(params_);
        std::string out = "config ok " + enc(c.output_topic) + " " +
          enc(c.status_topic) + " " + enc(c.hold_topic) + " " + f64(c.rate_hz) +
          " " + f64(c.hold_timeout_sec) + " " + f64(c.limits.max_abs_linear) + " " +
          f64(c.limits.max_abs_angular) + " " + std::to_string(c.shutdown_zero_count) + " " +
          (c.autostart ? "1" : "0") + " " + std::to_string(c.sources.size());
        for (const auto & s : c.sources) {
          out += " " + enc(s.name) + " " + enc(s.topic) + " " +
            std::to_string(s.priority) + " " + f64(s.timeout_sec);
        }
        return out;
      } catch (const helix_arbiter_cpp::ConfigError &) {
        return "config error";
      }
    }
    if (cmd == "unknown") {
      std::string out = "unknown";
      for (const auto & n : helix_arbiter_cpp::unknown_parameters(params_)) {
        out += " " + enc(n);
      }
      return out;
    }
    if (cmd == "layout") {
      const std::size_t n = static_cast<std::size_t>(u64(t.at(4)));
      std::vector<std::pair<std::string, std::string>> sources;
      for (std::size_t i = 0; i < n; ++i) {
        sources.emplace_back(dec(t.at(5 + 2 * i)), dec(t.at(6 + 2 * i)));
      }
      try {
        helix_arbiter_cpp::check_topic_layout(dec(t.at(1)), dec(t.at(2)), dec(t.at(3)), sources);
        return "layout ok";
      } catch (const helix_arbiter_cpp::ConfigError &) {
        return "layout error";
      }
    }
    throw std::runtime_error("unknown command '" + cmd + "'");
  }

  Arbiter & arbiter()
  {
    if (!arb_) {
      throw std::runtime_error("no arbiter built");
    }
    return *arb_;
  }

  const std::vector<helix_arbiter_cpp::SourceSpec> & specs() const {return specs_;}
  double hold_timeout() const {return hold_timeout_;}
  helix_arbiter_cpp::Limits limits() const {return limits_;}

private:
  double hold_timeout_{0.5};
  helix_arbiter_cpp::Limits limits_{};
  std::vector<helix_arbiter_cpp::SourceSpec> specs_;
  std::unique_ptr<Arbiter> arb_;
  ParamMap params_;
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

// -- benchmark ---------------------------------------------------------------

enum class Op : std::uint8_t {kSrc, kHold, kDecide, kReset};

struct Event
{
  Op op;
  std::size_t index{0};
  helix_arbiter_cpp::Twist6 twist{};
  bool hold{false};
  std::string fault;
  std::uint64_t epoch{0};
  std::uint64_t seq{0};
  double now{0.0};
};

struct Stats
{
  std::vector<std::int64_t> ns;
};

std::string percentiles_json(std::vector<std::int64_t> v)
{
  if (v.empty()) {
    return "{\"count\": 0}";
  }
  std::sort(v.begin(), v.end());
  auto pct = [&v](double p) {
      // Nearest-rank percentile, the same definition the Python bench uses.
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
  Session setup;
  std::vector<Event> events;
  std::string line;
  while (std::getline(f, line)) {
    const auto t = split(line);
    if (t.empty() || t[0][0] == '#') {
      continue;
    }
    const std::string & c = t[0];
    if (c == "new" || c == "spec" || c == "build") {
      setup.run(t);
      continue;
    }
    Event e;
    if (c == "src") {
      e.op = Op::kSrc;
      e.index = *setup.arbiter().index_of(dec(t.at(1)));
      e.twist = {f64(t.at(2)), f64(t.at(3)), f64(t.at(4)), f64(t.at(5)), f64(t.at(6)),
        f64(t.at(7))};
      e.now = f64(t.at(8));
    } else if (c == "hold") {
      e.op = Op::kHold;
      e.hold = t.at(1) == "1";
      e.fault = dec(t.at(2));
      e.epoch = u64(t.at(3));
      e.seq = u64(t.at(4));
      e.now = f64(t.at(5));
    } else if (c == "decide") {
      e.op = Op::kDecide;
      e.now = f64(t.at(1));
    } else if (c == "reset") {
      e.op = Op::kReset;
    } else {
      continue;  // counters/config lines are not part of the timed workload
    }
    events.push_back(std::move(e));
  }

  using clock = std::chrono::steady_clock;
  std::uint64_t sink = 0;
  auto run_once = [&](Arbiter & arb, Stats * per_op) {
      for (const Event & e : events) {
        const auto t0 = per_op ? clock::now() : clock::time_point{};
        switch (e.op) {
          case Op::kSrc: sink += arb.on_source(e.index, e.twist, e.now); break;
          case Op::kHold: sink += arb.on_hold(e.hold, e.fault, e.epoch, e.seq, e.now); break;
          case Op::kDecide: {
              const auto d = arb.decide(e.now);
              sink += static_cast<std::uint64_t>(d.reason) + d.source.size();
              break;
            }
          case Op::kReset: arb.reset(); break;
        }
        if (per_op) {
          per_op[static_cast<int>(e.op)].ns.push_back(
            std::chrono::duration_cast<std::chrono::nanoseconds>(clock::now() - t0).count());
        }
      }
    };

  for (int i = 0; i < warmup; ++i) {
    Arbiter arb(setup.specs(), setup.hold_timeout(), setup.limits());
    run_once(arb, nullptr);
  }
  // Pass 1: per-call latency (includes one steady_clock::now() pair per call).
  Stats per_op[4];
  for (int i = 0; i < repeat; ++i) {
    Arbiter arb(setup.specs(), setup.hold_timeout(), setup.limits());
    run_once(arb, per_op);
  }
  // Pass 2: uninstrumented throughput over whole replays.
  const auto t0 = clock::now();
  for (int i = 0; i < repeat; ++i) {
    Arbiter arb(setup.specs(), setup.hold_timeout(), setup.limits());
    run_once(arb, nullptr);
  }
  const double elapsed = std::chrono::duration<double>(clock::now() - t0).count();
  // Cost of the timing itself: back-to-back now() pairs.
  std::vector<std::int64_t> overhead;
  for (int i = 0; i < 100000; ++i) {
    const auto a = clock::now();
    overhead.push_back(
      std::chrono::duration_cast<std::chrono::nanoseconds>(clock::now() - a).count());
  }
  const double total = static_cast<double>(events.size()) * repeat;
  std::cout << "{\"implementation\": \"cpp\", \"events_per_replay\": " << events.size() <<
    ", \"repeat\": " << repeat << ", \"warmup\": " << warmup <<
    ", \"throughput_events_per_s\": " << total / elapsed <<
    ", \"timer_overhead\": " << percentiles_json(overhead) <<
    ", \"ops\": {\"src\": " << percentiles_json(per_op[0].ns) <<
    ", \"hold\": " << percentiles_json(per_op[1].ns) <<
    ", \"decide\": " << percentiles_json(per_op[2].ns) <<
    ", \"reset\": " << percentiles_json(per_op[3].ns) << "}, \"checksum\": " << sink << "}\n";
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
    std::cerr << "usage: helix_arbiter_replay < scenario | --bench FILE [--repeat N] "
      "[--warmup N]\n";
    return 2;
  }
  return replay(std::cin, std::cout);
}
