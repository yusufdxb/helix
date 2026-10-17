// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// helix_detector_eval: runs AnomalyCore (the code the node runs) over a
// recorded or synthesized metric stream and prints every fault and clear
// record. Used by analysis/detector_eval.py for the held-out detector study.
//
// Usage:
//   helix_detector_eval key=value ... < samples.csv > events.jsonl
//
// Input lines: "<t seconds>,<metric>,<value>" with value "nan" for a stale
// sample. Both core clocks are driven from t. Keys: zscore_threshold,
// consecutive_trigger, window_size, emit_cooldown_s, min_anomaly_duration_s,
// freeze_baseline (0/1), std_floor_rel, duration_tolerance_s, clear_samples,
// clear_zscore, anchor_tolerance_rel.
// Output: one JSON object per record:
//   {"t":..,"metric":"..","kind":"fault"|"clear","stale":0|1,"z":..}

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include "helix_sensing_cpp/anomaly_core.hpp"

namespace
{

using helix_sensing_cpp::AnomalyCore;
using helix_sensing_cpp::AnomalyParams;

void set_param(AnomalyParams & p, const std::string & kv)
{
  const auto eq = kv.find('=');
  if (eq == std::string::npos) {
    throw std::invalid_argument("expected key=value, got " + kv);
  }
  const std::string k = kv.substr(0, eq);
  const double v = std::stod(kv.substr(eq + 1));
  if (k == "zscore_threshold") {
    p.zscore_threshold = v;
  } else if (k == "consecutive_trigger") {
    p.consecutive_trigger = static_cast<std::int64_t>(v);
  } else if (k == "window_size") {
    p.window_size = static_cast<std::int64_t>(v);
  } else if (k == "emit_cooldown_s") {
    p.emit_cooldown_s = v;
  } else if (k == "min_anomaly_duration_s") {
    p.min_anomaly_duration_s = v;
  } else if (k == "freeze_baseline") {
    p.freeze_baseline = v != 0.0;
  } else if (k == "std_floor_rel") {
    p.std_floor_rel = v;
  } else if (k == "duration_tolerance_s") {
    p.duration_tolerance_s = v;
  } else if (k == "clear_samples") {
    p.clear_samples = static_cast<std::int64_t>(v);
  } else if (k == "clear_zscore") {
    p.clear_zscore = v;
  } else if (k == "anchor_tolerance_rel") {
    p.anchor_tolerance_rel = v;
  } else {
    throw std::invalid_argument("unknown key " + k);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  AnomalyParams p;
  try {
    for (int i = 1; i < argc; ++i) {
      set_param(p, argv[i]);
    }
  } catch (const std::exception & e) {
    std::cerr << "helix_detector_eval: " << e.what() << "\n";
    return 2;
  }
  AnomalyCore core(p);
  std::ios::sync_with_stdio(false);
  std::string line;
  while (std::getline(std::cin, line)) {
    const auto a = line.find(',');
    const auto b = line.rfind(',');
    if (a == std::string::npos || b == a) {
      continue;
    }
    const double t = std::stod(line.substr(0, a));
    const std::string metric = line.substr(a + 1, b - a - 1);
    const std::string vs = line.substr(b + 1);
    const double v = (vs == "nan" || vs.empty()) ? std::nan("") : std::stod(vs);
    const auto r = core.process(metric, v, t, t);
    if (r.fault) {
      std::printf(
        "{\"t\":%.6f,\"metric\":\"%s\",\"kind\":\"fault\",\"stale\":%d,\"z\":%.4f}\n",
        t, metric.c_str(), r.stale ? 1 : 0, r.zscore);
    }
    if (r.clear) {
      std::printf(
        "{\"t\":%.6f,\"metric\":\"%s\",\"kind\":\"clear\",\"stale\":0,\"z\":%.4f}\n",
        t, metric.c_str(), r.zscore);
    }
  }
  return 0;
}
