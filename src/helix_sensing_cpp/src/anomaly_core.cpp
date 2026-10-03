// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// See include/helix_sensing_cpp/anomaly_core.hpp. The statement order below
// follows helix_core.anomaly_detector._process_sample line by line.

#include "helix_sensing_cpp/anomaly_core.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "helix_sensing_cpp/pyfmt.hpp"

namespace helix_sensing_cpp
{

AnomalyCore::AnomalyCore(const AnomalyParams & params)
: params_(params)
{
  if (params_.window_size <= 0) {
    throw std::invalid_argument("window_size must be > 0");
  }
}

bool AnomalyCore::cooldown_expired(const MetricState & m, double wall_now) const noexcept
{
  if (params_.emit_cooldown_s <= 0.0) {
    return true;
  }
  if (!m.last_emit) {
    return true;
  }
  return (wall_now - *m.last_emit) >= params_.emit_cooldown_s;
}

void AnomalyCore::gate(MetricState & m, SampleResult & r, double mono_now, double wall_now)
{
  // Called once the streak has reached consecutive_trigger.
  r.elapsed = mono_now - *m.anomaly_start;
  const bool duration_ok =
    params_.min_anomaly_duration_s <= 0.0 || r.elapsed >= params_.min_anomaly_duration_s;
  if (!duration_ok) {
    r.outcome = SampleOutcome::kSuppressedDuration;
    return;
  }
  if (!cooldown_expired(m, wall_now)) {
    r.outcome = SampleOutcome::kSuppressedCooldown;
    r.since_last_emit = wall_now - *m.last_emit;
    return;
  }
  m.last_emit = wall_now;
  r.outcome = SampleOutcome::kEmitted;
}

SampleResult AnomalyCore::process(
  const std::string & metric_name, double value, double mono_now, double wall_now)
{
  auto it = metrics_.find(metric_name);
  if (it == metrics_.end()) {
    it = metrics_.emplace(
      metric_name, MetricState(static_cast<std::size_t>(params_.window_size))).first;
  }
  MetricState & m = it->second;
  SampleResult r;

  if (std::isnan(value)) {
    // Stale topic: count a violation, never pollute the window with NaN.
    r.stale = true;
    m.consecutive += 1;
    r.consecutive = m.consecutive;
    if (!m.anomaly_start) {
      m.anomaly_start = mono_now;
    }
    r.outcome = SampleOutcome::kStaleViolation;
    if (m.consecutive >= params_.consecutive_trigger) {
      gate(m, r, mono_now, wall_now);
      if (r.outcome == SampleOutcome::kEmitted) {
        r.fault = stale_fault(metric_name, m.consecutive, wall_now);
      }
    }
    return r;
  }

  const ZScoreResult z = m.stats.evaluate(value);
  r.std = z.std;
  if (z.status == ZScoreStatus::kInsufficient) {
    r.outcome = SampleOutcome::kInsufficient;
  } else if (z.status == ZScoreStatus::kFlat) {
    r.outcome = SampleOutcome::kFlat;
  } else if (z.zscore > params_.zscore_threshold) {
    r.zscore = z.zscore;
    m.consecutive += 1;
    r.consecutive = m.consecutive;
    if (!m.anomaly_start) {
      m.anomaly_start = mono_now;
    }
    r.outcome = SampleOutcome::kViolation;
    if (m.consecutive >= params_.consecutive_trigger) {
      gate(m, r, mono_now, wall_now);
      if (r.outcome == SampleOutcome::kEmitted) {
        r.fault = anomaly_fault(metric_name, value, z, m.consecutive, wall_now);
      }
    }
  } else {
    // Also covers a NaN z-score (an inf in the window): every comparison
    // with NaN is false in both languages, so the streak resets.
    r.zscore = z.zscore;
    r.streak_was_active = m.consecutive > 0;
    m.consecutive = 0;
    m.anomaly_start.reset();
    r.outcome = SampleOutcome::kNormal;
  }
  // Always append after evaluating: keeps the baseline from being poisoned.
  m.stats.push(value);
  r.consecutive = m.consecutive;
  return r;
}

FaultRecord AnomalyCore::anomaly_fault(
  const std::string & metric, double value, const ZScoreResult & z,
  std::int64_t consecutive, double wall_now) const
{
  FaultRecord f;
  f.node_name = metric;  // the metric name identifies an ANOMALY
  f.detail = "Metric '" + metric + "' Z-score " + python_fixed(z.zscore, 2) +
    " exceeded threshold on " + std::to_string(params_.consecutive_trigger) +
    " consecutive samples";
  f.timestamp = wall_now;
  f.context_keys = {"metric_name", "current_value", "window_mean", "window_std", "zscore",
    "consecutive_count"};
  f.context_values = {metric, python_round_str(value, 4), python_round_str(z.mean, 4),
    python_round_str(z.std, 6), python_round_str(z.zscore, 2), std::to_string(consecutive)};
  return f;
}

FaultRecord AnomalyCore::stale_fault(
  const std::string & metric, std::int64_t consecutive, double wall_now) const
{
  // Same fault_type as the z-score path so diagnosis rule R1 catches both;
  // violation_type == "stale" tells them apart.
  FaultRecord f;
  f.node_name = metric;
  f.detail = "Metric '" + metric + "' stale, no samples in window on " +
    std::to_string(consecutive) + " consecutive checks";
  f.timestamp = wall_now;
  f.context_keys = {"metric_name", "violation_type", "consecutive_count"};
  f.context_values = {metric, "stale", std::to_string(consecutive)};
  return f;
}

}  // namespace helix_sensing_cpp
