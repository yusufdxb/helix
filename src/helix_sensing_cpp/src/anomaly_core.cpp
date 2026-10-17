// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// See include/helix_sensing_cpp/anomaly_core.hpp. The statement order below
// follows helix_core.anomaly_detector._process_sample line by line.

#include "helix_sensing_cpp/anomaly_core.hpp"

#include <algorithm>
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
  const bool duration_ok = params_.min_anomaly_duration_s <= 0.0 ||
    r.elapsed + params_.duration_tolerance_s >= params_.min_anomaly_duration_s;
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
      metric_name, MetricState(
        static_cast<std::size_t>(params_.window_size),
        metric_name.rfind(params_.anchor_metric_prefix, 0) == 0)).first;
  }
  MetricState & m = it->second;
  SampleResult r;

  if (std::isnan(value)) {
    // Stale topic: count a violation, never pollute the window with NaN.
    r.stale = true;
    m.normal_run = 0;
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
        m.fault_active = true;
      }
    }
    return r;
  }

  ZScoreResult z = m.stats.evaluate(value);
  if (params_.std_floor_rel > 0.0 && z.status != ZScoreStatus::kInsufficient) {
    const double s = std::max(z.std, params_.std_floor_rel * std::fabs(z.mean));
    if (s >= kFlatSignalEpsilon) {
      z.zscore = std::fabs((value - z.mean) / s);
      z.status = ZScoreStatus::kOk;
    }
  }
  r.std = z.std;
  bool violation = false;
  bool outside_anchor = false;
  if (params_.anchor_tolerance_rel > 0.0 && m.anchorable) {
    if (!m.anchor && m.consecutive == 0 && !m.fault_active &&
      m.stats.size() >= static_cast<std::size_t>(params_.window_size))
    {
      m.anchor = z.mean;
    }
    if (m.anchor) {
      outside_anchor =
        std::fabs(value - *m.anchor) > params_.anchor_tolerance_rel * std::fabs(*m.anchor);
    }
  }
  r.anchor_violation = outside_anchor;
  // Hysteresis: while a fault is active, a sample only counts toward
  // clearing (and only enters the window) once it is back inside half the
  // anchor band, so a value hovering at the band edge can neither clear the
  // fault nor drag the baseline toward the faulted value.
  const bool in_clear_band = !m.anchor || !m.fault_active ||
    std::fabs(value - *m.anchor) <= 0.5 * params_.anchor_tolerance_rel * std::fabs(*m.anchor);
  const bool over_threshold =
    z.status == ZScoreStatus::kOk && z.zscore > params_.zscore_threshold;
  if (z.status == ZScoreStatus::kInsufficient && !outside_anchor) {
    r.outcome = SampleOutcome::kInsufficient;
  } else if (z.status == ZScoreStatus::kFlat && !outside_anchor) {
    r.outcome = SampleOutcome::kFlat;
  } else if (outside_anchor || over_threshold) {
    r.zscore = z.zscore;
    violation = true;
    m.normal_run = 0;
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
        if (m.anchor) {
          r.fault->context_keys.push_back("anchor");
          r.fault->context_values.push_back(python_round_str(*m.anchor, 4));
          r.fault->context_keys.push_back("anchor_violation");
          r.fault->context_values.push_back(outside_anchor ? "1" : "0");
        }
        m.fault_active = true;
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
    if (params_.clear_samples > 0) {
      // outside_anchor is false here: an anchored violation took the branch above.
      m.normal_run = (z.zscore <= clear_z() && in_clear_band) ? m.normal_run + 1 : 0;
      maybe_clear(metric_name, m, r, wall_now);
    }
  }
  // Reference: always append after evaluating. freeze_baseline keeps a
  // violating sample, and while a fault is active anything outside the clear
  // band, out of the window that judges it.
  bool push = true;
  const bool above_clear_z = z.status == ZScoreStatus::kOk && z.zscore > clear_z();
  if (params_.freeze_baseline) {
    if (violation) {
      push = false;
    } else if (m.fault_active && (!in_clear_band || above_clear_z)) {
      push = false;
    }
  }
  if (push) {
    m.stats.push(value);
  }
  r.pushed = push;
  r.consecutive = m.consecutive;
  return r;
}

double AnomalyCore::clear_z() const noexcept
{
  return params_.clear_zscore > 0.0 ? params_.clear_zscore : params_.zscore_threshold;
}

void AnomalyCore::maybe_clear(
  const std::string & metric, MetricState & m, SampleResult & r, double wall_now)
{
  if (m.normal_run < params_.clear_samples || (!m.fault_active && m.announced)) {
    return;
  }
  FaultRecord f;
  f.node_name = metric;
  f.severity = 0;
  const char * why = m.fault_active ? "recovered" : "healthy";
  f.detail = "Metric '" + metric + "' " + why + ": " + std::to_string(m.normal_run) +
    " consecutive samples with z <= " + python_fixed(clear_z(), 2);
  f.timestamp = wall_now;
  f.context_keys = {"metric_name", "clear_reason", "normal_run"};
  f.context_values = {metric, why, std::to_string(m.normal_run)};
  m.fault_active = false;
  m.announced = true;
  r.clear = std::move(f);
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
