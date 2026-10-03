// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// AnomalyCore: the ROS-free detection logic of AnomalyDetectorNode, a port
// of helix_core.anomaly_detector.AnomalyDetector._process_sample and its
// two emitters. The node owns one instance and supplies both clocks on every
// call, so the logic is deterministic under test and is compared with the
// Python reference sample by sample (test/test_python_parity.py).
//
// Per metric, in order:
//   * NaN (topic_rate_monitor's "stale topic"): counts as a violation of the
//     same shape as a z-score breach; NaN never enters the window.
//   * Fewer than 2 samples in the window: nothing to compare against.
//   * Window std < 1e-6 (flat): z-score skipped; the streak is untouched.
//   * z = |x - mean| / std over the window BEFORE x is appended.
//     z > zscore_threshold extends the streak, otherwise resets it.
//   * A streak of consecutive_trigger violations emits once it has lasted
//     min_anomaly_duration_s (monotonic clock; <= 0 disables), and at most
//     once per emit_cooldown_s per metric (wall clock, as time.time() in
//     the reference; <= 0 restores the legacy flood).
//   * Every non-NaN sample is appended after evaluation.
#ifndef HELIX_SENSING_CPP__ANOMALY_CORE_HPP_
#define HELIX_SENSING_CPP__ANOMALY_CORE_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "helix_sensing_cpp/rolling_stats.hpp"

namespace helix_sensing_cpp
{

struct AnomalyParams
{
  double zscore_threshold{3.0};
  std::int64_t consecutive_trigger{3};
  std::int64_t window_size{60};
  double emit_cooldown_s{1.0};
  double min_anomaly_duration_s{2.0};
};

/// The fields of a helix_msgs/FaultEvent, as the reference fills them.
struct FaultRecord
{
  std::string node_name;  // the metric name
  std::string fault_type{"ANOMALY"};
  std::int8_t severity{2};
  std::string detail;
  double timestamp{0.0};  // wall clock, seconds
  std::vector<std::string> context_keys;
  std::vector<std::string> context_values;
};

enum class SampleOutcome : std::uint8_t
{
  kInsufficient,        // < 2 samples in the window
  kFlat,                // window std below kFlatSignalEpsilon
  kNormal,              // z <= threshold, streak reset
  kViolation,           // z > threshold, streak below the trigger
  kStaleViolation,      // NaN, streak below the trigger
  kSuppressedDuration,  // trigger reached, min_anomaly_duration_s not yet
  kSuppressedCooldown,  // trigger reached, inside emit_cooldown_s
  kEmitted,             // fault produced
};

struct SampleResult
{
  SampleOutcome outcome{SampleOutcome::kInsufficient};
  bool stale{false};             // the sample was NaN
  std::int64_t consecutive{0};   // streak length after this sample
  double zscore{0.0};            // valid when a z-score was computed
  double std{0.0};
  double elapsed{0.0};           // streak duration, monotonic seconds
  double since_last_emit{0.0};   // wall seconds, when cooldown-suppressed
  bool streak_was_active{false};  // for the reference's "resetting" debug line
  std::optional<FaultRecord> fault;
};

class AnomalyCore
{
public:
  /// Throws std::invalid_argument when window_size <= 0 (the reference node
  /// refuses to configure in that case too). Other values are accepted as
  /// the reference accepts them.
  explicit AnomalyCore(const AnomalyParams & params);

  /// One sample of one metric. @p mono_now and @p wall_now are seconds on a
  /// monotonic and on the wall clock (time.monotonic() / time.time()).
  SampleResult process(
    const std::string & metric_name, double value, double mono_now, double wall_now);

  void clear() noexcept {metrics_.clear();}
  std::size_t metric_count() const noexcept {return metrics_.size();}
  const AnomalyParams & params() const noexcept {return params_;}

private:
  struct MetricState
  {
    explicit MetricState(std::size_t window)
    : stats(window) {}
    RollingStats stats;
    std::int64_t consecutive{0};
    std::optional<double> anomaly_start;  // monotonic; set while a streak is active
    std::optional<double> last_emit;      // wall clock of the last emitted fault
  };

  bool cooldown_expired(const MetricState & m, double wall_now) const noexcept;
  FaultRecord anomaly_fault(
    const std::string & metric, double value, const ZScoreResult & z,
    std::int64_t consecutive, double wall_now) const;
  FaultRecord stale_fault(
    const std::string & metric, std::int64_t consecutive, double wall_now) const;
  void gate(
    MetricState & m, SampleResult & r, double mono_now, double wall_now);

  AnomalyParams params_;
  std::unordered_map<std::string, MetricState> metrics_;
};

}  // namespace helix_sensing_cpp

#endif  // HELIX_SENSING_CPP__ANOMALY_CORE_HPP_
