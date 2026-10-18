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
//
// Opt-in extensions (every default reproduces the reference exactly):
//   * baseline_mode "freeze": a violating sample is NOT appended, and while a
//     fault is active only samples inside the clear band are. The rolling
//     reference lets a sustained fault fill its own window, so the z-score
//     decays below threshold at about the time the duration gate opens
//     (self-masking); freezing keeps the pre-fault baseline.
//   * std_floor_rel > 0: z is computed against max(std, std_floor_rel*|mean|).
//     A very steady rate (std ~0.3 % of mean) otherwise turns sub-percent
//     jitter into a large z; the floor states the smallest relative change
//     that can count as a fault. It also makes a flat window evaluable.
//   * duration_tolerance_s: the duration gate passes at
//     elapsed + tolerance >= min_anomaly_duration_s. With samples on a fixed
//     0.5 s grid a 2.0 s gate sits exactly on a sample, and sub-ms jitter
//     decides between 5 and 6 samples; a half-period tolerance makes it 5.
//   * anchor_tolerance_rel > 0: the first time the window is full with no
//     active streak, its mean becomes the metric's anchor. A sample further
//     than anchor_tolerance_rel * |anchor| from it is a violation too (same
//     streak and gates), and while that fault is active a clear needs the
//     sample back inside half the band (hysteresis against edge flapping).
//     A slow drift moves the rolling mean with it and inflates the window
//     std, so z alone never sees it; the anchor does. This is the
//     expected-rate band of diagnostic_updater's FrequencyStatus, with the
//     expected rate learned instead of configured, so it applies only to
//     metrics whose name starts with anchor_metric_prefix (rate_hz/ by
//     default): a band around a near-zero or motion-dependent metric such
//     as pose displacement rate is meaningless.
//   * clear_samples > 0: after a fault, clear_samples consecutive samples
//     with z <= clear_zscore (default zscore_threshold) produce a clear
//     record: evidence that the metric is normal again. A metric that never
//     faulted announces itself healthy once the same way.
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
  bool freeze_baseline{false};
  double std_floor_rel{0.0};
  double duration_tolerance_s{0.0};
  std::int64_t clear_samples{0};
  double clear_zscore{0.0};          // <= 0: use zscore_threshold
  double anchor_tolerance_rel{0.0};
  std::string anchor_metric_prefix{"rate_hz/"};  // metrics the anchor applies to
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
  bool anchor_violation{false};  // outside the anchored band (anchor_tolerance_rel)
  double std{0.0};
  double elapsed{0.0};           // streak duration, monotonic seconds
  double since_last_emit{0.0};   // wall seconds, when cooldown-suppressed
  bool streak_was_active{false};  // for the reference's "resetting" debug line
  bool pushed{false};            // the sample entered the window
  std::optional<FaultRecord> fault;
  std::optional<FaultRecord> clear;   // clear record (clear_samples > 0 only)
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
    MetricState(std::size_t window, bool anchorable)
    : stats(window), anchorable(anchorable) {}
    RollingStats stats;
    std::int64_t consecutive{0};
    std::optional<double> anomaly_start;  // monotonic; set while a streak is active
    std::optional<double> last_emit;      // wall clock of the last emitted fault
    bool fault_active{false};             // a fault was emitted and not yet cleared
    bool announced{false};                // a clear record was published at least once
    std::int64_t normal_run{0};           // consecutive samples inside the clear band
    std::optional<double> anchor;         // anchor_tolerance_rel reference
    bool anchorable{false};               // name matches anchor_metric_prefix
  };

  bool cooldown_expired(const MetricState & m, double wall_now) const noexcept;
  FaultRecord anomaly_fault(
    const std::string & metric, double value, const ZScoreResult & z,
    std::int64_t consecutive, double wall_now) const;
  FaultRecord stale_fault(
    const std::string & metric, std::int64_t consecutive, double wall_now) const;
  void gate(
    MetricState & m, SampleResult & r, double mono_now, double wall_now);
  void maybe_clear(
    const std::string & metric, MetricState & m, SampleResult & r, double wall_now);
  double clear_z() const noexcept;

  AnomalyParams params_;
  std::unordered_map<std::string, MetricState> metrics_;
};

}  // namespace helix_sensing_cpp

#endif  // HELIX_SENSING_CPP__ANOMALY_CORE_HPP_
