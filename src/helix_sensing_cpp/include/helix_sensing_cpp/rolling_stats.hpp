// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// RollingStats — sliding-window mean / population-std / z-score kernel.
//
// Header-only so tests link with zero dependencies. Semantics mirror the
// Python helix_core.anomaly_detector._process_sample contract exactly:
//
//   * window is a bounded FIFO of the last N samples (std::deque-backed).
//   * mean  = sum(samples) / len(samples)
//   * var   = sum((s - mean)^2) / len(samples)        // POPULATION, /N
//   * std   = sqrt(var)
//   * if std < FLAT_SIGNAL_EPSILON -> Z-score not meaningful (kFlat).
//   * otherwise z = |x - mean| / std.
//   * Z-score is evaluated BEFORE pushing the new sample, so a streak of
//     spikes cannot poison its own baseline.
//   * Fewer than 2 historical samples -> Z-score not meaningful
//     (kInsufficient).
//
// Thread-safety: NOT internally synchronized. The node owns a map of
// RollingStats per metric and protects access with its own mutex, which
// matches the Python node's _data_lock pattern. Keeping this kernel
// lock-free makes the math trivially unit-testable.
#ifndef HELIX_SENSING_CPP__ROLLING_STATS_HPP_
#define HELIX_SENSING_CPP__ROLLING_STATS_HPP_

#include <cmath>
#include <cstddef>
#include <deque>
#include <stdexcept>

namespace helix_sensing_cpp
{

inline constexpr double kFlatSignalEpsilon = 1e-6;

enum class ZScoreStatus
{
  kOk,            // z-score is meaningful
  kInsufficient,  // fewer than 2 historical samples
  kFlat,          // std below kFlatSignalEpsilon
};

struct ZScoreResult
{
  ZScoreStatus status = ZScoreStatus::kInsufficient;
  double mean = 0.0;
  double std = 0.0;
  double zscore = 0.0;  // only valid when status == kOk
};

class RollingStats
{
public:
  explicit RollingStats(std::size_t window_size)
  : window_size_(window_size)
  {
    if (window_size == 0) {
      throw std::invalid_argument("RollingStats window_size must be > 0");
    }
  }

  // Compute mean / std / z-score against the CURRENT contents of the
  // window. Does NOT mutate the window; callers are expected to call
  // push() afterwards (so that spikes don't contaminate their own baseline).
  ZScoreResult evaluate(double value) const
  {
    ZScoreResult r;
    const std::size_t n = samples_.size();
    if (n < 2) {
      r.status = ZScoreStatus::kInsufficient;
      return r;
    }

    double sum = 0.0;
    for (double s : samples_) {
      sum += s;
    }
    const double mean = sum / static_cast<double>(n);

    double sq = 0.0;
    for (double s : samples_) {
      const double d = s - mean;
      sq += d * d;
    }
    // POPULATION variance — divide by N, not N-1, matching Python.
    const double variance = sq / static_cast<double>(n);
    const double std_ = std::sqrt(variance);

    r.mean = mean;
    r.std = std_;

    if (std_ < kFlatSignalEpsilon) {
      r.status = ZScoreStatus::kFlat;
      return r;
    }

    r.zscore = std::fabs((value - mean) / std_);
    r.status = ZScoreStatus::kOk;
    return r;
  }

  // Append a sample. Oldest element is evicted once size > window_size_.
  void push(double value)
  {
    samples_.push_back(value);
    while (samples_.size() > window_size_) {
      samples_.pop_front();
    }
  }

  std::size_t size() const noexcept {return samples_.size();}
  std::size_t capacity() const noexcept {return window_size_;}

  void clear() noexcept {samples_.clear();}

private:
  std::size_t window_size_;
  std::deque<double> samples_;
};

}  // namespace helix_sensing_cpp

#endif  // HELIX_SENSING_CPP__ROLLING_STATS_HPP_
