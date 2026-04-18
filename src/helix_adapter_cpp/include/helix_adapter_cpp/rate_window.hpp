// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// Sliding-window callback-arrival rate estimator.
//
// Thread-safe C++ analogue of helix_adapter.rate_window.RateWindow. Same
// semantics, same unit-test contract:
//   - empty window       -> rate_or_nan() returns NaN
//   - exactly one sample -> rate_or_nan() returns 0.0
//   - >=2 samples        -> (n - 1) / (last - first), at least 1e-6 span
// The clock source is injected so tests can drive synthetic time.
#ifndef HELIX_ADAPTER_CPP__RATE_WINDOW_HPP_
#define HELIX_ADAPTER_CPP__RATE_WINDOW_HPP_

#include <cstddef>
#include <deque>
#include <functional>
#include <mutex>

namespace helix_adapter_cpp
{

class RateWindow
{
public:
  using Clock = std::function<double ()>;

  explicit RateWindow(double window_sec, Clock clock = DefaultClock());

  void record();
  void record(double now_sec);

  double rate_or_nan();
  double rate_or_nan(double now_sec);

  std::size_t size() const;

  static Clock DefaultClock();

private:
  void evict_locked(double now_sec);

  double window_sec_;
  Clock clock_;
  mutable std::mutex mutex_;
  std::deque<double> timestamps_;
};

}  // namespace helix_adapter_cpp

#endif  // HELIX_ADAPTER_CPP__RATE_WINDOW_HPP_
