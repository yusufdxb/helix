// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// RateWindow: sliding-window arrival-rate estimator, a port of
// helix_adapter.rate_window.RateWindow with the same contract:
//
//   * keeps arrival times of the last window_sec seconds
//   * rate = (n - 1) / (newest - oldest)
//   * NaN when the window is empty (stale topic), 0.0 with one sample or a
//     span below 1 us
//
// Optional stale_after_s > 0: NaN as soon as the newest arrival is older than
// stale_after_s, instead of waiting until the whole window has emptied. With
// the shipped 5 s window a dead topic otherwise reports a near-normal rate
// for about 5 s (old arrivals keep (n-1)/span unchanged while they drain).
//
// The caller supplies time; not internally synchronized (the node runs a
// single-threaded executor).
#ifndef HELIX_SENSING_CPP__RATE_WINDOW_HPP_
#define HELIX_SENSING_CPP__RATE_WINDOW_HPP_

#include <deque>
#include <limits>
#include <stdexcept>

namespace helix_sensing_cpp
{

class RateWindow
{
public:
  explicit RateWindow(double window_sec, double stale_after_s = 0.0)
  : window_sec_(window_sec), stale_after_s_(stale_after_s)
  {
    if (!(window_sec > 0.0)) {
      throw std::invalid_argument("window_sec must be > 0");
    }
    if (stale_after_s < 0.0) {
      throw std::invalid_argument("stale_after_s must be >= 0");
    }
  }

  void record(double now)
  {
    stamps_.push_back(now);
    evict(now);
  }

  double rate_or_nan(double now)
  {
    evict(now);
    const auto n = stamps_.size();
    if (n == 0) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    if (stale_after_s_ > 0.0 && now - stamps_.back() > stale_after_s_) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    if (n == 1) {
      return 0.0;
    }
    const double span = stamps_.back() - stamps_.front();
    if (span < 1e-6) {
      return 0.0;
    }
    return static_cast<double>(n - 1) / span;
  }

  std::size_t size() const noexcept {return stamps_.size();}

private:
  void evict(double now)
  {
    const double cutoff = now - window_sec_;
    while (!stamps_.empty() && stamps_.front() < cutoff) {
      stamps_.pop_front();
    }
  }

  double window_sec_;
  double stale_after_s_;
  std::deque<double> stamps_;
};

}  // namespace helix_sensing_cpp

#endif  // HELIX_SENSING_CPP__RATE_WINDOW_HPP_
