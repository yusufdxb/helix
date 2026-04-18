// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
#include "helix_adapter_cpp/rate_window.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace helix_adapter_cpp
{

RateWindow::Clock RateWindow::DefaultClock()
{
  return []() {
           const auto now = std::chrono::steady_clock::now().time_since_epoch();
           return std::chrono::duration<double>(now).count();
         };
}

RateWindow::RateWindow(double window_sec, Clock clock)
: window_sec_(window_sec), clock_(std::move(clock))
{
  if (window_sec_ <= 0.0) {
    throw std::invalid_argument("window_sec must be > 0");
  }
  if (!clock_) {
    clock_ = DefaultClock();
  }
}

void RateWindow::record() {record(clock_());}

void RateWindow::record(double now_sec)
{
  std::lock_guard<std::mutex> lk(mutex_);
  timestamps_.push_back(now_sec);
  evict_locked(now_sec);
}

double RateWindow::rate_or_nan() {return rate_or_nan(clock_());}

double RateWindow::rate_or_nan(double now_sec)
{
  std::lock_guard<std::mutex> lk(mutex_);
  evict_locked(now_sec);
  const auto n = timestamps_.size();
  if (n == 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (n == 1) {
    return 0.0;
  }
  const double span = timestamps_.back() - timestamps_.front();
  if (span < 1e-6) {
    return 0.0;
  }
  return static_cast<double>(n - 1) / span;
}

std::size_t RateWindow::size() const
{
  std::lock_guard<std::mutex> lk(mutex_);
  return timestamps_.size();
}

void RateWindow::evict_locked(double now_sec)
{
  const double cutoff = now_sec - window_sec_;
  while (!timestamps_.empty() && timestamps_.front() < cutoff) {
    timestamps_.pop_front();
  }
}

}  // namespace helix_adapter_cpp
