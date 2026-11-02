// Adaptive timing envelope for one (producer, output) pair. Pure C++, no ROS.
//
// While the producer is healthy it learns the inter-message gap online:
//   - an EWMA of the mean gap,
//   - a decaying maximum gap (jitter envelope),
//   - the largest schedule deficit seen while healthy.
// It then judges two things with no fixed period assumption:
//   - stale:    no message for longer than max(floor, mult * decaying max gap)
//   - rate loss: a one-sided CUSUM of how far the gaps run behind schedule. Each gap adds
//     gap/mean - k, where k sits halfway between nominal (1) and the degraded gap ratio
//     (1/degraded_ratio). The open gap since the last message counts too, so a slow-down is
//     seen before the next message arrives.
// Until enough healthy gaps are seen, every limit is 0 and the caller keeps its fixed rule.
#pragma once

#include <cstddef>

namespace helix_healer {

struct EnvelopeParams {
  double floor_s = 0.1;
  double stale_mult = 3.0;
  double min_gaps = 50;
  double min_s = 2.0;
  double tau_s = 60.0;
  double h_min = 4.0;
  double h_mult = 2.0;
  double degraded_ratio = 0.5;
};

class Envelope {
 public:
  Envelope() = default;
  explicit Envelope(const EnvelopeParams& p) : p_(p) {}

  // A message arrived at time t. learn=true only while the producer is judged healthy:
  // the detector state always advances, the learned baseline only when learn is set.
  void arrival(double t, bool learn);
  // The producer was respawned: forget the last arrival and the running deficit, keep the baseline.
  void reset_process();

  bool ready() const;
  std::size_t gaps() const { return n_; }
  double mean_gap() const { return mean_; }
  double max_gap() const { return gmax_; }
  double k() const;
  // 0 when not ready.
  double stale_limit() const;
  double deficit_limit() const;
  // Running deficit including the open gap up to `now`.
  double deficit(double now) const;

 private:
  EnvelopeParams p_;
  double last_ = -1;
  double mean_ = 0;
  double gmax_ = 0;
  double d_ = 0;
  double dmax_ = 0;
  double learned_s_ = 0;
  std::size_t n_ = 0;
};

}  // namespace helix_healer
