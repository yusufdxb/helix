#include "helix_healer/envelope.hpp"

#include <algorithm>
#include <cmath>

namespace helix_healer {

double Envelope::k() const { return 1.0 + (1.0 / std::max(p_.degraded_ratio, 1e-3) - 1.0) / 2.0; }

void Envelope::arrival(double t, bool learn) {
  if (last_ < 0) {
    last_ = t;
    return;
  }
  const double g = std::max(0.0, t - last_);
  last_ = t;
  if (mean_ > 0) d_ = std::max(0.0, d_ + g / mean_ - k());
  if (!learn) return;
  ++n_;
  learned_s_ += g;
  const double a = std::max(1.0 / static_cast<double>(n_ + 1), 0.02);
  mean_ = n_ == 1 ? g : mean_ + a * (g - mean_);
  const double decay = std::exp(-g / p_.tau_s);
  gmax_ = std::max(g, mean_ + (gmax_ - mean_) * decay);
  dmax_ = std::max(d_, dmax_ * decay);
}

void Envelope::reset_process() {
  last_ = -1;
  d_ = 0;
}

bool Envelope::ready() const {
  return mean_ > 0 && static_cast<double>(n_) >= p_.min_gaps && learned_s_ >= p_.min_s;
}

double Envelope::stale_limit() const { return ready() ? std::max(p_.floor_s, p_.stale_mult * gmax_) : 0.0; }

double Envelope::deficit_limit() const { return ready() ? std::max(p_.h_min, p_.h_mult * dmax_) : 0.0; }

double Envelope::deficit(double now) const {
  if (mean_ <= 0) return 0;
  const double open = last_ >= 0 ? std::max(0.0, (now - last_) / mean_ - k()) : 0.0;
  return d_ + open;
}

}  // namespace helix_healer
