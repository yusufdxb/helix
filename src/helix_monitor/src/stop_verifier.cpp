#include "helix_monitor/stop_verifier.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace helix_monitor {

const char* to_string(Verdict v) {
  switch (v) {
    case Verdict::kNone: return "NONE";
    case Verdict::kPending: return "PENDING";
    case Verdict::kVerified: return "VERIFIED";
    case Verdict::kFailed: return "FAILED";
    case Verdict::kUnverified: return "UNVERIFIED";
  }
  return "?";
}

StopVerifier::StopVerifier(std::vector<EvidenceGroup> groups, Params p)
    : groups_(std::move(groups)), p_(p), bufs_(groups_.size()) {}

void StopVerifier::command(double t) {
  state_ = Verdict::kPending;
  t_cmd_ = t;
}

void StopVerifier::on_sample(const std::string& stream, const Sample& s) {
  for (std::size_t g = 0; g < groups_.size(); ++g) {
    const auto& G = groups_[g];
    if (G.stream != stream) continue;
    std::vector<double> x(G.channels.size(), std::numeric_limits<double>::quiet_NaN());
    for (std::size_t i = 0; i < G.channels.size(); ++i)
      if (G.channels[i] < s.v.size()) x[i] = s.v[G.channels[i]] * (i < G.scale.size() ? G.scale[i] : 1.0);
    auto& b = bufs_[g];
    b.w.emplace_back(s.t, std::move(x));
    b.last = s.t;
    while (!b.w.empty() && b.w.front().first < s.t - p_.window_s) b.w.pop_front();
  }
}

double StopVerifier::metric(std::size_t g, double t) const {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto& b = bufs_[g];
  const auto& G = groups_[g];
  if (b.last < 0.0 || t - b.last > p_.stale_s || b.w.size() < 3) return nan;
  if (b.w.back().first - b.w.front().first < 0.8 * p_.window_s) return nan;
  const std::size_t n = G.channels.size();
  std::vector<double> mean(n, 0.0);
  if (G.detrend) {
    for (const auto& e : b.w)
      for (std::size_t i = 0; i < n; ++i) mean[i] += e.second[i];
    for (auto& m : mean) m /= static_cast<double>(b.w.size());
  }
  double acc = 0.0;
  for (const auto& e : b.w) {
    double sq = 0.0, mx = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      const double v = e.second[i] - mean[i];
      if (!std::isfinite(v)) return nan;
      sq += v * v;
      mx = std::max(mx, std::abs(v));
    }
    acc = G.use_max ? std::max(acc, mx) : acc + sq;
  }
  return G.use_max ? acc : std::sqrt(acc / static_cast<double>(b.w.size()));
}

bool StopVerifier::tick(double t, StopResult& out) {
  if (state_ != Verdict::kPending) return false;
  // Evidence must come entirely from after the command.
  const bool window_ready = t >= t_cmd_ + p_.window_s;
  const bool expired = t >= t_cmd_ + p_.deadline_s;
  if (!window_ready && !expired) return false;
  int available = 0, quiet = 0, raw_quiet = 0, moving = 0;
  std::vector<double> m(groups_.size());
  for (std::size_t g = 0; g < groups_.size(); ++g) {
    m[g] = metric(g, t);
    if (std::isnan(m[g])) continue;
    ++available;
    if (m[g] < groups_[g].quiet) {
      ++quiet;
      if (groups_[g].raw) ++raw_quiet;
    } else if (m[g] > groups_[g].moving) {
      ++moving;
    }
  }
  Verdict v = Verdict::kPending;
  if (quiet >= p_.min_groups && (!p_.require_raw || raw_quiet > 0) && moving == 0) {
    v = Verdict::kVerified;
  } else if (expired) {
    v = moving > 0 ? Verdict::kFailed : Verdict::kUnverified;
  }
  (void)available;
  if (v == Verdict::kPending) return false;
  state_ = v;
  out = {v, t_cmd_, t, m};
  return true;
}

}  // namespace helix_monitor
