#include "helix_monitor/checks.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace helix_monitor {

int Latch::update(double t, bool condition) {
  if (condition == active_) {           // nothing pending
    since_ = -1.0;
    return 0;
  }
  if (since_ < 0.0) since_ = t;
  const double need = active_ ? clear_s_ : hold_s_;
  if (t - since_ >= need) {
    active_ = condition;
    since_ = -1.0;
    return active_ ? 1 : -1;
  }
  return 0;
}

// ---------------------------------------------------------------- RateCheck
void RateCheck::on_sample(const std::string& /*stream*/, const Sample& s, std::vector<Event>& out) {
  if (last_ >= 0.0) max_gap_ = std::max(max_gap_, s.t - last_);
  if (first_ < 0.0) first_ = s.t;
  last_ = s.t;
  times_.push_back(s.t);
  evaluate(s.t, out);
}

void RateCheck::tick(double t, std::vector<Event>& out) { evaluate(t, out); }

void RateCheck::evaluate(double t, std::vector<Event>& out) {
  if (first_ < 0.0) return;             // a stream that never appeared is not monitored
  while (!times_.empty() && times_.front() <= t - p_.window_s) times_.pop_front();
  const double gap = t - last_;
  bool bad = gap > p_.timeout_s;
  double value = gap;
  if (t - first_ >= p_.window_s) {      // window filled once
    const double rate = static_cast<double>(times_.size()) / p_.window_s;
    if (gap <= p_.timeout_s) {
      min_rate_ = std::min(min_rate_, rate);
      max_rate_ = std::max(max_rate_, rate);
    }
    if (rate < p_.min_hz || rate > p_.max_hz) {
      bad = true;
      value = rate;
    }
  }
  emit(out, t, latch_.update(t, bad), value);
}

// -------------------------------------------------------------- FrozenCheck
void FrozenCheck::on_sample(const std::string& /*stream*/, const Sample& s, std::vector<Event>& out) {
  const bool same = !last_.empty() && last_.size() == s.v.size() &&
                    std::memcmp(last_.data(), s.v.data(), s.v.size() * sizeof(double)) == 0;
  if (same) {
    ++run_count_;
  } else {
    run_start_ = s.t;
    run_count_ = 1;
    last_ = s.v;
  }
  const double run = s.t - run_start_;
  if (run_count_ >= 2) stat_max_ = std::max(stat_max_, run);
  const bool bad = run_count_ >= p_.min_count && run >= p_.frozen_s;
  emit(out, s.t, latch_.update(s.t, bad), run);
}

// --------------------------------------------------------------- StampCheck
void StampCheck::on_sample(const std::string& /*stream*/, const Sample& s, std::vector<Event>& out) {
  bool stalled = false;
  if (have_ && s.stamp <= last_stamp_) {
    if (stall_start_ < 0.0) stall_start_ = s.t;
    stalled = true;
  } else {
    stall_start_ = -1.0;
    last_stamp_ = s.stamp;
  }
  have_ = true;
  const double run = stalled ? s.t - stall_start_ : 0.0;
  stat_max_ = std::max(stat_max_, run);
  emit(out, s.t, latch_.update(s.t, stalled && run >= p_.stall_s), run);
}

// --------------------------------------------------------------- RangeCheck
void RangeCheck::on_sample(const std::string& /*stream*/, const Sample& s, std::vector<Event>& out) {
  double worst = 0.0;                   // largest excursion beyond a bound, in channel units
  for (std::size_t i = 0; i < p_.channels.size(); ++i) {
    const std::size_t c = p_.channels[i];
    if (c >= s.v.size()) continue;
    const double x = s.v[c];
    if (!std::isfinite(x)) { worst = 1e18; break; }
    worst = std::max({worst, p_.lo[i] - x, x - p_.hi[i]});
  }
  stat_max_ = std::max(stat_max_, worst);
  emit(out, s.t, latch_.update(s.t, worst > 0.0), worst);
}

// ------------------------------------------------------------ ResidualCheck
void ResidualCheck::lp_update(Lp& lp, double t, const std::array<double, 3>& x, double tau) {
  if (lp.t < 0.0 || t - lp.t > 10.0 * tau) {
    lp.y = x;
  } else {
    const double a = 1.0 - std::exp(-(t - lp.t) / tau);
    for (int i = 0; i < 3; ++i) lp.y[i] += a * (x[i] - lp.y[i]);
  }
  lp.t = t;
}

void ResidualCheck::on_sample(const std::string& stream, const Sample& s, std::vector<Event>& out) {
  const bool is_a = stream == a_;
  const auto& ch = is_a ? p_.a_channels : p_.b_channels;
  std::array<double, 3> x{};
  for (int i = 0; i < 3; ++i) {
    if (ch[i] >= s.v.size()) return;
    x[i] = s.v[ch[i]];
  }
  lp_update(is_a ? la_ : lb_, s.t, x, p_.tau_s);
  if (la_.t < 0.0 || lb_.t < 0.0) return;
  if (s.t - la_.t > p_.stale_s || s.t - lb_.t > p_.stale_s) {   // one side silent: rate check owns that
    recent_.clear();
    return;
  }
  const auto& R = p_.R;
  double r2 = 0.0;
  for (int i = 0; i < 3; ++i) {
    const double pred = R[3 * i] * lb_.y[0] + R[3 * i + 1] * lb_.y[1] + R[3 * i + 2] * lb_.y[2] + p_.d[i];
    const double e = la_.y[i] - pred;
    r2 += e * e;
  }
  const double r = std::isfinite(r2) ? std::sqrt(r2) : 1e18;
  recent_.emplace_back(s.t, r);
  while (!recent_.empty() && recent_.front().first < s.t - p_.hold_s) recent_.pop_front();
  if (recent_.size() >= 2 && s.t - recent_.front().first >= 0.9 * p_.hold_s) {
    double held = 1e18;
    for (const auto& e : recent_) held = std::min(held, e.second);
    held_max_ = std::max(held_max_, held);
  }
  // Hysteresis: raise above threshold for hold_s, stay raised until below 0.7 * threshold for clear_s.
  const bool cond = latch_.active() ? r > 0.7 * p_.threshold : r > p_.threshold;
  emit(out, s.t, latch_.update(s.t, cond), r);
}

// ------------------------------------------------------------------ Monitor
void Monitor::add(std::unique_ptr<Check> c) { checks_.push_back(std::move(c)); }

void Monitor::on_sample(const std::string& stream, const Sample& s, std::vector<Event>& out) {
  for (auto& c : checks_)
    if (c->wants(stream)) c->on_sample(stream, s, out);
}

void Monitor::tick(double t, std::vector<Event>& out) {
  for (auto& c : checks_) c->tick(t, out);
}

}  // namespace helix_monitor
