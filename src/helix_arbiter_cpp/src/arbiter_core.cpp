// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// See include/helix_arbiter_cpp/arbiter_core.hpp for the policy contract.

#include "helix_arbiter_cpp/arbiter_core.hpp"

#include <cmath>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace helix_arbiter_cpp
{

const char * reason_name(Reason reason) noexcept
{
  switch (reason) {
    case Reason::kSource: return "SOURCE";
    case Reason::kHold: return "HELIX_HOLD";
    case Reason::kStale: return "HELIX_STATE_STALE";
    case Reason::kMissing: return "HELIX_STATE_MISSING";
    case Reason::kNoInput: return "NO_LIVE_INPUT";
    case Reason::kShutdown: return "SHUTDOWN";
  }
  return "UNKNOWN";
}

bool is_helix_forced(Reason reason) noexcept
{
  return reason == Reason::kHold || reason == Reason::kStale || reason == Reason::kMissing;
}

const char * rejection_name(Rejection why) noexcept
{
  switch (why) {
    case Rejection::kNone: return "accepted";
    case Rejection::kNonFinite: return "non-finite component";
    case Rejection::kLinearOverLimit: return "linear over limit";
    case Rejection::kAngularOverLimit: return "angular over limit";
  }
  return "unknown";
}

std::optional<Command> validate_twist(
  const Twist6 & in, const Limits & limits, Rejection * why) noexcept
{
  auto reject = [why](Rejection r) -> std::optional<Command> {
      if (why != nullptr) {
        *why = r;
      }
      return std::nullopt;
    };
  for (double v : {in.lx, in.ly, in.lz, in.ax, in.ay, in.az}) {
    if (!std::isfinite(v)) {
      return reject(Rejection::kNonFinite);
    }
  }
  if (std::fabs(in.lx) > limits.max_abs_linear || std::fabs(in.ly) > limits.max_abs_linear) {
    return reject(Rejection::kLinearOverLimit);
  }
  if (std::fabs(in.az) > limits.max_abs_angular) {
    return reject(Rejection::kAngularOverLimit);
  }
  if (why != nullptr) {
    *why = Rejection::kNone;
  }
  // + 0.0 turns -0.0 into +0.0 (round-to-nearest), as in the Python core, so
  // is_zero and equality are exact. Valid without -ffast-math, which this
  // package never enables.
  return Command{in.lx + 0.0, in.ly + 0.0, in.az + 0.0};
}

Arbiter::Arbiter(std::vector<SourceSpec> sources, double hold_timeout_sec, Limits limits)
: hold_timeout_sec_(hold_timeout_sec), limits_(limits)
{
  if (sources.empty()) {
    throw std::invalid_argument("arbiter needs at least one source");
  }
  std::set<std::string> names;
  for (const auto & s : sources) {
    if (s.name.empty()) {
      throw std::invalid_argument("source names must be non-empty");
    }
    if (!names.insert(s.name).second) {
      throw std::invalid_argument("duplicate source name '" + s.name + "'");
    }
    // twist_mux treats timeout 0 as "never expires"; that would let a dead
    // source keep authority forever, so it is refused here. NaN and +inf are
    // refused for the same reason.
    if (!std::isfinite(s.timeout_sec) || s.timeout_sec <= 0.0) {
      throw std::invalid_argument("source '" + s.name + "' needs a finite timeout > 0");
    }
  }
  if (!std::isfinite(hold_timeout_sec) || hold_timeout_sec <= 0.0) {
    throw std::invalid_argument("hold_timeout_sec must be finite and > 0");
  }
  // A NaN limit would disable the bound (every comparison is false), and an
  // infinite one is no bound at all.
  if (!std::isfinite(limits.max_abs_linear) || limits.max_abs_linear < 0.0 ||
    !std::isfinite(limits.max_abs_angular) || limits.max_abs_angular < 0.0)
  {
    throw std::invalid_argument("velocity limits must be finite and >= 0");
  }
  slots_.reserve(sources.size());
  for (auto & s : sources) {
    slots_.push_back(Slot{std::move(s), std::nullopt, 0.0, 0, 0});
  }
}

std::optional<std::size_t> Arbiter::index_of(std::string_view name) const noexcept
{
  for (std::size_t i = 0; i < slots_.size(); ++i) {
    if (slots_[i].spec.name == name) {
      return i;
    }
  }
  return std::nullopt;
}

bool Arbiter::on_source(std::size_t index, const Twist6 & twist, double now, Rejection * why)
{
  Slot & slot = slots_.at(index);
  const std::optional<Command> cmd = validate_twist(twist, limits_, why);
  if (!cmd) {
    slot.command.reset();  // P4: never fall back to an older value
    ++counters_.rejected;
    ++slot.rejected;
    return false;
  }
  ++order_;
  slot.command = cmd;
  slot.received_at = now;
  slot.order = order_;
  return true;
}

bool Arbiter::on_hold(
  bool hold, std::string_view fault_id, std::uint64_t epoch, std::uint64_t seq, double now)
{
  if (hold_ && hold_fresh(now)) {
    // Lexicographic (epoch, seq) <= (cur.epoch, cur.seq) on full-width
    // unsigned integers, exactly like the Python tuple comparison on ints.
    const bool not_newer = epoch < hold_->epoch || (epoch == hold_->epoch && seq <= hold_->seq);
    if (not_newer) {
      ++counters_.hold_reordered;
      return false;
    }
  }
  const bool prev_effective = effective_hold(now);
  hold_ = HoldState{hold, std::string(fault_id), epoch, seq, now};
  if (effective_hold(now) != prev_effective) {
    clear_sources();  // P7
    ++counters_.hold_transitions;
  }
  return true;
}

Decision Arbiter::decide(double now)
{
  if (effective_hold(now)) {
    // Staleness is also a hold transition: clear sources so a command stored
    // before the state went stale cannot resume motion later.
    clear_sources();
  }
  Decision d;
  if (!hold_) {
    d.reason = Reason::kMissing;
    return d;
  }
  if (!hold_fresh(now)) {
    d.reason = Reason::kStale;
    d.hold_fault_id = hold_->fault_id;
    return d;
  }
  if (hold_->hold) {
    d.reason = Reason::kHold;
    d.hold_fault_id = hold_->fault_id;
    return d;
  }
  const Slot * win = nullptr;
  for (const Slot & s : slots_) {
    if (!source_fresh(s, now)) {
      continue;
    }
    // max by (priority, receipt order): priority first, most recent receipt
    // breaks ties, like twist_mux. Receipt orders are unique.
    if (win == nullptr || s.spec.priority > win->spec.priority ||
      (s.spec.priority == win->spec.priority && s.order > win->order))
    {
      win = &s;
    }
  }
  if (win == nullptr) {
    d.reason = Reason::kNoInput;
    return d;
  }
  d.command = *win->command;
  d.reason = Reason::kSource;
  d.source = win->spec.name;
  return d;
}

void Arbiter::reset() noexcept
{
  clear_sources();
  hold_.reset();
}

bool Arbiter::hold_fresh(double now) const noexcept
{
  return hold_.has_value() && (now - hold_->received_at) <= hold_timeout_sec_;
}

bool Arbiter::effective_hold(double now) const noexcept
{
  return !hold_.has_value() || !hold_fresh(now) || hold_->hold;
}

bool Arbiter::source_fresh(const Slot & slot, double now) noexcept
{
  return slot.command.has_value() && (now - slot.received_at) <= slot.spec.timeout_sec;
}

void Arbiter::clear_sources() noexcept
{
  for (Slot & s : slots_) {
    s.command.reset();
  }
}

}  // namespace helix_arbiter_cpp
