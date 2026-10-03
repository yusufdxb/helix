// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// Pure motion-arbitration core. No ROS includes; every safety policy is here.
//
// C++ port of helix_arbiter/arbiter_core.py, which stays the reference
// implementation. The policy table (P1..P10) is documented in
// docs/MOTION_ARBITRATION.md and the two implementations are compared
// decision by decision by test/test_python_parity.py.
//
// P1  HELIX hold asserted                     -> output ZERO
// P2  HELIX state never received              -> output ZERO  (HELIX_STATE_MISSING)
// P3  HELIX state older than hold_timeout     -> output ZERO  (HELIX_STATE_STALE)
// P4  Malformed input (NaN, Inf, over limit)  -> rejected, AND that source's
//     previous command is discarded, so a stale good value is never reused.
// P5  Source older than its timeout           -> dropped from arbitration.
// P6  No valid fresh source                   -> output ZERO  (NO_LIVE_INPUT)
// P7  Any hold transition (assert, release, stale) discards every stored
//     source command, so RESUME never replays a pre-hold command.
// P8  HELIX state ordered by (epoch, seq) as full-width unsigned 64-bit
//     integers; older or duplicate states are dropped while the current
//     state is fresh. Once stale, any epoch is accepted.
// P9  Only linear.x, linear.y, angular.z pass; all six axes must be finite.
// P10 Freshness uses the caller's monotonic receipt clock only.
//
// Times are seconds as double, exactly like time.monotonic() in the Python
// node, so boundary comparisons round identically on both sides.
//
// Not thread-safe: the node drives it from one single-threaded executor.
#ifndef HELIX_ARBITER_CPP__ARBITER_CORE_HPP_
#define HELIX_ARBITER_CPP__ARBITER_CORE_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace helix_arbiter_cpp
{

enum class Reason : std::uint8_t
{
  kSource,    // an upstream input won
  kHold,      // HELIX hold asserted
  kStale,     // HELIX state older than hold_timeout_sec
  kMissing,   // no HELIX state received yet
  kNoInput,   // released, but no valid fresh source
  kShutdown,  // deactivating or exiting
};

/// The ArbiterStatus.reason string for @p reason.
const char * reason_name(Reason reason) noexcept;

/// True whenever the zero is forced by HELIX (hold, stale or missing).
bool is_helix_forced(Reason reason) noexcept;

/// The three GO2-actionable axes. Everything else is forced to zero.
struct Command
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};

  bool is_zero() const noexcept {return vx == 0.0 && vy == 0.0 && wz == 0.0;}
};

inline bool operator==(const Command & a, const Command & b) noexcept
{
  return a.vx == b.vx && a.vy == b.vy && a.wz == b.wz;
}

inline bool operator!=(const Command & a, const Command & b) noexcept {return !(a == b);}

inline constexpr Command kZero{};

/// A full geometry_msgs/Twist worth of input, all six axes.
struct Twist6
{
  double lx{0.0};
  double ly{0.0};
  double lz{0.0};
  double ax{0.0};
  double ay{0.0};
  double az{0.0};
};

struct SourceSpec
{
  std::string name;
  std::string topic;
  std::int64_t priority{0};
  double timeout_sec{0.0};
};

struct Limits
{
  double max_abs_linear{1.0};   // m/s; larger commands are rejected, not clamped
  double max_abs_angular{1.5};  // rad/s
};

enum class Rejection : std::uint8_t
{
  kNone,
  kNonFinite,
  kLinearOverLimit,
  kAngularOverLimit,
};

const char * rejection_name(Rejection why) noexcept;

/// Mirror of arbiter_core.validate_twist: the Command if acceptable, else
/// std::nullopt with @p why set. Negative zero is normalised to +0.0.
std::optional<Command> validate_twist(
  const Twist6 & in, const Limits & limits, Rejection * why = nullptr) noexcept;

struct HoldState
{
  bool hold{false};
  std::string fault_id;
  std::uint64_t epoch{0};
  std::uint64_t seq{0};
  double received_at{0.0};
};

struct Decision
{
  Command command{};
  Reason reason{Reason::kMissing};
  std::string source;         // winning source name, empty unless kSource
  std::string hold_fault_id;  // fault behind a HELIX-forced zero

  bool helix_forced() const noexcept {return is_helix_forced(reason);}
};

struct Counters
{
  std::uint64_t rejected{0};
  std::uint64_t hold_reordered{0};
  std::uint64_t hold_transitions{0};
};

/// Deterministic arbiter; the caller supplies time on every call.
class Arbiter
{
public:
  /// Throws std::invalid_argument for every configuration that
  /// arbiter_core.Arbiter refuses: no sources, empty or duplicate names,
  /// a source timeout or hold timeout that is not finite and > 0, or a
  /// velocity limit that is not finite and >= 0.
  Arbiter(std::vector<SourceSpec> sources, double hold_timeout_sec, Limits limits = Limits{});

  std::size_t source_count() const noexcept {return slots_.size();}
  const SourceSpec & spec(std::size_t index) const {return slots_.at(index).spec;}
  /// Index of the source called @p name, or std::nullopt.
  std::optional<std::size_t> index_of(std::string_view name) const noexcept;

  /// Record a source message. Returns false if it was rejected (P4).
  bool on_source(std::size_t index, const Twist6 & twist, double now, Rejection * why = nullptr);

  /// Record a HELIX hold state. Returns false if dropped as out of order (P8).
  bool on_hold(
    bool hold, std::string_view fault_id, std::uint64_t epoch, std::uint64_t seq, double now);

  /// The output for time @p now. Not const: a HELIX-forced zero clears the
  /// stored source commands (P7).
  Decision decide(double now);

  /// Forget all inputs and HELIX state (lifecycle activate/deactivate).
  /// Counters are kept, as in the Python reference.
  void reset() noexcept;

  const Counters & counters() const noexcept {return counters_;}
  std::uint64_t rejected_by_source(std::size_t index) const {return slots_.at(index).rejected;}
  double hold_timeout_sec() const noexcept {return hold_timeout_sec_;}
  const Limits & limits() const noexcept {return limits_;}
  const std::optional<HoldState> & hold_state() const noexcept {return hold_;}

private:
  struct Slot
  {
    SourceSpec spec;
    std::optional<Command> command;
    double received_at{0.0};
    std::uint64_t order{0};
    std::uint64_t rejected{0};
  };

  bool hold_fresh(double now) const noexcept;
  bool effective_hold(double now) const noexcept;
  static bool source_fresh(const Slot & slot, double now) noexcept;
  void clear_sources() noexcept;

  std::vector<Slot> slots_;
  double hold_timeout_sec_;
  Limits limits_;
  std::optional<HoldState> hold_;
  std::uint64_t order_{0};
  Counters counters_;
};

}  // namespace helix_arbiter_cpp

#endif  // HELIX_ARBITER_CPP__ARBITER_CORE_HPP_
