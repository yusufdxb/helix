// Verifies that a commanded stop actually happened, using independent evidence groups.
//
// An acknowledged command proves nothing about motion, and the state estimator is a single
// source that can be wrong (or stale) while the robot keeps moving. VERIFIED therefore needs
// agreement from at least min_groups groups, one of them a raw sensor, and no group showing motion.
#pragma once

#include <deque>
#include <string>
#include <vector>

#include "helix_monitor/checks.hpp"

namespace helix_monitor {

struct EvidenceGroup {
  std::string name;
  std::string stream;
  std::vector<std::size_t> channels;
  std::vector<double> scale;      // per channel; the group signal uses scaled values
  bool raw = true;                // raw sensor (encoders, IMU) vs derived estimate
  bool use_max = false;           // false: window RMS of the scaled vector norm; true: window max |scaled|
  bool detrend = false;           // subtract the window mean first (e.g. gravity in accelerometers)
  double quiet = 1.0;             // metric below: quiet
  double moving = 3.0;            // metric above: moving
};

enum class Verdict { kNone, kPending, kVerified, kFailed, kUnverified };
const char* to_string(Verdict v);

struct StopResult {
  Verdict verdict = Verdict::kNone;
  double t_cmd = 0.0;
  double t = 0.0;                           // decision time
  std::vector<double> metric;               // per group, at decision time (NaN = no data)
};

class StopVerifier {
 public:
  struct Params { double window_s = 0.5, deadline_s = 2.0, stale_s = 0.2; int min_groups = 2; bool require_raw = true; };
  StopVerifier(std::vector<EvidenceGroup> groups, Params p);

  void command(double t);                                   // a stop was commanded at t
  void on_sample(const std::string& stream, const Sample& s);
  // Advances the decision; returns true and fills `out` when a pending stop resolves.
  bool tick(double t, StopResult& out);
  Verdict state() const { return state_; }
  const std::vector<EvidenceGroup>& groups() const { return groups_; }
  double metric(std::size_t g, double t) const;            // NaN if stale or not enough data

 private:
  struct Buf { std::deque<std::pair<double, std::vector<double>>> w; double last = -1.0; };
  std::vector<EvidenceGroup> groups_;
  Params p_;
  std::vector<Buf> bufs_;
  Verdict state_ = Verdict::kNone;
  double t_cmd_ = 0.0;
};

}  // namespace helix_monitor
