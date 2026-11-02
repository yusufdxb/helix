// HELIX v2 monitor core: ROS-free checks over timestamped sample streams.
//
// Every check has fixed thresholds (calibrated offline on development recordings).
// No check adapts its baseline online, so a persistent fault cannot teach the
// monitor that the fault is normal.
#pragma once

#include <array>
#include <cstddef>
#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace helix_monitor {

struct Sample {
  double t = 0.0;               // receive time, seconds
  double stamp = 0.0;           // producer stamp (header stamp or tick), seconds
  std::vector<double> v;        // payload fields
};

struct Event {
  double t = 0.0;
  std::string check;            // e.g. "rate", "frozen", "residual"
  std::string stream;           // stream (or stream pair) name
  bool raised = false;          // true = raised, false = cleared
  double value = 0.0;           // statistic that triggered the transition
};

// Raise after the condition holds for hold_s, clear after it has been false for clear_s.
class Latch {
 public:
  Latch(double hold_s, double clear_s) : hold_s_(hold_s), clear_s_(clear_s) {}
  // Returns +1 on raise, -1 on clear, 0 otherwise.
  int update(double t, bool condition);
  bool active() const { return active_; }

 private:
  double hold_s_, clear_s_;
  bool active_ = false;
  double since_ = -1.0;         // start of the current run of the opposite condition
};

class Check {
 public:
  Check(std::string name, std::string stream) : name_(std::move(name)), stream_(std::move(stream)) {}
  virtual ~Check() = default;
  virtual bool wants(const std::string& stream) const { return stream == stream_; }
  virtual void on_sample(const std::string& stream, const Sample& s, std::vector<Event>& out) = 0;
  virtual void tick(double /*t*/, std::vector<Event>& /*out*/) {}
  virtual double statistic() const { return stat_max_; }   // worst statistic seen (calibration)
  const std::string& name() const { return name_; }
  const std::string& stream() const { return stream_; }

 protected:
  void emit(std::vector<Event>& out, double t, int edge, double value) {
    if (edge != 0) out.push_back({t, name_, stream_, edge > 0, value});
  }
  std::string name_, stream_;
  double stat_max_ = 0.0;
};

// FrequencyStatus-style rate band over a sliding window plus a silence timeout.
class RateCheck : public Check {
 public:
  struct Params { double window_s = 1.0, min_hz = 0.0, max_hz = 1e9, timeout_s = 0.5, clear_s = 0.5; };
  RateCheck(std::string stream, Params p) : Check("rate", std::move(stream)), p_(p), latch_(0.0, p.clear_s) {}
  void on_sample(const std::string& stream, const Sample& s, std::vector<Event>& out) override;
  void tick(double t, std::vector<Event>& out) override;
  double min_rate() const { return min_rate_; }
  double max_rate() const { return max_rate_; }
  double max_gap() const { return max_gap_; }

 private:
  void evaluate(double t, std::vector<Event>& out);
  Params p_;
  Latch latch_;
  std::deque<double> times_;
  double first_ = -1.0, last_ = -1.0;
  double min_rate_ = 1e18, max_rate_ = 0.0, max_gap_ = 0.0;
};

// Payload identical (bit for bit) for frozen_s while messages keep arriving.
class FrozenCheck : public Check {
 public:
  struct Params { double frozen_s = 0.2; std::size_t min_count = 3; };
  FrozenCheck(std::string stream, Params p) : Check("frozen", std::move(stream)), p_(p), latch_(0.0, 0.0) {}
  void on_sample(const std::string& stream, const Sample& s, std::vector<Event>& out) override;

 private:
  Params p_;
  Latch latch_;
  std::vector<double> last_;
  double run_start_ = -1.0;
  std::size_t run_count_ = 0;
};

// Producer stamp not advancing for stall_s while messages keep arriving.
class StampCheck : public Check {
 public:
  struct Params { double stall_s = 0.2; };
  StampCheck(std::string stream, Params p) : Check("stamp", std::move(stream)), p_(p), latch_(0.0, 0.0) {}
  void on_sample(const std::string& stream, const Sample& s, std::vector<Event>& out) override;

 private:
  Params p_;
  Latch latch_;
  bool have_ = false;
  double last_stamp_ = 0.0, stall_start_ = -1.0;
};

// Per-channel value bounds (the conventional plausibility check).
class RangeCheck : public Check {
 public:
  struct Params { std::vector<std::size_t> channels; std::vector<double> lo, hi; double hold_s = 0.0, clear_s = 0.5; };
  RangeCheck(std::string stream, Params p) : Check("range", std::move(stream)), p_(std::move(p)), latch_(p_.hold_s, p_.clear_s) {}
  void on_sample(const std::string& stream, const Sample& s, std::vector<Event>& out) override;

 private:
  Params p_;
  Latch latch_;
};

// Cross-sensor residual: two sensors observing the same physical quantity.
// residual = || lp(a) - (R * lp(b) + d) ||, lp = continuous-time first-order low-pass.
class ResidualCheck : public Check {
 public:
  struct Params {
    std::array<std::size_t, 3> a_channels{0, 1, 2}, b_channels{0, 1, 2};
    std::array<double, 9> R{1, 0, 0, 0, 1, 0, 0, 0, 1};
    std::array<double, 3> d{0, 0, 0};
    double tau_s = 0.1, threshold = 0.1, hold_s = 0.2, clear_s = 0.5, stale_s = 0.1;
  };
  ResidualCheck(std::string a, std::string b, Params p)
      : Check("residual", a + "|" + b), a_(std::move(a)), b_(std::move(b)), p_(p), latch_(p.hold_s, p.clear_s) {}
  void on_sample(const std::string& stream, const Sample& s, std::vector<Event>& out) override;
  bool wants(const std::string& stream) const override { return stream == a_ || stream == b_; }
  const std::string& a() const { return a_; }
  const std::string& b() const { return b_; }
  // Worst residual that persisted for hold_s (the quantity the latch thresholds).
  double statistic() const override { return held_max_; }

 private:
  struct Lp { std::array<double, 3> y{}; double t = -1.0; };
  static void lp_update(Lp& lp, double t, const std::array<double, 3>& x, double tau);
  std::string a_, b_;
  Params p_;
  Latch latch_;
  Lp la_, lb_;
  std::deque<std::pair<double, double>> recent_;  // (t, residual) over the last hold_s
  double held_max_ = 0.0;
};

// Routes samples to the checks registered for each stream.
class Monitor {
 public:
  void add(std::unique_ptr<Check> c);
  void on_sample(const std::string& stream, const Sample& s, std::vector<Event>& out);
  void tick(double t, std::vector<Event>& out);
  const std::vector<std::unique_ptr<Check>>& checks() const { return checks_; }

 private:
  std::vector<std::unique_ptr<Check>> checks_;
};

}  // namespace helix_monitor
