// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// AnomalyCore extensions beyond the Python reference (all off by default,
// which test_python_parity covers): frozen baseline, relative std floor,
// clear records, and the anchored expected-rate band.

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "helix_sensing_cpp/anomaly_core.hpp"

using helix_sensing_cpp::AnomalyCore;
using helix_sensing_cpp::AnomalyParams;

namespace
{

constexpr double kDt = 0.5;   // topic_rate_monitor publish period

AnomalyParams chosen()
{
  // The config chosen on DEV in analysis/detector_eval.py.
  AnomalyParams p;
  p.zscore_threshold = 4.0;
  p.window_size = 60;
  p.consecutive_trigger = 3;
  p.min_anomaly_duration_s = 1.0;
  p.duration_tolerance_s = 0.25;
  p.emit_cooldown_s = 1.0;
  p.freeze_baseline = true;
  p.std_floor_rel = 0.02;
  p.clear_samples = 4;
  p.anchor_tolerance_rel = 0.1;
  return p;
}

struct Counts
{
  int faults{0};
  int clears{0};
  double first_fault_t{-1.0};
  double first_clear_t{-1.0};
};

// Feed value(t) for t in [t0, t1) and count records.
template<typename F>
Counts feed(AnomalyCore & c, const std::string & metric, double t0, double t1, F value)
{
  Counts n;
  for (double t = t0; t < t1 - 1e-9; t += kDt) {
    const auto r = c.process(metric, value(t), t, t);
    if (r.fault) {
      if (n.faults++ == 0) {
        n.first_fault_t = t;
      }
    }
    if (r.clear && r.clear->context_values.at(1) == "recovered") {
      if (n.clears++ == 0) {
        n.first_clear_t = t;
      }
    }
  }
  return n;
}

// 15 Hz with small deterministic jitter, like /utlidar/cloud.
double cloud(double t) {return 15.0 + 0.05 * std::sin(t * 1.7);}

// 15 Hz with about 2% std: wide enough that an 8% deficit is inside the
// z clear band, so only the anchor's hysteresis decides.
double noisy(double t) {return 15.0 + 0.45 * std::sin(t * 1.7);}

}  // namespace

TEST(AnomalyCore, FlatWindowIsEvaluableWithStdFloor)
{
  AnomalyParams p = chosen();
  p.anchor_tolerance_rel = 0.0;
  AnomalyCore c(p);
  feed(c, "rate_hz/utlidar_cloud", 0.0, 70.0, [](double) {return 15.0;});
  // Without the floor a perfectly flat window is never judged (kFlat).
  const auto n = feed(c, "rate_hz/utlidar_cloud", 70.0, 80.0, [](double) {return 7.5;});
  EXPECT_GE(n.faults, 1);
}

TEST(AnomalyCore, FrozenBaselineKeepsSustainedFaultActive)
{
  AnomalyParams p = chosen();
  p.anchor_tolerance_rel = 0.0;
  AnomalyCore c(p);
  feed(c, "rate_hz/utlidar_cloud", 0.0, 70.0, cloud);
  // A sustained 50% collapse: with the reference's always-push window the
  // violating samples become the baseline and the fault reads as recovered.
  const auto n = feed(c, "rate_hz/utlidar_cloud", 70.0, 200.0, [](double) {return 7.5;});
  EXPECT_GE(n.faults, 1);
  EXPECT_EQ(n.clears, 0);
  // Return to normal: one recovered clear after clear_samples.
  const auto back = feed(c, "rate_hz/utlidar_cloud", 200.0, 210.0, cloud);
  EXPECT_EQ(back.clears, 1);
  EXPECT_NEAR(back.first_clear_t, 200.0 + (p.clear_samples - 1) * kDt, 1e-9);
}

TEST(AnomalyCore, ReferenceWindowSelfMasks)
{
  AnomalyParams p;   // reference defaults, plus clear records to observe it
  p.clear_samples = 4;
  AnomalyCore c(p);
  feed(c, "rate_hz/utlidar_cloud", 0.0, 40.0, cloud);
  const auto n = feed(c, "rate_hz/utlidar_cloud", 40.0, 200.0, [](double) {return 7.5;});
  EXPECT_GE(n.faults, 1);
  EXPECT_GE(n.clears, 1) << "expected the documented self-masking false clear";
}

TEST(AnomalyCore, AnchorCatchesSlowDriftThatZMisses)
{
  auto drift = [](double t) {
      return t < 70.0 ? cloud(t) : cloud(t) * (1.0 - 0.3 * std::min(t - 70.0, 300.0) / 300.0);
    };
  AnomalyParams p = chosen();
  p.anchor_tolerance_rel = 0.0;
  AnomalyCore no_anchor(p);
  EXPECT_EQ(feed(no_anchor, "rate_hz/utlidar_cloud", 0.0, 400.0, drift).faults, 0);

  AnomalyCore anchored(chosen());
  const auto n = feed(anchored, "rate_hz/utlidar_cloud", 0.0, 400.0, drift);
  ASSERT_GE(n.faults, 1);
  // 10% band crossed at 100 s into the 300 s ramp, then the 1 s duration gate.
  EXPECT_NEAR(n.first_fault_t, 70.0 + 100.0 + 1.0, 1.5);
  EXPECT_EQ(n.clears, 0);
}

TEST(AnomalyCore, AnchorClearNeedsHalfBand)
{
  AnomalyCore c(chosen());
  feed(c, "rate_hz/utlidar_cloud", 0.0, 70.0, noisy);
  EXPECT_GE(feed(c, "rate_hz/utlidar_cloud", 70.0, 80.0, [](double) {return 12.0;}).faults, 1);
  // 8% low: z is inside the clear band and the value inside the 10% band,
  // but outside half of it, so the fault must not clear.
  EXPECT_EQ(feed(c, "rate_hz/utlidar_cloud", 80.0, 120.0, [](double) {return 13.8;}).clears, 0);
  EXPECT_EQ(feed(c, "rate_hz/utlidar_cloud", 120.0, 130.0, noisy).clears, 1);
}

TEST(AnomalyCore, AnchorOnlyAppliesToPrefixedMetrics)
{
  AnomalyCore c(chosen());
  auto pose = [](double t) {return 0.002 + 0.0005 * std::sin(t);};
  feed(c, "pose/displacement_rate_m_s", 0.0, 70.0, pose);
  // A 30% change on a near-zero metric is not an anchored violation.
  const auto n = feed(
    c, "pose/displacement_rate_m_s", 70.0, 200.0, [&](double t) {return pose(t) * 1.3;});
  EXPECT_EQ(n.faults, 0);
}

TEST(AnomalyCore, StaleNanStillFaults)
{
  AnomalyCore c(chosen());
  feed(c, "rate_hz/utlidar_cloud", 0.0, 70.0, cloud);
  const auto n = feed(c, "rate_hz/utlidar_cloud", 70.0, 80.0, [](double) {return std::nan("");});
  ASSERT_GE(n.faults, 1);
  EXPECT_LE(n.first_fault_t - 70.0, 1.0 + 0.25 + kDt);
}
