// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// Unit tests for the header-only RollingStats kernel. Mirrors the
// behavioral spec in helix_core.anomaly_detector._process_sample:
// population variance (divide by N), z-score BEFORE push, flat-signal
// guard at 1e-6, capacity-bounded FIFO.

#include <cmath>
#include <stdexcept>

#include "gtest/gtest.h"

#include "helix_sensing_cpp/rolling_stats.hpp"

using helix_sensing_cpp::RollingStats;
using helix_sensing_cpp::ZScoreStatus;

TEST(RollingStats, RejectsZeroWindow)
{
  EXPECT_THROW(RollingStats(0), std::invalid_argument);
}

TEST(RollingStats, EmptyEvaluatesInsufficient)
{
  RollingStats rs(10);
  auto r = rs.evaluate(42.0);
  EXPECT_EQ(r.status, ZScoreStatus::kInsufficient);
}

TEST(RollingStats, SingleSampleEvaluatesInsufficient)
{
  RollingStats rs(10);
  rs.push(1.0);
  auto r = rs.evaluate(42.0);
  EXPECT_EQ(r.status, ZScoreStatus::kInsufficient);
}

TEST(RollingStats, FlatWindowIsFlat)
{
  RollingStats rs(10);
  // All samples identical -> std == 0 -> flat.
  for (int i = 0; i < 5; ++i) {
    rs.push(7.0);
  }
  auto r = rs.evaluate(42.0);
  EXPECT_EQ(r.status, ZScoreStatus::kFlat);
  EXPECT_DOUBLE_EQ(r.mean, 7.0);
  EXPECT_DOUBLE_EQ(r.std, 0.0);
}

TEST(RollingStats, PopulationVarianceNotSample)
{
  // samples {1, 2, 3}: mean 2.0
  // population var = ((1-2)^2 + (2-2)^2 + (3-2)^2) / 3 = 2/3
  // NOT sample var (which would divide by 2 and give 1.0)
  RollingStats rs(10);
  rs.push(1.0);
  rs.push(2.0);
  rs.push(3.0);
  auto r = rs.evaluate(2.0);
  EXPECT_EQ(r.status, ZScoreStatus::kOk);
  EXPECT_DOUBLE_EQ(r.mean, 2.0);
  EXPECT_NEAR(r.std, std::sqrt(2.0 / 3.0), 1e-12);
}

TEST(RollingStats, ZScoreMatchesPythonFormula)
{
  // Python: zscore = abs((x - mean) / std)
  RollingStats rs(10);
  // Window {10, 10.1, 10.2} -> mean 10.1, pop-std = sqrt((.01+0+.01)/3)
  rs.push(10.0);
  rs.push(10.1);
  rs.push(10.2);
  auto r = rs.evaluate(100.0);
  EXPECT_EQ(r.status, ZScoreStatus::kOk);
  const double expected_std = std::sqrt((0.01 + 0.0 + 0.01) / 3.0);
  EXPECT_NEAR(r.std, expected_std, 1e-12);
  const double expected_z = std::fabs((100.0 - 10.1) / expected_std);
  EXPECT_NEAR(r.zscore, expected_z, 1e-9);
}

TEST(RollingStats, EvaluateDoesNotMutateWindow)
{
  RollingStats rs(10);
  rs.push(1.0);
  rs.push(2.0);
  rs.push(3.0);
  const auto before = rs.size();
  (void)rs.evaluate(999.0);
  EXPECT_EQ(rs.size(), before);
}

TEST(RollingStats, PushRespectsCapacity)
{
  RollingStats rs(3);
  for (int i = 0; i < 10; ++i) {
    rs.push(static_cast<double>(i));
  }
  EXPECT_EQ(rs.size(), 3u);
  // Last three values pushed were 7, 8, 9; mean = 8
  auto r = rs.evaluate(8.0);
  EXPECT_EQ(r.status, ZScoreStatus::kOk);
  EXPECT_DOUBLE_EQ(r.mean, 8.0);
}

TEST(RollingStats, FlatSignalEpsilonBoundary)
{
  // std just above 1e-6 -> kOk, just below -> kFlat.
  RollingStats rs_ok(10);
  // Values spaced so pop-std >~ 1e-6:
  // two samples {0, 2e-6} -> mean 1e-6, pop var = 2 * (1e-6)^2 / 2 = 1e-12
  // pop std = 1e-6 exactly; evaluate treats "<" epsilon as flat, so this
  // should be kOk (std == epsilon, not < epsilon).
  rs_ok.push(0.0);
  rs_ok.push(2e-6);
  auto r_ok = rs_ok.evaluate(0.5);
  EXPECT_EQ(r_ok.status, ZScoreStatus::kOk);

  RollingStats rs_flat(10);
  // Two samples {0, 2e-7} -> pop std = 1e-7, < epsilon -> flat.
  rs_flat.push(0.0);
  rs_flat.push(2e-7);
  auto r_flat = rs_flat.evaluate(0.5);
  EXPECT_EQ(r_flat.status, ZScoreStatus::kFlat);
}

TEST(RollingStats, Clear)
{
  RollingStats rs(5);
  rs.push(1.0); rs.push(2.0); rs.push(3.0);
  rs.clear();
  EXPECT_EQ(rs.size(), 0u);
  auto r = rs.evaluate(1.0);
  EXPECT_EQ(r.status, ZScoreStatus::kInsufficient);
}
