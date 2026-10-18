// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// RateWindow contract, mirrored from helix_adapter/test/test_rate_window.py
// plus the stale_after_s extension.

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "helix_sensing_cpp/rate_window.hpp"

using helix_sensing_cpp::RateWindow;

TEST(RateWindow, EmptyIsNan)
{
  RateWindow w(5.0);
  EXPECT_TRUE(std::isnan(w.rate_or_nan(0.0)));
}

TEST(RateWindow, OneSampleIsZero)
{
  RateWindow w(5.0);
  w.record(1.0);
  EXPECT_DOUBLE_EQ(w.rate_or_nan(1.0), 0.0);
}

TEST(RateWindow, SteadyRate)
{
  RateWindow w(5.0);
  for (int i = 0; i <= 50; ++i) {
    w.record(i * 0.1);
  }
  EXPECT_NEAR(w.rate_or_nan(5.0), 10.0, 1e-9);
}

TEST(RateWindow, EvictsOldSamples)
{
  RateWindow w(1.0);
  for (int i = 0; i <= 20; ++i) {
    w.record(i * 0.1);   // 0.0 .. 2.0
  }
  // window [1.0, 2.0]: 11 samples over 1.0 s
  EXPECT_NEAR(w.rate_or_nan(2.0), 10.0, 1e-9);
  EXPECT_TRUE(std::isnan(w.rate_or_nan(3.5)));
}

TEST(RateWindow, ReferenceKeepsOldRateWhileDraining)
{
  // Reference behavior: a dead topic still reads ~10 Hz until the window
  // has drained, because (n-1)/span is unchanged by eviction.
  RateWindow w(5.0);
  for (int i = 0; i <= 50; ++i) {
    w.record(i * 0.1);
  }
  EXPECT_NEAR(w.rate_or_nan(8.0), 10.0, 1e-9);
}

TEST(RateWindow, StaleAfterReportsNanEarly)
{
  RateWindow w(5.0, 0.5);
  for (int i = 0; i <= 50; ++i) {
    w.record(i * 0.1);
  }
  EXPECT_NEAR(w.rate_or_nan(5.4), 10.0, 1e-9);
  EXPECT_TRUE(std::isnan(w.rate_or_nan(5.6)));
}

TEST(RateWindow, RejectsBadParams)
{
  EXPECT_THROW(RateWindow(0.0), std::invalid_argument);
  EXPECT_THROW(RateWindow(1.0, -1.0), std::invalid_argument);
}
