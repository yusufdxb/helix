// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// C++ mirror of helix_adapter/test/test_rate_window.py. Same eight cases,
// same expected behavior. The C++ RateWindow and Python RateWindow are
// spec-equivalent; these tests keep them from drifting.

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "helix_adapter_cpp/rate_window.hpp"

using helix_adapter_cpp::RateWindow;

namespace
{

struct FakeClock
{
  double t = 0.0;
  RateWindow::Clock as_clock()
  {
    return [this]() {return this->t;};
  }
};

}  // namespace

TEST(RateWindowTest, EmptyWindowReturnsNan) {
  FakeClock clock;
  RateWindow rw(5.0, clock.as_clock());
  EXPECT_TRUE(std::isnan(rw.rate_or_nan()));
}

TEST(RateWindowTest, SingleSampleReturnsZero) {
  FakeClock clock;
  RateWindow rw(5.0, clock.as_clock());
  rw.record();
  EXPECT_EQ(rw.rate_or_nan(), 0.0);
}

TEST(RateWindowTest, TwoSamplesOneSecondApartIsOneHz) {
  FakeClock clock;
  RateWindow rw(5.0, clock.as_clock());
  rw.record();
  clock.t = 1.0;
  rw.record();
  EXPECT_EQ(rw.rate_or_nan(), 1.0);
}

TEST(RateWindowTest, TenSamplesInOneSecondIsTenHz) {
  FakeClock clock;
  RateWindow rw(5.0, clock.as_clock());
  for (int i = 0; i < 10; ++i) {
    clock.t = i * 0.1;
    rw.record();
  }
  EXPECT_NEAR(rw.rate_or_nan(), 10.0, 0.1);
}

TEST(RateWindowTest, StaleAfterWindowExpires) {
  FakeClock clock;
  RateWindow rw(5.0, clock.as_clock());
  rw.record();
  clock.t = 0.5;
  rw.record();
  clock.t = 10.0;
  EXPECT_TRUE(std::isnan(rw.rate_or_nan()));
}

TEST(RateWindowTest, EvictionDropsOldSamples) {
  FakeClock clock;
  RateWindow rw(2.0, clock.as_clock());
  clock.t = 0.0;
  rw.record();
  clock.t = 1.0;
  rw.record();
  clock.t = 5.0;
  rw.record();
  clock.t = 5.5;
  rw.record();
  EXPECT_NEAR(rw.rate_or_nan(), 2.0, 0.02);
}

TEST(RateWindowTest, IdenticalTimestampsReturnsZero) {
  FakeClock clock;
  RateWindow rw(5.0, clock.as_clock());
  rw.record();
  rw.record();
  rw.record();
  EXPECT_EQ(rw.rate_or_nan(), 0.0);
}

TEST(RateWindowTest, InvalidWindowThrows) {
  EXPECT_THROW(RateWindow(0.0), std::invalid_argument);
  EXPECT_THROW(RateWindow(-1.0), std::invalid_argument);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
