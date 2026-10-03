// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// Absolute expectations for the Python-compatible formatting and parsing
// helpers. Every expected string below was produced by CPython 3.10;
// test_python_parity.py compares thousands more cases against the live
// interpreter.

#include <cmath>
#include <limits>
#include <string>

#include "gtest/gtest.h"

#include "helix_sensing_cpp/pyfmt.hpp"

using helix_sensing_cpp::parse_python_float;
using helix_sensing_cpp::python_fixed;
using helix_sensing_cpp::python_repr;
using helix_sensing_cpp::python_round_str;

TEST(PyFmt, RoundMatchesStrOfRound)
{
  EXPECT_EQ(python_round_str(100.0, 4), "100.0");
  EXPECT_EQ(python_round_str(2.675, 2), "2.67");   // binary value is below the tie
  EXPECT_EQ(python_round_str(1e16, 4), "1e+16");
  EXPECT_EQ(python_round_str(1.5e-05, 6), "1.5e-05");
  EXPECT_EQ(python_round_str(5e-05, 4), "0.0001");
  EXPECT_EQ(python_round_str(-0.0, 4), "-0.0");
  EXPECT_EQ(python_round_str(123456.78905, 4), "123456.7891");
  EXPECT_EQ(python_round_str(0.30000000000000004, 4), "0.3");
  EXPECT_EQ(python_round_str(1e300, 4), "1e+300");
  EXPECT_EQ(python_round_str(5e-324, 6), "0.0");
  EXPECT_EQ(python_round_str(std::numeric_limits<double>::infinity(), 2), "inf");
}

TEST(PyFmt, ReprSwitchesToExponentLikeCPython)
{
  EXPECT_EQ(python_repr(1e16), "1e+16");
  EXPECT_EQ(python_repr(1e15), "1000000000000000.0");
  EXPECT_EQ(python_repr(1e-05), "1e-05");
  EXPECT_EQ(python_repr(0.0001), "0.0001");
  EXPECT_EQ(python_repr(1.5e-310), "1.5e-310");
  EXPECT_EQ(python_repr(12345678.9), "12345678.9");
  EXPECT_EQ(python_repr(1.0 / 3.0), "0.3333333333333333");
  EXPECT_EQ(python_repr(std::nan("")), "nan");
}

TEST(PyFmt, FixedMatchesFormatSpec)
{
  EXPECT_EQ(python_fixed(2.675, 2), "2.67");
  EXPECT_EQ(python_fixed(-0.0, 2), "-0.00");
  EXPECT_EQ(python_fixed(std::numeric_limits<double>::infinity(), 2), "inf");
  EXPECT_EQ(python_fixed(-std::nan(""), 2), "nan");
}

TEST(PyFmt, ParseFollowsFloatOfStr)
{
  EXPECT_EQ(*parse_python_float(" 1_0.5 "), 10.5);
  EXPECT_EQ(*parse_python_float("\xc2\xa0" "7\xe2\x80\x83"), 7.0);  // NBSP, EM SPACE
  EXPECT_EQ(*parse_python_float("Infinity"), std::numeric_limits<double>::infinity());
  EXPECT_EQ(*parse_python_float("1e999"), std::numeric_limits<double>::infinity());
  EXPECT_EQ(*parse_python_float("1e-400"), 0.0);
  EXPECT_TRUE(std::isnan(*parse_python_float("-nan")));
  for (const char * bad : {"1__0", "_1", "1_", "0x10", "\x1c" "3", "", " ", "1e", ".", "nan(1)",
      "1,5", "OK", "+"})
  {
    EXPECT_FALSE(parse_python_float(bad).has_value()) << '"' << bad << '"';
  }
}
