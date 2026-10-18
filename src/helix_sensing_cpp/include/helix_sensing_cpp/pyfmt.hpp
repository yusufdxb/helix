// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// Python-compatible number formatting and parsing for the anomaly detector.
//
// The Python reference node (helix_core.anomaly_detector) fills FaultEvent
// context_values with str(round(x, n)), formats the detail with
// f"{z:.2f}", and turns DiagnosticArray strings into numbers with float().
// Downstream consumers (diagnosis rules, trace tooling, recorded evidence)
// see those exact strings, so the native node reproduces them exactly:
//
//   python_round_str(100.0, 4)  -> "100.0"   (not "100")
//   python_round_str(2.675, 2)  -> "2.67"    (correct rounding of the binary value)
//   python_repr(1e16)           -> "1e+16",  python_repr(1e-05) -> "1e-05"
//   parse_python_float(" 1_0.5 ") -> 10.5,   parse_python_float("0x10") -> none
//
// Known limit: float() also accepts non-ASCII Unicode decimal digits (for
// example Arabic-Indic digits); parse_python_float rejects them, so such a
// value is skipped instead of processed. Unicode whitespace is handled.
#ifndef HELIX_SENSING_CPP__PYFMT_HPP_
#define HELIX_SENSING_CPP__PYFMT_HPP_

#include <optional>
#include <string>
#include <string_view>

namespace helix_sensing_cpp
{

/// repr(x) / str(x) for a Python float: shortest round-trip digits, fixed
/// notation for decimal exponents -4 <= e < 16, otherwise d.ddde+XX.
std::string python_repr(double x);

/// str(round(x, ndigits)) for 0 <= ndigits <= 17.
std::string python_round_str(double x, int ndigits);

/// f"{x:.{ndigits}f}" for 0 <= ndigits <= 17 ("nan", "inf", "-inf").
std::string python_fixed(double x, int ndigits);

/// float(text) for a Python str: surrounding whitespace, PEP 515
/// underscores, signs, inf/infinity/nan in any case, decimal notation,
/// overflow to +-inf. std::nullopt where float() raises ValueError.
std::optional<double> parse_python_float(std::string_view text);

}  // namespace helix_sensing_cpp

#endif  // HELIX_SENSING_CPP__PYFMT_HPP_
