// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// See include/helix_sensing_cpp/pyfmt.hpp.

#include "helix_sensing_cpp/pyfmt.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

namespace helix_sensing_cpp
{

namespace
{

// glibc printf/strtod are correctly rounded (round-half-even on the exact
// binary value), as is CPython's dtoa, so "%.*f" agrees with round() and
// format() digit for digit.
std::string printf_fixed(double x, int ndigits)
{
  if (ndigits < 0 || ndigits > 17) {
    throw std::invalid_argument("ndigits must be in [0, 17]");
  }
  const int n = std::snprintf(nullptr, 0, "%.*f", ndigits, x);
  std::string out(static_cast<std::size_t>(n) + 1, '\0');
  std::snprintf(out.data(), out.size(), "%.*f", ndigits, x);
  out.resize(static_cast<std::size_t>(n));
  return out;
}

bool is_ascii_space(unsigned char c)
{
  // Py_ISSPACE, which float() strips with. Not str.isspace(): float() keeps
  // ASCII as is, so '\x1c'..'\x1f' are rejected rather than stripped.
  return c == ' ' || (c >= '\t' && c <= '\r');
}

// Length of a non-ASCII Unicode whitespace sequence (UTF-8) at s[i], or 0.
// Exactly the code points >= 0x80 for which str.isspace() is true; float()
// maps each of them to ' ' before parsing.
std::size_t unicode_space_len(std::string_view s, std::size_t i)
{
  static constexpr std::array<std::string_view, 19> kSpaces{
    "\xc2\x85", "\xc2\xa0", "\xe1\x9a\x80", "\xe2\x80\x80", "\xe2\x80\x81", "\xe2\x80\x82",
    "\xe2\x80\x83", "\xe2\x80\x84", "\xe2\x80\x85", "\xe2\x80\x86", "\xe2\x80\x87",
    "\xe2\x80\x88", "\xe2\x80\x89", "\xe2\x80\x8a", "\xe2\x80\xa8", "\xe2\x80\xa9",
    "\xe2\x80\xaf", "\xe2\x81\x9f", "\xe3\x80\x80"};
  for (const auto sp : kSpaces) {
    if (s.substr(i, sp.size()) == sp) {
      return sp.size();
    }
  }
  return 0;
}

bool is_digit(char c) {return c >= '0' && c <= '9';}

bool iequals(std::string_view a, std::string_view b)
{
  if (a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    char c = a[i];
    if (c >= 'A' && c <= 'Z') {
      c = static_cast<char>(c - 'A' + 'a');
    }
    if (c != b[i]) {
      return false;
    }
  }
  return true;
}

}  // namespace

std::string python_repr(double x)
{
  if (std::isnan(x)) {
    return "nan";
  }
  if (std::isinf(x)) {
    return x > 0 ? "inf" : "-inf";
  }
  if (x == 0.0) {
    return std::signbit(x) ? "-0.0" : "0.0";
  }
  // Shortest round-trip digits, as CPython's repr (dtoa mode 0).
  std::array<char, 64> buf{};
  const auto res = std::to_chars(
    buf.data(), buf.data() + buf.size(), x, std::chars_format::scientific);
  if (res.ec != std::errc()) {
    throw std::runtime_error("to_chars failed");
  }
  const std::string_view sci(buf.data(), static_cast<std::size_t>(res.ptr - buf.data()));
  const std::size_t e = sci.find('e');
  std::string_view mant = sci.substr(0, e);
  const int exp10 = std::atoi(std::string(sci.substr(e + 1)).c_str());
  const bool neg = mant.front() == '-';
  if (neg) {
    mant.remove_prefix(1);
  }
  std::string digits;
  for (char c : mant) {
    if (c != '.') {
      digits.push_back(c);
    }
  }
  const int decpt = exp10 + 1;  // digits are 0.d1d2... * 10^decpt
  std::string out = neg ? "-" : "";
  if (decpt <= -4 || decpt > 16) {
    out += digits.substr(0, 1);
    if (digits.size() > 1) {
      out += "." + digits.substr(1);
    }
    char ebuf[8];
    std::snprintf(ebuf, sizeof(ebuf), "e%+03d", decpt - 1);
    out += ebuf;
  } else if (decpt <= 0) {
    out += "0." + std::string(static_cast<std::size_t>(-decpt), '0') + digits;
  } else if (static_cast<std::size_t>(decpt) < digits.size()) {
    out += digits.substr(0, static_cast<std::size_t>(decpt)) + "." +
      digits.substr(static_cast<std::size_t>(decpt));
  } else {
    out += digits + std::string(static_cast<std::size_t>(decpt) - digits.size(), '0') + ".0";
  }
  return out;
}

std::string python_round_str(double x, int ndigits)
{
  if (!std::isfinite(x)) {
    return python_repr(x);
  }
  const double rounded = std::strtod(printf_fixed(x, ndigits).c_str(), nullptr);
  return python_repr(rounded);
}

std::string python_fixed(double x, int ndigits)
{
  if (std::isnan(x)) {
    return "nan";  // format() ignores the sign of a NaN
  }
  if (std::isinf(x)) {
    return x > 0 ? "inf" : "-inf";
  }
  return printf_fixed(x, ndigits);
}

std::optional<double> parse_python_float(std::string_view text)
{
  // 1. Unicode whitespace becomes ' ' (CPython transforms it to ASCII first).
  std::string s;
  s.reserve(text.size());
  for (std::size_t i = 0; i < text.size(); ) {
    const std::size_t n = unicode_space_len(text, i);
    if (n > 0) {
      s.push_back(' ');
      i += n;
    } else {
      s.push_back(text[i]);
      ++i;
    }
  }
  // 2. PEP 515: an underscore must sit between two digits; then drop it.
  if (s.find('_') != std::string::npos) {
    std::string kept;
    char prev = '\0';
    for (char c : s) {
      if (c == '_') {
        if (!is_digit(prev)) {
          return std::nullopt;
        }
      } else {
        if (prev == '_' && !is_digit(c)) {
          return std::nullopt;
        }
        kept.push_back(c);
      }
      prev = c;
    }
    if (prev == '_') {
      return std::nullopt;
    }
    s = kept;
  }
  // 3. Strip surrounding whitespace.
  std::size_t b = 0;
  std::size_t e = s.size();
  while (b < e && is_ascii_space(static_cast<unsigned char>(s[b]))) {
    ++b;
  }
  while (e > b && is_ascii_space(static_cast<unsigned char>(s[e - 1]))) {
    --e;
  }
  const std::string body = s.substr(b, e - b);
  if (body.empty()) {
    return std::nullopt;
  }
  // 4. Optional sign, then inf / infinity / nan, or a decimal literal.
  std::size_t i = 0;
  bool negative = false;
  if (body[i] == '+' || body[i] == '-') {
    negative = body[i] == '-';
    ++i;
  }
  const std::string_view rest(body.data() + i, body.size() - i);
  if (iequals(rest, "inf") || iequals(rest, "infinity")) {
    return negative ? -HUGE_VAL : HUGE_VAL;
  }
  if (iequals(rest, "nan")) {
    return negative ? -std::nan("") : std::nan("");
  }
  // Grammar: digits [ '.' digits ] [ (e|E) [+|-] digits ], >= 1 mantissa digit.
  std::size_t j = 0;
  std::size_t mantissa_digits = 0;
  while (j < rest.size() && is_digit(rest[j])) {
    ++j;
    ++mantissa_digits;
  }
  if (j < rest.size() && rest[j] == '.') {
    ++j;
    while (j < rest.size() && is_digit(rest[j])) {
      ++j;
      ++mantissa_digits;
    }
  }
  if (mantissa_digits == 0) {
    return std::nullopt;
  }
  if (j < rest.size() && (rest[j] == 'e' || rest[j] == 'E')) {
    ++j;
    if (j < rest.size() && (rest[j] == '+' || rest[j] == '-')) {
      ++j;
    }
    std::size_t exp_digits = 0;
    while (j < rest.size() && is_digit(rest[j])) {
      ++j;
      ++exp_digits;
    }
    if (exp_digits == 0) {
      return std::nullopt;
    }
  }
  if (j != rest.size()) {
    return std::nullopt;
  }
  // Validated: strtod sees only [sign] digits [. digits] [e [sign] digits].
  // Overflow gives +-HUGE_VAL and underflow a correctly rounded subnormal or
  // zero, exactly as float() does; errno is irrelevant here.
  return std::strtod(body.c_str(), nullptr);
}

}  // namespace helix_sensing_cpp
