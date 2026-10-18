// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// helix_log_replay <rules.yaml> <dedup_window_sec>: runs LogMatcher over
// tab-separated records "<wall t>\t<level>\t<name>\t<file>\t<msg>" and
// prints each fault as "<t>\t<node>\t<type>\t<severity>\t<detail>\t<ctx...>",
// for test_log_parser_parity.py.

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "helix_sensing_cpp/log_matcher.hpp"

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "usage: helix_log_replay <rules.yaml> <dedup_window_sec>\n";
    return 2;
  }
  helix_sensing_cpp::LogMatcher m(std::strtod(argv[2], nullptr));
  for (const auto & w : m.load(argv[1])) {
    std::cerr << w << "\n";
  }
  std::string line;
  while (std::getline(std::cin, line)) {
    std::vector<std::string> f;
    std::size_t start = 0;
    for (int i = 0; i < 4; ++i) {
      const auto tab = line.find('\t', start);
      if (tab == std::string::npos) {
        break;
      }
      f.push_back(line.substr(start, tab - start));
      start = tab + 1;
    }
    if (f.size() != 4) {
      continue;
    }
    f.push_back(line.substr(start));
    const double t = std::strtod(f[0].c_str(), nullptr);
    const auto level = static_cast<std::uint8_t>(std::atoi(f[1].c_str()));
    for (const auto & r : m.process(level, f[2], f[4], f[3], t)) {
      char ts[32];
      std::snprintf(ts, sizeof(ts), "%.6f", r.timestamp);
      std::cout << ts << '\t' << r.node_name << '\t' << r.fault_type << '\t' <<
        static_cast<int>(r.severity) << '\t' << r.detail;
      for (const auto & v : r.context_values) {
        std::cout << '\t' << v;
      }
      std::cout << '\n';
    }
  }
  return 0;
}
