// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// helix_sink_replay <mode>: drives SinkLogic + SinkFeedback the way the node
// does and prints one line per event, for test_python_parity.py.
//
// Input lines (whitespace separated):
//   cmd   <t> <vx> <vy> <wz>
//   tick  <t>
//   resp  <t> <request_id> <api_id> <code>
//   state <t> <vx> <vy> <yaw_speed> <mode>
// Request ids start at 1000 and increase by one per decision; outside
// dry_run each decision is recorded with on_sent, as the node does.

#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "helix_go2_sink_cpp/sink_core.hpp"

using helix_go2_sink_cpp::Mode;
using helix_go2_sink_cpp::SinkFeedback;
using helix_go2_sink_cpp::SinkLogic;

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: helix_sink_replay <dry_run|stop_only|armed>\n";
    return 2;
  }
  const Mode mode = helix_go2_sink_cpp::parse_mode(argv[1]);
  SinkLogic logic(mode);
  SinkFeedback fb;
  std::int64_t req_id = 1000;
  std::string line;
  while (std::getline(std::cin, line)) {
    // strtod, not operator>>: the stream extractor does not parse "nan".
    std::istringstream in(line);
    std::vector<std::string> tok;
    for (std::string w; in >> w; ) {
      tok.push_back(w);
    }
    if (tok.size() < 2) {
      continue;
    }
    auto num = [&tok](std::size_t i) {return std::strtod(tok.at(i).c_str(), nullptr);};
    const std::string & kind = tok[0];
    const double t = num(1);
    std::optional<helix_go2_sink_cpp::Request> r;
    if (kind == "cmd") {
      r = logic.on_command(num(2), num(3), num(4), t);
    } else if (kind == "tick") {
      r = logic.on_tick(t);
    } else if (kind == "resp") {
      fb.on_response(
        std::strtoll(tok.at(2).c_str(), nullptr, 10), std::atoi(tok.at(3).c_str()),
        std::atoi(tok.at(4).c_str()), t);
    } else if (kind == "state") {
      fb.on_state(
        num(2), num(3), num(4), static_cast<std::uint8_t>(std::atoi(tok.at(5).c_str())),
        t);
    } else {
      continue;
    }
    if (r) {
      ++req_id;
      if (mode != Mode::kDryRun) {
        fb.on_sent(req_id, r->api_id, r->reason, t);
      }
      std::printf("D %d %s %s", r->api_id, r->reason.c_str(), r->parameter.c_str());
    } else {
      std::printf("-");
    }
    std::printf(
      " | %d %.6f %.6f %d %.6f %d %" PRId64 " %d %d\n", fb.stop_acked ? 1 : 0,
      fb.stop_ack_sent_at, fb.stop_acked_at, fb.stopped ? 1 : 0, fb.stopped_since,
      fb.last_api_id, fb.last_ack_request_id, fb.last_ack_code, fb.stopped_now(t) ? 1 : 0);
  }
  return 0;
}
