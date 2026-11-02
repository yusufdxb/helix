// LLM repair ranker: one constrained /api/chat call to a local Ollama server per decision.
// The model can only pick among the candidates the policy already proposed; the healer keeps the
// budget, execution, verification and rollback. Plain POSIX sockets, HTTP/1.0 (no chunked body).
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <sstream>
#include <stdexcept>

#include "helix_healer/agent.hpp"

namespace helix_healer {
namespace {

std::string json_str(const std::string& s) {
  std::string o = "\"";
  for (unsigned char ch : s) {
    switch (ch) {
      case '"': o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\n': o += "\\n"; break;
      case '\t': o += "\\t"; break;
      default:
        if (ch < 0x20 || ch > 0x7e) o += ' ';  // ASCII only; logs may carry colour codes
        else o += static_cast<char>(ch);
    }
  }
  return o + "\"";
}

const char* describe(Action a) {
  switch (a) {
    case Action::Restart: return "restart the process with its current config file and parameters";
    case Action::LifecycleReset: return "deactivate, clean up and reactivate the lifecycle node in place, no restart";
    case Action::ParamRestore:
      return "set the parameters that differ from last-known-good back to their last-known-good values on the "
             "live node, no restart";
    case Action::RestartLKG: return "restart the process from its last-known-good config file and parameters";
    default: return "";
  }
}

// Reads the string value of `key` from a JSON object text, undoing the common escapes.
bool json_field(const std::string& s, const std::string& key, std::string& out) {
  const std::string k = "\"" + key + "\"";
  auto p = s.find(k);
  if (p == std::string::npos) return false;
  p = s.find(':', p + k.size());
  if (p == std::string::npos) return false;
  p = s.find('"', p);
  if (p == std::string::npos) return false;
  out.clear();
  for (++p; p < s.size(); ++p) {
    char ch = s[p];
    if (ch == '"') return true;
    if (ch == '\\' && p + 1 < s.size()) {
      char n = s[++p];
      out += n == 'n' ? '\n' : n == 't' ? '\t' : n;
    } else {
      out += ch;
    }
  }
  return false;
}

double json_num(const std::string& s, const std::string& key) {
  const std::string k = "\"" + key + "\":";
  auto p = s.find(k);
  return p == std::string::npos ? -1 : std::atof(s.c_str() + p + k.size());
}

}  // namespace

LlmRanker::LlmRanker(const std::string& arg, double timeout_s) : timeout_s_(timeout_s) {
  auto at = arg.find('@');
  model_ = arg.substr(0, at);
  if (model_.empty()) throw std::runtime_error("llm ranker needs a model name (agent_arg)");
  if (at != std::string::npos) {
    std::string hp = arg.substr(at + 1);
    auto c = hp.rfind(':');
    host_ = hp.substr(0, c);
    if (c != std::string::npos) port_ = std::stoi(hp.substr(c + 1));
  }
}

std::string LlmRanker::prompt(const std::string& node, const AgentEvidence& e, const std::vector<Action>& c) const {
  std::ostringstream o;
  o << "Supervised ROS 2 node: " << node << "\n"
    << "Detected fault: " << to_string(e.fault) << "\n"
    << "Evidence:\n"
    << "- process alive: " << (e.alive ? "yes" : "no") << "\n"
    << "- consecutive exits shortly after start: " << e.quick_deaths << "\n"
    << "- live parameters differing from last-known-good: " << e.param_diff << "\n"
    << "- a config file on disk differs from its last-known-good copy: " << (e.config_changed ? "yes" : "no") << "\n"
    << "- the current process was started with that non-last-known-good config: " << (e.config_ran ? "yes" : "no")
    << "\n"
    << "- ERROR or FATAL lines in the recent log: " << e.log_errors << "\n"
    << "- lifecycle node: " << (e.lifecycle ? "yes" : "no") << "\n"
    << "- repair attempt in this incident: " << e.incident_action << " of " << Policy::kAgentK << "\n"
    << "Candidate actions (actions already tried in this incident are not listed):\n";
  for (auto a : c) o << "- " << to_string(a) << ": " << describe(a) << "\n";
  o << "Recent log:\n" << (e.log_tail.empty() ? "(empty)\n" : e.log_tail);
  return o.str();
}

std::string LlmRanker::request(const std::string& node, const AgentEvidence& e, const std::vector<Action>& c) const {
  std::string en;
  for (auto a : c) en += (en.empty() ? "" : ",") + json_str(to_string(a));
  const std::string sys =
      "You are the repair planner of a ROS 2 process supervisor. A supervised node has a detected fault. Read the "
      "evidence and the log, work out the most likely root cause, and choose the one candidate action most likely to "
      "restore the node on this attempt. Reply with JSON only.";
  std::ostringstream o;
  o << "{\"model\":" << json_str(model_) << ",\"stream\":false,\"think\":false,\"keep_alive\":\"30m\","
    << "\"options\":{\"temperature\":0,\"seed\":0,\"num_predict\":24},"
    << "\"format\":{\"type\":\"object\",\"properties\":{\"action\":{\"type\":\"string\",\"enum\":[" << en
    << "]}},\"required\":[\"action\"]},"
    << "\"messages\":[{\"role\":\"system\",\"content\":" << json_str(sys) << "},{\"role\":\"user\",\"content\":"
    << json_str(prompt(node, e, c)) << "}]}";
  return o.str();
}

Action LlmRanker::parse(const std::string& body, const std::vector<Action>& c) {
  std::string content, name;
  if (!json_field(body, "content", content) || !json_field(content, "action", name)) return Action::None;
  Action a = action_from(name);
  for (auto x : c)
    if (x == a) return a;
  return Action::None;
}

std::vector<Action> LlmRanker::rank(const std::string& node, const AgentEvidence& e, const std::vector<Action>& c) {
  detail_.clear();
  const auto t0 = std::chrono::steady_clock::now();
  const std::string body = request(node, e, c);
  addrinfo hints{}, *res = nullptr;
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  if (getaddrinfo(host_.c_str(), std::to_string(port_).c_str(), &hints, &res) != 0 || !res)
    throw std::runtime_error("llm_resolve");
  const int fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
  if (fd < 0) {
    freeaddrinfo(res);
    throw std::runtime_error("llm_socket");
  }
  timeval tv{static_cast<time_t>(timeout_s_), static_cast<suseconds_t>((timeout_s_ - static_cast<long>(timeout_s_)) * 1e6)};
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
  const int rc = connect(fd, res->ai_addr, res->ai_addrlen);
  freeaddrinfo(res);
  if (rc != 0) {
    close(fd);
    throw std::runtime_error("llm_connect");
  }
  std::string req = "POST /api/chat HTTP/1.0\r\nHost: " + host_ + "\r\nContent-Type: application/json\r\nContent-Length: " +
                    std::to_string(body.size()) + "\r\n\r\n" + body;
  for (size_t off = 0; off < req.size();) {
    ssize_t n = send(fd, req.data() + off, req.size() - off, MSG_NOSIGNAL);
    if (n <= 0) {
      close(fd);
      throw std::runtime_error("llm_send");
    }
    off += static_cast<size_t>(n);
  }
  std::string resp;
  char buf[4096];
  const auto deadline = t0 + std::chrono::duration<double>(timeout_s_);
  for (;;) {
    ssize_t n = recv(fd, buf, sizeof buf, 0);
    if (n > 0) resp.append(buf, static_cast<size_t>(n));
    if (n == 0) break;
    if (n < 0 || std::chrono::steady_clock::now() > deadline) {
      close(fd);
      throw std::runtime_error("llm_timeout");
    }
  }
  close(fd);
  const auto hdr = resp.find("\r\n\r\n");
  if (resp.compare(0, 12, "HTTP/1.1 200") != 0 && resp.compare(0, 12, "HTTP/1.0 200") != 0)
    throw std::runtime_error("llm_http_" + resp.substr(9, 3));
  const std::string json = hdr == std::string::npos ? resp : resp.substr(hdr + 4);
  char d[96];
  snprintf(d, sizeof d, " llm_total_ms=%.0f llm_eval_tok=%.0f", json_num(json, "total_duration") / 1e6,
           json_num(json, "eval_count"));
  detail_ = d;
  const Action first = parse(json, c);
  if (first == Action::None) throw std::runtime_error("llm_invalid");
  std::vector<Action> out{first};
  for (auto a : c)
    if (a != first) out.push_back(a);
  return out;
}

}  // namespace helix_healer
