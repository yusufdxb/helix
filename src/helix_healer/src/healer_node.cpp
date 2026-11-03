// helix_healer: supervises the processes named in a spec, observes them through the
// ROS graph and /proc, and executes the bounded repairs the Policy chooses.
//
// Threads: the executor thread owns topic state and the Policy. Repairs run in job
// threads that touch only the managed-process table (under mu_) and the Supervisor.
// The healer never publishes motion commands; its only outputs are /helix/mode,
// /helix/heal_events, /diagnostics and its own logs.
#include <ament_index_cpp/get_package_prefix.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <helix_msgs/msg/heal_event.hpp>
#include <helix_healer/build_info.hpp>
#include <helix_healer/envelope.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <signal.h>
#include <sys/stat.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "helix_healer/agent.hpp"
#include "helix_healer/policy.hpp"
#include "helix_healer/process.hpp"
#include "helix_healer/spec.hpp"

using namespace std::chrono_literals;
namespace hh = helix_healer;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

namespace {

std::string jesc(const std::string& s) {
  std::string o;
  o.reserve(s.size());
  for (char c : s) {
    if (c == '"' || c == '\\') o += '\\', o += c;
    else if (c == '\n') o += "\\n";
    else if (static_cast<unsigned char>(c) < 0x20) o += ' ';
    else o += c;
  }
  return o;
}

double slope(const std::deque<std::pair<double, double>>& h) {
  if (h.size() < 3) return 0;
  double mt = 0, mv = 0;
  for (auto& [t, v] : h) mt += t, mv += v;
  mt /= h.size();
  mv /= h.size();
  double num = 0, den = 0;
  for (auto& [t, v] : h) num += (t - mt) * (v - mv), den += (t - mt) * (t - mt);
  return den > 1e-9 ? num / den : 0;
}

// FNV-1a 64 of a file's bytes, cached by (mtime, size). A missing file hashes to a fixed marker.
uint64_t file_hash(const std::string& path) {
  struct Entry { int64_t mtime = 0, size = -1; uint64_t h = 0; };
  static std::mutex mu;
  static std::map<std::string, Entry> cache;
  struct stat st {};
  if (::stat(path.c_str(), &st) != 0) return 0x6d697373696e67ULL;
  const int64_t mt = static_cast<int64_t>(st.st_mtim.tv_sec) * 1000000000LL + st.st_mtim.tv_nsec;
  {
    std::lock_guard lk(mu);
    auto it = cache.find(path);
    if (it != cache.end() && it->second.mtime == mt && it->second.size == st.st_size) return it->second.h;
  }
  uint64_t h = helix_healer::kFnv64Offset;
  std::ifstream in(path, std::ios::binary);
  char buf[8192];
  while (in.read(buf, sizeof buf) || in.gcount() > 0) {
    h = helix_healer::fnv1a64(reinterpret_cast<const unsigned char*>(buf), static_cast<size_t>(in.gcount()), h);
  }
  std::lock_guard lk(mu);
  cache[path] = {mt, static_cast<int64_t>(st.st_size), h};
  return h;
}

// ERROR/FATAL lines in the last `bytes` of a log file.
int log_errors(const std::string& path, std::streamoff bytes = 8192) {
  std::ifstream in(path, std::ios::binary | std::ios::ate);
  if (!in) return 0;
  const std::streamoff size = in.tellg();
  in.seekg(std::max<std::streamoff>(0, size - bytes));
  std::string line;
  int n = 0;
  while (std::getline(in, line))
    if (line.find("[ERROR]") != std::string::npos || line.find("[FATAL]") != std::string::npos) ++n;
  return n;
}

// Last whole lines within the final `bytes` of a log file, for the LLM ranker's prompt.
std::string log_tail(const std::string& path, std::streamoff bytes = 1200) {
  std::ifstream in(path, std::ios::binary | std::ios::ate);
  if (!in) return {};
  const std::streamoff size = in.tellg();
  const std::streamoff from = std::max<std::streamoff>(0, size - bytes);
  in.seekg(from);
  std::string s((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  if (from > 0) {
    auto nl = s.find('\n');
    s = nl == std::string::npos ? std::string() : s.substr(nl + 1);
  }
  return s;
}

using ParamMap = std::map<std::string, rcl_interfaces::msg::ParameterValue>;
constexpr double kLkgHoldS = 5.0;  // health required past a snapshot before it becomes last-known-good

struct TopicState {
  std::deque<double> arrivals;
  double last = -1;
  double first = -1;
  double learned_hz = 0;
  rclcpp::GenericSubscription::SharedPtr sub;
};

// Adaptive envelope of one (producer, topic). Reset when the producer's process changes or an
// in-place repair (ParamRestore, LifecycleReset) succeeds: the deficit run up by the fault would
// otherwise keep a repaired node symptomatic for seconds after its rate recovered.
struct EnvSlot {
  hh::Envelope e;
  double start_t = -1e18;
  int repairs = 0;
};

struct Managed {
  hh::NodeSpec spec;
  std::vector<std::string> argv;  // resolved
  pid_t pid = -1;
  bool alive = false;
  bool started = false;
  bool retired = false;
  double start_t = 0;
  int repairs = 0;  // successful in-place repairs of this process, see EnvSlot
  double watch_t = 0;  // when the healer began observing this process (adoption time for attach=)
  double death_t = 0;
  int quick_deaths = 0;
  double dump_t = -1;  // first tick a core dump of the current process was seen
  std::set<std::string> upstream;  // sticky: an edge seen once stays
  bool qos_incompat = false;
  double cpu = 0;
  double rss = 0;
  double rss_base = 0;
  std::deque<std::pair<double, double>> rss_hist;
  double last_cpu_s = -1, last_cpu_t = 0;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get;
  bool learn_ok = false;  // the policy judged it healthy at the last tick: its timing may be learned
  // attach mode: adopted from an outside launcher; argv/env/cwd captured to re-execute it
  bool adopted = false;
  pid_t owner = -1;   // the launcher of the adopted process; its exit releases the node
  bool ours = false;  // the current process was started by the healer, not by the launcher
  std::vector<std::string> env;
  std::string cwd;
  std::vector<std::vector<std::string>> then;  // resolved helper commands
  // agent policy: last-known-good (LKG) parameters and config files
  std::shared_ptr<rclcpp::AsyncParametersClient> params;
  ParamMap cur_params;   // live values, from fetches and /parameter_events
  ParamMap lkg_params;   // promoted after kLkgHoldS of health past the fetch
  ParamMap cand_params;  // fetched while healthy, waiting for promotion
  double cand_t = -1, last_fetch = -1, healthy_since = -1;
  pid_t cand_pid = -1;
  uint64_t cand_gen = 0;
  bool cand_busy = false;
  double param_change_t = -1;   // last changed/deleted parameter event
  std::vector<size_t> cfg_idx;  // argv entries naming config files (.yaml .yml .urdf .xml)
  uint64_t spawn_hash = 0;      // hash of the config files the current process was started with
  uint64_t lkg_hash = 0;        // 0 = no LKG copies yet
  uint64_t reject_hash = 0;     // disk config rejected by a RestartLKG
  bool pinned = false;          // running on the LKG copies until the disk config changes again
  std::vector<std::string> lkg_argv;
  // gen ranker: versioned config snapshots, oldest first. lkg_hash/lkg_argv mirror the newest
  // generation that is not rejected, so spawn and pin work unchanged.
  struct Gen {
    uint64_t id = 0, hash = 0;
    std::vector<std::string> argv;
    std::string dir;
    bool trusted = false;   // a process running it stayed healthy (with its upstream) for the probation window
    bool rejected = false;  // stepped back from after recurring failures; never a rollback target again
    std::vector<uint64_t> files;  // FNV-1a 64 of each committed config file (matched against acks)
  };
  std::vector<Gen> gens;
  uint64_t next_gen = 0;
};

constexpr size_t kMaxGens = 3;

Managed::Gen* find_gen(Managed& m, uint64_t hash) {
  for (auto& g : m.gens)
    if (g.hash == hash) return &g;
  return nullptr;
}

// Newest non-rejected generation older than the one with id `below` (0 = no bound).
const Managed::Gen* prev_gen(const Managed& m, uint64_t below) {
  for (auto it = m.gens.rbegin(); it != m.gens.rend(); ++it)
    if (!it->rejected && (below == 0 || it->id < below)) return &*it;
  return nullptr;
}

void sync_lkg(Managed& m) {
  if (const auto* g = prev_gen(m, 0)) {
    m.lkg_hash = g->hash;
    m.lkg_argv = g->argv;
  }
}

uint64_t files_hash(const std::vector<std::string>& argv, const std::vector<size_t>& idx) {
  uint64_t h = helix_healer::kFnv64Offset;
  for (size_t i : idx) {
    const uint64_t f = file_hash(argv[i]);
    for (int b = 0; b < 8; ++b) h = (h ^ ((f >> (8 * b)) & 0xff)) * helix_healer::kFnv64Prime;
  }
  return h ? h : 1;
}

uint64_t config_hash(const Managed& m) { return files_hash(m.argv, m.cfg_idx); }

// LKG parameters that the live node currently holds with a different value.
std::vector<std::string> param_diff(const Managed& m) {
  std::vector<std::string> out;
  for (const auto& [k, v] : m.lkg_params) {
    auto it = m.cur_params.find(k);
    if (it != m.cur_params.end() && !(it->second == v)) out.push_back(k);
  }
  return out;
}

struct Result {
  std::string node;
  hh::Action action;
  bool ok;
  std::string detail;
};

}  // namespace

class Healer : public rclcpp::Node {
 public:
  Healer() : Node("helix_healer"), t0_(std::chrono::steady_clock::now()) {
    const auto spec_path = declare_parameter<std::string>("spec", "");
    const auto policy_name = declare_parameter<std::string>("policy", "helix");
    const auto event_log = declare_parameter<std::string>("event_log", "");
    const auto snap_log = declare_parameter<std::string>("snapshot_log", "");
    log_dir_ = declare_parameter<std::string>("log_dir", "");
    if (spec_path.empty()) throw std::runtime_error("parameter 'spec' is required");
    spec_ = hh::load_spec(spec_path);
    policy_ = std::make_unique<hh::Policy>(hh::policy_kind(policy_name), spec_);
    agent_ = policy_->kind() == hh::PolicyKind::Agent;
    const auto ranker_name = declare_parameter<std::string>("agent_ranker", "rule");
    const auto ranker_arg = declare_parameter<std::string>("agent_arg", "");
    const auto ranker_seed = declare_parameter<int64_t>("agent_seed", 0);
    // Health a config or parameter snapshot needs before it becomes last-known-good (the commit
    // window). The default is the v3 agent's value; the TXN study varies it.
    lkg_hold_s_ = declare_parameter<double>("agent_lkg_hold_s", kLkgHoldS);
    if (!(lkg_hold_s_ > 0)) throw std::runtime_error("agent_lkg_hold_s must be > 0");
    if (agent_) {
      ranker_ = hh::make_ranker(ranker_name, ranker_arg, static_cast<uint64_t>(ranker_seed));
      const bool two = ranker_name == "txn2" || ranker_name == "gen";
      policy_->set_txn(ranker_name == "txn" || two, two ? 2 : 1);
      policy_->set_gen(ranker_name == "gen");
    }
    // gen ranker: health a committed generation needs before it is trusted (no longer on probation).
    gen_probation_s_ = declare_parameter<double>("agent_gen_probation_s", 30.0);
    if (!(gen_probation_s_ > 0)) throw std::runtime_error("agent_gen_probation_s must be > 0");
    // gen ranker: directory of deploy acknowledgements (<node>.ack); empty = no acks are read.
    gen_ack_dir_ = declare_parameter<std::string>("agent_gen_ack_dir", "");
    if (!log_dir_.empty()) std::filesystem::create_directories(log_dir_);
    if (!event_log.empty()) {
      events_.open(event_log, std::ios::out | std::ios::trunc);
      const double wall = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
      events_ << "{\"t0_wall\":" << std::fixed << wall << ",\"policy\":\"" << policy_name << "\",\"pid\":" << getpid()
              << "}\n";
      events_.flush();
    }
    if (!snap_log.empty()) snaps_.open(snap_log, std::ios::out | std::ios::trunc);

    rclcpp::QoS latched(1);
    latched.reliable().transient_local();
    mode_pub_ = create_publisher<std_msgs::msg::String>("/helix/mode", latched);
    ev_pub_ = create_publisher<helix_msgs::msg::HealEvent>("/helix/heal_events", rclcpp::QoS(50).reliable());
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    publish_mode("OK");

    for (const auto& ns : spec_.nodes) {
      Managed m;
      m.spec = ns;
      if (!ns.argv.empty()) m.argv = resolve_argv(ns);
      for (const auto& h : ns.then) m.then.push_back(resolve_exe(h));
      for (const auto& o : ns.outputs) producers_[o.topic].push_back(ns.name);
      if (ns.lifecycle) {
        m.change = create_client<lifecycle_msgs::srv::ChangeState>("/" + ns.name + "/change_state");
        m.get = create_client<lifecycle_msgs::srv::GetState>("/" + ns.name + "/get_state");
      }
      for (const auto& o : ns.outputs) topics_[o.topic];
      if (agent_) {
        m.params = std::make_shared<rclcpp::AsyncParametersClient>(this, ns.ros_node.empty() ? "/" + ns.name : ns.ros_node);
        if (!ns.ros_node.empty()) ros_alias_[ns.ros_node] = ns.name;
        for (size_t i = 1; i < m.argv.size(); ++i) {
          const std::filesystem::path f(m.argv[i]);
          const auto ext = f.extension().string();
          if ((ext == ".yaml" || ext == ".yml" || ext == ".urdf" || ext == ".xml") && std::filesystem::is_regular_file(f))
            m.cfg_idx.push_back(i);
        }
      }
      nodes_.emplace(ns.name, std::move(m));
    }
    if (agent_) {
      param_sub_ = create_subscription<rcl_interfaces::msg::ParameterEvent>(
          "/parameter_events", rclcpp::ParameterEventsQoS(),
          [this](rcl_interfaces::msg::ParameterEvent::ConstSharedPtr ev) { on_param_event(*ev); });
      if (log_dir_.empty()) RCLCPP_WARN(get_logger(), "agent: log_dir is empty, so no config LKG copies (no restart_lkg)");
    }
    // Declared inputs give upstream edges before the graph shows them.
    for (auto& [name, m] : nodes_)
      for (const auto& in : m.spec.inputs)
        for (const auto& [other, om] : nodes_)
          for (const auto& o : om.spec.outputs)
            if (o.topic == in && other != name) m.upstream.insert(other);

    for (auto& [name, m] : nodes_)
      if (m.spec.autostart && m.spec.attach.empty())
        start_job(name, [this, name] { return spawn_and_bring_up(name, true); }, false);
    try_attach();

    const auto& p = spec_.p;
    tick_ = create_wall_timer(std::chrono::duration<double>(1.0 / p.tick_hz), [this] { tick(); });
    graph_ = create_wall_timer(500ms, [this] { refresh_graph(); });
    res_ = create_wall_timer(std::chrono::duration<double>(p.resource_period_s), [this] { sample_resources(); });
    diag_ = create_wall_timer(1s, [this] { publish_diagnostics(); });
    RCLCPP_INFO(get_logger(), "helix_healer: policy=%s%s%s, %zu managed nodes%s%s", policy_name.c_str(),
                agent_ ? " ranker=" : "", agent_ ? ranker_->name() : "", nodes_.size(),
                gen_ack_dir_.empty() ? "" : ", acks from ", gen_ack_dir_.c_str());
  }

  ~Healer() override {
    stopping_ = true;
    for (auto& j : jobs_) j.th.join();
    jobs_.clear();
    sup_.terminate_all(spec_.p.term_grace_s);
  }

 private:
  struct Job {
    std::string node;
    std::jthread th;
    std::shared_ptr<std::atomic<bool>> done;
  };

  double now() const { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count(); }

  static std::vector<std::string> resolve_exe(std::vector<std::string> a) {
    if (!a.empty() && a[0].size() > 1 && a[0][0] == '@') {
      const auto slash = a[0].find('/');
      if (slash == std::string::npos) throw std::runtime_error("exec '" + a[0] + "': expected @pkg/exe");
      const std::string pkg = a[0].substr(1, slash - 1);
      a[0] = ament_index_cpp::get_package_prefix(pkg) + "/lib/" + pkg + "/" + a[0].substr(slash + 1);
    }
    return a;
  }

  std::vector<std::string> resolve_argv(const hh::NodeSpec& ns) {
    std::vector<std::string> a = resolve_exe(ns.argv);
    // Name the node after the spec entry unless the command names it, or is a ros2 CLI verb other
    // than `run` (bag, launch, topic...), whose own options would swallow the remap.
    bool named = false;
    for (const auto& s : a)
      if (s.find("__node:=") != std::string::npos) named = true;
    const bool cli = !a.empty() && std::filesystem::path(a[0]).filename() == "ros2" && (a.size() < 2 || a[1] != "run");
    if (!named && !cli) a.insert(a.end(), {"--ros-args", "-r", "__node:=" + ns.name});
    return a;
  }

  // ---------- events ----------
  void publish_mode(const std::string& m) {
    std_msgs::msg::String s;
    s.data = m;
    mode_pub_->publish(s);
  }

  void emit(const hh::Event& e) {
    if (events_.is_open()) {
      std::lock_guard lk(log_mu_);
      events_ << std::fixed;
      events_.precision(4);
      events_ << "{\"t\":" << e.t << ",\"node\":\"" << jesc(e.node) << "\",\"kind\":\"" << e.kind << "\",\"fault\":\""
              << hh::to_string(e.fault) << "\",\"action\":\"" << hh::to_string(e.action) << "\",\"attempt\":" << e.attempt
              << ",\"detail\":\"" << jesc(e.detail) << "\"}\n";
      events_.flush();
    }
    helix_msgs::msg::HealEvent m;
    m.t = e.t;
    m.node = e.node;
    m.kind = e.kind;
    m.fault = hh::to_string(e.fault);
    m.action = hh::to_string(e.action);
    m.attempt = e.attempt;
    m.detail = e.detail;
    ev_pub_->publish(m);
    if (e.kind != "spawn")
      RCLCPP_INFO(get_logger(), "[%.2f] %s %s fault=%s action=%s %s", e.t, e.node.c_str(), e.kind.c_str(),
                  hh::to_string(e.fault), hh::to_string(e.action), e.detail.c_str());
  }

  void emit_simple(const std::string& node, const std::string& kind, const std::string& detail) {
    emit({now(), node, kind, hh::Fault::None, hh::Action::None, 0, detail});
  }

  // ---------- observation ----------
  void on_msg(const std::string& topic) {
    auto& ts = topics_[topic];
    const double t = now();
    if (auto pr = producers_.find(topic); pr != producers_.end()) {
      std::lock_guard lk(mu_);
      for (const auto& name : pr->second) {
        const auto& m = nodes_.at(name);
        if (!m.started || !m.alive) continue;
        auto& slot = env_slot(name, topic);
        if (slot.start_t != m.start_t || slot.repairs != m.repairs)
          slot.e.reset_process(), slot.start_t = m.start_t, slot.repairs = m.repairs;
        slot.e.arrival(t, m.learn_ok);
      }
    }
    ts.last = t;
    if (ts.first < 0) ts.first = t;
    ts.arrivals.push_back(t);
    while (!ts.arrivals.empty() && t - ts.arrivals.front() > 10.0) ts.arrivals.pop_front();
  }

  hh::TopicObs topic_obs(const hh::OutputSpec& o, const Managed& m, double t) {
    auto& ts = topics_[o.topic];
    hh::TopicObs ob;
    ob.topic = o.topic;
    const auto& p = spec_.p;
    if (o.hz > 0) {
      ob.expected_hz = o.hz;
    } else {
      if (ts.learned_hz <= 0 && ts.first >= 0 && t - ts.first >= p.learn_s) {
        int c = 0;
        for (double a : ts.arrivals)
          if (a >= ts.first && a <= ts.first + p.learn_s) ++c;
        ts.learned_hz = c / p.learn_s;
      }
      ob.expected_hz = ts.learned_hz;
    }
    ob.age_s = ts.last >= 0 ? t - ts.last : t - std::max(m.start_t, m.watch_t);
    const double w = ob.expected_hz > 0 ? std::max(p.rate_window_s, 3.0 / ob.expected_hz) : p.rate_window_s;
    int c = 0;
    // Since a respawn the window would still hold the outage, so a freshly restarted node is
    // judged on the arrivals since it started, once it has had three periods to show them.
    const double since = t - m.start_t;
    if (p.fast_repair > 0 && ob.expected_hz > 0 && since < w && since >= 3.0 / ob.expected_hz) {
      for (auto it = ts.arrivals.rbegin(); it != ts.arrivals.rend() && *it > m.start_t; ++it) ++c;
      double first = t;
      for (auto it = ts.arrivals.rbegin(); it != ts.arrivals.rend() && *it > m.start_t; ++it) first = *it;
      ob.rate_hz = c >= 2 ? (c - 1) / std::max(t - first, 1e-3) : 0.0;
    } else {
      for (auto it = ts.arrivals.rbegin(); it != ts.arrivals.rend() && t - *it <= w; ++it) ++c;
      ob.rate_hz = c / w;
    }
    ob.own_endpoint = own_.count({m.spec.name, o.topic}) > 0;
    if (p.adaptive > 0 && m.alive) {
      auto& slot = env_slot(m.spec.name, o.topic);
      if (slot.start_t == m.start_t && slot.repairs == m.repairs && slot.e.ready()) {
        ob.stale_limit_s = slot.e.stale_limit();
        ob.deficit = slot.e.deficit(t);
        ob.deficit_limit = slot.e.deficit_limit();
      }
    }
    return ob;
  }

  EnvSlot& env_slot(const std::string& node, const std::string& topic) {
    auto it = env_.find({node, topic});
    if (it != env_.end()) return it->second;
    const auto& p = spec_.p;
    hh::EnvelopeParams ep;
    ep.floor_s = p.adaptive_floor_s;
    ep.stale_mult = p.adaptive_stale_mult;
    ep.min_gaps = p.adaptive_min_gaps;
    ep.min_s = p.adaptive_min_s;
    ep.tau_s = p.adaptive_tau_s;
    ep.h_min = p.adaptive_cusum_h_min;
    ep.h_mult = p.adaptive_cusum_mult;
    ep.degraded_ratio = p.degraded_ratio;
    return env_.emplace(std::make_pair(node, topic), EnvSlot{hh::Envelope(ep)}).first->second;
  }

  // attach mode: adopt each attach= node's process once it exists.
  void try_attach() {
    for (auto& [name, m] : nodes_) {
      if (m.spec.attach.empty() || m.started || !m.spec.autostart) continue;
      if (adopt_running(name)) emit_simple(name, "attach", "pid " + std::to_string(m.pid) + " " + join(m.argv));
    }
  }

  // Caller must not hold mu_. Adopts a live process matching the node's attach pattern.
  bool adopt_running(const std::string& name) {
    const auto info = hh::find_process(nodes_.at(name).spec.attach);  // /proc scan outside the lock
    std::lock_guard lk(mu_);
    auto& m = nodes_.at(name);
    // Never adopt our own child: it is a replacement the healer started (possibly one it is stopping).
    if (info.pid <= 0 || info.pid == m.pid || info.ppid == ::getpid() || !sup_.adopt(info.pid)) return false;
    if (info.matches > 1)
      RCLCPP_WARN(get_logger(), "attach %s: %d processes match '%s', took the newest leaf pid %d", name.c_str(),
                  info.matches, m.spec.attach.c_str(), info.pid);
    m.pid = info.pid;
    m.argv = info.argv;
    m.env = info.env;
    m.cwd = info.cwd;
    m.adopted = true;
    m.owner = info.ppid;
    m.ours = false;
    m.alive = true;
    m.started = true;
    m.start_t = now() - info.age_s;
    m.watch_t = now();  // judge outputs only after the healer has had its own startup grace
    m.rss_base = 0;
    m.rss_hist.clear();
    m.last_cpu_s = -1;
    m.cpu = 0;
    return true;
  }

  static std::string join(const std::vector<std::string>& v) {
    std::string o;
    for (const auto& x : v) o += (o.empty() ? "" : " ") + x;
    return o;
  }

  hh::Snapshot snapshot(double t) {
    hh::Snapshot s;
    s.t = t;
    std::lock_guard lk(mu_);
    for (auto& [name, m] : nodes_) {
      if (!m.started) continue;
      if (m.alive && t - m.start_t >= spec_.p.loop_s) m.quick_deaths = 0;
      hh::NodeObs n;
      n.name = name;
      n.alive = m.alive;
      n.up_s = m.alive ? t - std::max(m.start_t, m.watch_t) : 0;
      n.down_s = m.alive ? 0 : t - m.death_t;
      n.cpu = m.cpu;
      n.rss_mb = m.rss;
      n.rss_slope = slope(m.rss_hist);
      n.rss_base = m.rss_base;
      n.quick_deaths = m.quick_deaths;
      n.qos_incompatible = m.qos_incompat;
      n.upstream.assign(m.upstream.begin(), m.upstream.end());
      for (const auto& o : m.spec.outputs) n.outputs.push_back(topic_obs(o, m, t));
      if (agent_) {
        n.ev.param_diff = static_cast<int>(param_diff(m).size());
        n.ev.has_lkg_config = m.lkg_hash != 0 && !m.lkg_argv.empty();
        n.ev.config_changed = n.ev.has_lkg_config && config_hash(m) != m.lkg_hash;
        n.ev.config_ran = n.ev.has_lkg_config && m.spawn_hash != m.lkg_hash;
        n.ev.config_pinned = m.pinned;
        if (policy_->gen()) {
          const auto* g = find_gen(m, m.spawn_hash);
          n.ev.run_hash = m.spawn_hash;
          n.ev.probation = !(g && g->trusted);
          n.ev.has_prev_gen = g && !g->rejected && (m.pinned || config_hash(m) == m.spawn_hash) && prev_gen(m, g->id);
        }
      }
      s.nodes.push_back(std::move(n));
    }
    return s;
  }

  void refresh_graph() {
    const auto tt = get_topic_names_and_types();
    std::set<std::pair<std::string, std::string>> own;
    std::map<std::string, std::set<std::string>> ups;
    std::set<std::string> incompat;
    for (const auto& [topic, types] : tt) {
      if (topic == "/rosout" || topic == "/parameter_events" || topic.rfind("/helix/", 0) == 0) continue;
      auto ts = topics_.find(topic);
      if (ts != topics_.end() && !ts->second.sub && !types.empty()) {
        const std::string tp = topic;
        ts->second.sub = create_generic_subscription(
            topic, types.front(), rclcpp::QoS(10).best_effort().durability_volatile(),
            [this, tp](std::shared_ptr<rclcpp::SerializedMessage>) { on_msg(tp); });
      }
      const auto pubs = get_publishers_info_by_topic(topic);
      const auto subs = get_subscriptions_info_by_topic(topic);
      for (const auto& pe : pubs) own.insert({pe.node_name(), topic});
      for (const auto& se : subs) {
        if (se.node_name() == get_name() || !nodes_.count(se.node_name())) continue;
        for (const auto& pe : pubs) {
          if (pe.node_name() == se.node_name()) continue;
          if (nodes_.count(pe.node_name())) ups[se.node_name()].insert(pe.node_name());
          if (rclcpp::qos_check_compatible(pe.qos_profile(), se.qos_profile()).compatibility ==
              rclcpp::QoSCompatibility::Error)
            incompat.insert(se.node_name());
        }
      }
    }
    own_.swap(own);
    try_attach();
    std::lock_guard lk(mu_);
    for (auto& [name, m] : nodes_) {
      for (const auto& u : ups[name]) m.upstream.insert(u);
      m.qos_incompat = incompat.count(name) > 0;
    }
  }

  void sample_resources() {
    const double t = now();
    std::lock_guard lk(mu_);
    for (auto& [name, m] : nodes_) {
      if (!m.alive || m.pid <= 0) continue;
      const auto ps = hh::read_proc(m.pid);
      if (!ps.ok) continue;
      if (m.last_cpu_s >= 0 && t > m.last_cpu_t) m.cpu = (ps.cpu_s - m.last_cpu_s) / (t - m.last_cpu_t);
      m.last_cpu_s = ps.cpu_s;
      m.last_cpu_t = t;
      m.rss = ps.rss_mb;
      if (m.rss_base <= 0 && t - m.start_t >= 1.0) m.rss_base = ps.rss_mb;
      m.rss_hist.emplace_back(t, ps.rss_mb);
      while (!m.rss_hist.empty() && t - m.rss_hist.front().first > 5.0) m.rss_hist.pop_front();
    }
  }

  // A crashed process that is still writing its core dump: seen at once, killed after core_wait_s.
  void reap_core_dumps(double t) {
    std::vector<std::pair<std::string, pid_t>> live;
    {
      std::lock_guard lk(mu_);
      for (auto& [name, m] : nodes_)
        if (m.alive && m.pid > 0) live.emplace_back(name, m.pid);
    }
    for (const auto& [name, pid] : live) {
      const bool dumping = hh::core_dumping(pid);  // outside the lock
      bool first = false, kill_now = false;
      {
        std::lock_guard lk(mu_);
        auto& m = nodes_.at(name);
        if (m.pid != pid || !m.alive) continue;
        if (!dumping) {
          m.dump_t = -1;
          continue;
        }
        first = m.dump_t < 0;
        if (first) m.dump_t = t;
        kill_now = t - m.dump_t >= spec_.p.core_wait_s;
      }
      if (first) emit_simple(name, "core_dump", "pid " + std::to_string(pid));
      // The exit is reaped on a later tick through the normal path and counts as a crash there.
      if (kill_now) ::kill(pid, SIGKILL);
    }
  }

  // ---------- main loop ----------
  void tick() {
    const double t = now();
    // Reap. An exit only counts if it is the node's current process; kills we issue are not faults.
    for (const auto& ex : sup_.poll()) {
      std::string who;
      {
        std::lock_guard lk(mu_);
        for (auto& [name, m] : nodes_) {
          if (m.pid != ex.pid || !m.alive) continue;
          m.alive = false;
          m.death_t = t;
          m.quick_deaths = (t - m.start_t < spec_.p.loop_s) ? m.quick_deaths + 1 : 0;
          m.cpu = 0;
          who = name;
        }
      }
      if (!who.empty()) emit_simple(who, "exit", ex.describe());
    }
    reap_core_dumps(t);
    // Finished jobs feed back into the policy on this thread.
    std::vector<Result> results;
    {
      std::lock_guard lk(res_mu_);
      results.swap(results_);
    }
    for (const auto& r : results) {
      if (!r.detail.empty() && !r.ok) emit_simple(r.node, "job_error", r.detail);
      policy_->on_result(r.node, r.action, r.ok, t);
    }
    std::erase_if(jobs_, [](Job& j) {
      if (!j.done->load()) return false;
      j.th.join();
      return true;
    });

    release_orphans();
    const auto snap = snapshot(t);
    auto out = policy_->step(snap);
    for (const auto& e : out.events) emit(e);
    if (agent_) for (const auto& e : out.events) verdict(e);
    for (const auto& d : out.decisions) dispatch(d, snap);
    {
      std::lock_guard lk(mu_);
      for (auto& [name, m] : nodes_)
        m.learn_ok = m.alive && t - std::max(m.start_t, m.watch_t) >= spec_.p.startup_grace_s && policy_->healthy(name);
    }
    if (agent_) maintain_lkg(t);
    if (snaps_.is_open() && (t - last_snap_ >= 0.25 || !out.decisions.empty())) {
      last_snap_ = t;
      write_snapshot(snap, out);
    }
  }

  void write_snapshot(const hh::Snapshot& s, const hh::StepOut& out) {
    std::ostringstream o;
    o << std::fixed;
    o.precision(3);
    o << "{\"t\":" << s.t << ",\"nodes\":[";
    bool first = true;
    for (const auto& n : s.nodes) {
      o << (first ? "" : ",") << "{\"name\":\"" << n.name << "\",\"alive\":" << n.alive << ",\"up\":" << n.up_s
        << ",\"down\":" << n.down_s << ",\"cpu\":" << n.cpu << ",\"rss\":" << n.rss_mb << ",\"slope\":" << n.rss_slope
        << ",\"base\":" << n.rss_base << ",\"qd\":" << n.quick_deaths << ",\"qos\":" << n.qos_incompatible
        << ",\"phase\":\"" << policy_->phase(n.name) << "\",\"ups\":[";
      for (std::size_t i = 0; i < n.upstream.size(); ++i) o << (i ? "," : "") << "\"" << n.upstream[i] << "\"";
      o << "],\"outs\":[";
      for (std::size_t i = 0; i < n.outputs.size(); ++i) {
        const auto& ob = n.outputs[i];
        o << (i ? "," : "") << "{\"topic\":\"" << ob.topic << "\",\"hz\":" << ob.expected_hz << ",\"age\":" << ob.age_s
          << ",\"rate\":" << ob.rate_hz << ",\"own\":" << ob.own_endpoint << ",\"lim\":" << ob.stale_limit_s
          << ",\"def\":" << ob.deficit << ",\"dlim\":" << ob.deficit_limit << "}";
      }
      o << "]}";
      first = false;
    }
    o << "],\"decisions\":[";
    for (std::size_t i = 0; i < out.decisions.size(); ++i)
      o << (i ? "," : "") << "{\"node\":\"" << out.decisions[i].node << "\",\"fault\":\""
        << hh::to_string(out.decisions[i].fault) << "\",\"action\":\"" << hh::to_string(out.decisions[i].action)
        << "\"}";
    o << "]}\n";
    snaps_ << o.str();
    snaps_.flush();
  }

  void publish_diagnostics() {
    diagnostic_msgs::msg::DiagnosticArray a;
    a.header.stamp = now_ros();
    std::lock_guard lk(mu_);
    for (const auto& [name, m] : nodes_) {
      if (!m.started) continue;
      diagnostic_msgs::msg::DiagnosticStatus st;
      st.name = "helix_healer: " + name;
      st.hardware_id = name;
      const std::string ph = policy_->phase(name);
      st.message = ph + (m.alive ? "" : " (down)");
      st.level = ph == "normal" && m.alive ? diagnostic_msgs::msg::DiagnosticStatus::OK
                 : ph == "retired"        ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                                          : diagnostic_msgs::msg::DiagnosticStatus::WARN;
      a.status.push_back(st);
    }
    diag_pub_->publish(a);
  }
  rclcpp::Time now_ros() { return get_clock()->now(); }

  // ---------- repairs ----------
  void start_job(const std::string& node, std::function<bool()> fn, bool report, hh::Action a = hh::Action::None) {
    launch(node, [fn = std::move(fn)](hh::Action&) { return fn(); }, report, a);
  }

  // fn may replace the reported action (the agent reports the ranked choice, not the default).
  void launch(const std::string& node, std::function<bool(hh::Action&)> fn, bool report, hh::Action a) {
    auto done = std::make_shared<std::atomic<bool>>(false);
    Job j{node, std::jthread([this, node, fn = std::move(fn), report, a, done] {
            bool ok = false;
            std::string err;
            hh::Action act = a;
            try {
              ok = fn(act);
            } catch (const std::exception& e) {
              err = e.what();
            }
            if (report) {
              std::lock_guard lk(res_mu_);
              results_.push_back({node, act, ok, err});
            }
            done->store(true);
          }),
          done};
    jobs_.push_back(std::move(j));
  }

  void dispatch(const hh::Decision& d, const hh::Snapshot& snap) {
    const std::string node = d.node;
    if (!d.candidates.empty()) return dispatch_agent(d, snap);
    switch (d.action) {
      case hh::Action::Restart:
        start_job(node, [this, node, delay = d.delay_s, f = d.fault] { return restart(node, delay, f); }, true,
                  d.action);
        break;
      case hh::Action::LifecycleReset:
        start_job(node, [this, node] { return lifecycle_reset(node) && repaired_in_place(node); }, true, d.action);
        break;
      case hh::Action::Fallback:
        start_job(node, [this, node] { return fallback(node); }, true, d.action);
        break;
      case hh::Action::Report: {
        std::lock_guard lk(mu_);
        nodes_.at(node).retired = true;
        std::lock_guard rl(res_mu_);
        results_.push_back({node, d.action, true, ""});
        break;
      }
      case hh::Action::ParamRestore:
      case hh::Action::RestartLKG:  // only reachable through candidates
      case hh::Action::None:
        break;
    }
  }

  // ---------- agent: incident dispatch, verdicts, last-known-good ----------
  struct Pending {
    hh::AgentEvidence e;
    hh::Action action = hh::Action::None;
    std::vector<rclcpp::Parameter> prev;  // values a ParamRestore overwrote, for rollback
    pid_t pid = -1;
  };

  static void invalidate(Managed& m) {
    m.cand_t = -1;
    m.cand_params.clear();
    m.cand_busy = false;
    ++m.cand_gen;
  }

  void dispatch_agent(const hh::Decision& d, const hh::Snapshot& snap) {
    const std::string node = d.node;
    hh::AgentEvidence e;
    e.fault = d.fault;
    e.incident_action = d.incident_action;
    e.lifecycle = nodes_.at(node).spec.lifecycle;
    for (const auto& n : snap.nodes) {
      if (n.name != node) continue;
      e.alive = n.alive;
      e.quick_deaths = n.quick_deaths;
      e.param_diff = n.ev.param_diff;
      e.config_changed = n.ev.config_changed;
      e.config_ran = n.ev.config_ran;
    }
    if (!log_dir_.empty()) {
      e.log_errors = log_errors(log_dir_ + "/" + node + ".log");
      if (std::string_view(ranker_->name()) == "llm") e.log_tail = log_tail(log_dir_ + "/" + node + ".log");
    }
    std::vector<rclcpp::Parameter> restore, prev;
    std::shared_ptr<rclcpp::AsyncParametersClient> pc;
    pid_t pid;
    {
      std::lock_guard lk(mu_);
      auto& m = nodes_.at(node);
      pc = m.params;
      pid = m.pid;
      for (const auto& k : param_diff(m)) {
        restore.emplace_back(k, rclcpp::ParameterValue(m.lkg_params.at(k)));
        prev.emplace_back(k, rclcpp::ParameterValue(m.cur_params.at(k)));
      }
    }
    launch(node, [this, d, node, e, restore, prev, pc, pid](hh::Action& chosen) -> bool {
      const auto t0 = std::chrono::steady_clock::now();
      std::vector<hh::Action> order;
      std::string note;
      try {
        std::lock_guard lk(ranker_mu_);
        order = ranker_->rank(node, e, d.candidates);
        note = ranker_->detail();
      } catch (const std::exception& ex) {
        note = std::string(" ranker_error=") + ex.what();
      }
      // A ranker may only reorder: anything else falls back to the rule order.
      auto sorted = [](std::vector<hh::Action> v) { std::sort(v.begin(), v.end()); return v; };
      if (order.empty() || sorted(order) != sorted(d.candidates)) {
        if (!order.empty()) note += " ranker_invalid";
        order = d.candidates;
      }
      chosen = order.front();
      const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      auto names = [](const std::vector<hh::Action>& v) {
        std::string s;
        for (auto a : v) s += (s.empty() ? "" : ",") + std::string(hh::to_string(a));
        return s;
      };
      char buf[32];
      snprintf(buf, sizeof buf, "%.3f", ms);
      std::string x;  // evidence features, so offline training can pair them with the verdict
      for (float f : hh::features(e)) {
        char fb[16];
        snprintf(fb, sizeof fb, "%.4g", f);
        x += (x.empty() ? "" : ",") + std::string(fb);
      }
      emit({now(), node, "act", d.fault, chosen, d.incident_action,
            std::string("ranker=") + ranker_->name() + " ms=" + buf + " cands=" + names(d.candidates) +
                " order=" + names(order) + " sig=" + hh::signature(e) + " x=" + x + note});
      {
        std::lock_guard lk(pend_mu_);
        pending_[node] = Pending{e, chosen, chosen == hh::Action::ParamRestore ? prev : std::vector<rclcpp::Parameter>{},
                                 pid};
      }
      switch (chosen) {
        case hh::Action::Restart:
          return restart(node, d.delay_s, d.fault);
        case hh::Action::LifecycleReset:
          return lifecycle_reset(node) && repaired_in_place(node);
        case hh::Action::ParamRestore: {
          if (!pc || restore.empty() || !pc->service_is_ready()) return false;
          auto f = pc->set_parameters(restore);
          if (f.wait_for(2s) != std::future_status::ready) return false;
          for (const auto& r : f.get())
            if (!r.successful) return false;
          emit_simple(node, "param_restore", names_of(restore));
          return repaired_in_place(node);
        }
        case hh::Action::RestartLKG: {
          std::string what = "restart on last-known-good config copies";
          {
            std::lock_guard lk(mu_);
            auto& m = nodes_.at(node);
            if (policy_->gen()) {
              // Step back a generation: the running committed generation keeps failing on probation.
              auto* g = find_gen(m, m.spawn_hash);
              const auto* p = g && !g->rejected ? prev_gen(m, g->id) : nullptr;
              if (p && (m.pinned || config_hash(m) == m.spawn_hash)) {
                g->rejected = true;
                what = "step back g" + std::to_string(g->id) + " -> g" + std::to_string(p->id);
                sync_lkg(m);
              }
            }
            if (m.lkg_hash == 0 || m.lkg_argv.empty()) return false;
            m.pinned = true;
            m.reject_hash = config_hash(m);
          }
          emit_simple(node, "pin", what);
          return restart(node, d.delay_s, d.fault);
        }
        default:
          return false;
      }
    }, true, d.action);
  }

  bool repaired_in_place(const std::string& node) {
    std::lock_guard lk(mu_);
    ++nodes_.at(node).repairs;
    return true;
  }

  static std::string names_of(const std::vector<rclcpp::Parameter>& ps) {
    std::string s;
    for (const auto& p : ps) s += (s.empty() ? "" : ",") + p.get_name();
    return s;
  }

  // Outcome of an agent action: feed the ranker (repair history) and undo a failed ParamRestore.
  void verdict(const hh::Event& ev) {
    if (ev.kind != "verified" && ev.kind != "verify_failed" && ev.kind != "action_failed") return;
    Pending p;
    {
      std::lock_guard lk(pend_mu_);
      auto it = pending_.find(ev.node);
      if (it == pending_.end()) return;
      p = std::move(it->second);
      pending_.erase(it);
    }
    const bool ok = ev.kind == "verified";
    {
      std::lock_guard lk(ranker_mu_);
      ranker_->record(ev.node, p.e, p.action, ok);
    }
    if (ok || p.action != hh::Action::ParamRestore || p.prev.empty()) return;
    std::shared_ptr<rclcpp::AsyncParametersClient> pc;
    {
      std::lock_guard lk(mu_);
      auto& m = nodes_.at(ev.node);
      if (m.pid != p.pid || !m.alive) return;  // a different process now: nothing to undo
      pc = m.params;
    }
    if (!pc || !pc->service_is_ready()) return;
    pc->set_parameters(p.prev);  // fire and forget; the outcome shows in the next parameter event
    emit_simple(ev.node, "rollback", names_of(p.prev));
  }

  void on_param_event(const rcl_interfaces::msg::ParameterEvent& ev) {
    std::string name = ev.node;
    if (auto a = ros_alias_.find(name); a != ros_alias_.end()) name = a->second;
    else if (!name.empty() && name[0] == '/') name.erase(0, 1);
    std::lock_guard lk(mu_);
    auto it = nodes_.find(name);
    if (it == nodes_.end()) return;
    auto& m = it->second;
    for (const auto& p : ev.new_parameters) m.cur_params[p.name] = p.value;
    for (const auto& p : ev.changed_parameters) m.cur_params[p.name] = p.value;
    for (const auto& p : ev.deleted_parameters) m.cur_params.erase(p.name);
    if (!ev.changed_parameters.empty() || !ev.deleted_parameters.empty()) {
      m.param_change_t = now();
      invalidate(m);
    }
  }

  // Parameters: fetch a candidate while healthy, promote it after agent_lkg_hold_s more of health with no
  // change in between. Config files: copy the files the running process was started with once it
  // has been healthy for agent_lkg_hold_s. The user's files are only ever read.
  void maintain_lkg(double t) {
    std::vector<std::string> fetch;
    std::vector<std::pair<std::string, std::string>> promoted;
    struct Copy {
      std::string node;
      std::vector<std::string> argv;
      std::vector<size_t> idx;
      uint64_t hash;
      uint64_t gen = 0;  // gen ranker: the generation id being committed
    };
    std::vector<Copy> copies;
    std::vector<std::pair<std::string, std::string>> trusted, acks_seen;
    const bool gen = agent_ && policy_->gen();
    {
      std::lock_guard lk(mu_);
      for (auto& [name, m] : nodes_) {
        if (gen && !gen_ack_dir_.empty() && !m.gens.empty()) {
          // Deploy acknowledgement: evidence from outside the healer that this exact config is intended
          // and accepted. It trusts the generation whether or not the node is healthy right now.
          bool fresh = false;
          const auto& ack = acked(name, &fresh);
          if (fresh) {
            // Record what the ack holds against what each generation would need, so an ack that
            // matches nothing is visible instead of silently ignored.
            std::ostringstream d;
            d << ack.size() << " hash(es)";
            for (const auto& g : m.gens) {
              d << "; g" << g.id << (g.trusted ? " trusted" : "") << (g.rejected ? " rejected" : "") << " needs";
              for (uint64_t f : g.files) d << ' ' << std::hex << std::setw(16) << std::setfill('0') << f << std::dec;
            }
            acks_seen.push_back({name, d.str()});
          }
          for (auto& g : m.gens)
            if (!g.trusted && !g.rejected && helix_healer::ack_covers(ack, g.files)) {
              g.trusted = true;
              trusted.push_back({name, "g" + std::to_string(g.id) + " ack"});
            }
        }
        if (!m.learn_ok || !m.alive || m.retired) {
          m.healthy_since = -1;
          if (m.cand_t >= 0 || m.cand_busy) invalidate(m);
          continue;
        }
        if (m.healthy_since < 0) m.healthy_since = t;
        if (m.params) {
          if (m.cand_busy && t - m.last_fetch > lkg_hold_s_) invalidate(m);  // lost reply
          if (m.cand_t >= 0 && t - m.cand_t >= lkg_hold_s_ && m.cand_pid == m.pid) {
            if (m.cand_params != m.lkg_params) {
              m.lkg_params = m.cand_params;
              promoted.push_back({name, "params=" + std::to_string(m.lkg_params.size())});
            }
            m.cand_t = -1;
            m.cand_params.clear();
          } else if (!m.cand_busy && m.cand_t < 0 && t - m.last_fetch >= lkg_hold_s_ &&
                     (m.param_change_t < 0 || t - m.param_change_t >= lkg_hold_s_) && m.params->service_is_ready()) {
            m.cand_busy = true;
            m.last_fetch = t;
            fetch.push_back(name);
          }
        }
        if (gen) {
          // Health evidence for a generation: the node and every managed upstream node are healthy now.
          bool up_ok = true;
          for (const auto& u : m.upstream) {
            auto it = nodes_.find(u);
            if (it != nodes_.end() && !(it->second.learn_ok && it->second.alive)) up_ok = false;
          }
          if (!up_ok) {
            m.healthy_since = t;  // the window restarts once the upstream is healthy again
            continue;
          }
          auto* g = find_gen(m, m.spawn_hash);
          if (g && !g->trusted && !g->rejected && t - m.healthy_since >= gen_probation_s_) {
            g->trusted = true;
            trusted.push_back({name, "g" + std::to_string(g->id)});
          }
          if (!g && !m.cfg_idx.empty() && !m.pinned && !log_dir_.empty() && m.spawn_hash != 0 &&
              t - m.healthy_since >= lkg_hold_s_ && config_hash(m) == m.spawn_hash)
            copies.push_back({name, m.argv, m.cfg_idx, m.spawn_hash, ++m.next_gen});
          continue;
        }
        if (!m.cfg_idx.empty() && !m.pinned && !log_dir_.empty() && m.spawn_hash != 0 && m.spawn_hash != m.lkg_hash &&
            t - m.healthy_since >= lkg_hold_s_ && config_hash(m) == m.spawn_hash)
          copies.push_back({name, m.argv, m.cfg_idx, m.spawn_hash});
      }
    }
    for (const auto& [n, d] : acks_seen) emit_simple(n, "lkg_ack", d);
    for (const auto& [n, d] : trusted) emit_simple(n, "lkg_trust", d);
    for (const auto& c : copies)
      if (c.gen) commit_gen(c.node, c.argv, c.idx, c.hash, c.gen);
    if (gen) copies.clear();
    for (const auto& [n, d] : promoted) emit_simple(n, "lkg", d);
    for (const auto& n : fetch) fetch_params(n);
    for (const auto& c : copies) {
      std::vector<std::string> argv = c.argv;
      try {
        const auto dir = std::filesystem::path(log_dir_) / "lkg" / c.node;
        std::filesystem::create_directories(dir);
        for (size_t i : c.idx) {
          const auto dst = dir / (std::to_string(i) + "_" + std::filesystem::path(c.argv[i]).filename().string());
          const auto tmp = dst.string() + ".tmp";
          std::filesystem::copy_file(c.argv[i], tmp, std::filesystem::copy_options::overwrite_existing);
          std::filesystem::rename(tmp, dst);
          argv[i] = dst.string();
        }
      } catch (const std::exception& ex) {
        emit_simple(c.node, "lkg_error", ex.what());
        continue;
      }
      bool ok = false;
      {
        std::lock_guard lk(mu_);
        auto& m = nodes_.at(c.node);
        // The source must not have changed while it was copied.
        if (m.spawn_hash == c.hash && config_hash(m) == c.hash) {
          m.lkg_argv = std::move(argv);
          m.lkg_hash = c.hash;
          ok = true;
        }
      }
      if (ok) emit_simple(c.node, "lkg_config", std::to_string(c.idx.size()) + " file(s)");
    }
  }

  // gen ranker: copy the config files into a fresh generation directory, check that the copies hash to
  // the config the healthy process was started with, then publish the directory with one rename and
  // add it to the ring. Any mismatch or error discards the attempt; the ring is left unchanged.
  void commit_gen(const std::string& node, const std::vector<std::string>& src, const std::vector<size_t>& idx,
                  uint64_t hash, uint64_t id) {
    namespace fs = std::filesystem;
    const auto base = fs::path(log_dir_) / "lkg" / node;
    const auto dir = base / ("g" + std::to_string(id));
    const auto tmp = base / ("g" + std::to_string(id) + ".tmp");
    std::vector<std::string> argv = src;
    std::vector<uint64_t> fh;
    std::error_code ec;
    try {
      fs::remove_all(tmp, ec);
      fs::create_directories(tmp);
      std::vector<std::string> staged = src;
      for (size_t i : idx) {
        const auto name = std::to_string(i) + "_" + fs::path(src[i]).filename().string();
        fs::copy_file(src[i], tmp / name, fs::copy_options::overwrite_existing);
        staged[i] = (tmp / name).string();
        argv[i] = (dir / name).string();
      }
      if (files_hash(staged, idx) != hash) {
        fs::remove_all(tmp, ec);
        emit_simple(node, "lkg_error", "g" + std::to_string(id) + " copy does not match the running config");
        return;
      }
      fs::remove_all(dir, ec);
      fs::rename(tmp, dir);
      for (size_t i : idx) fh.push_back(file_hash(argv[i]));
    } catch (const std::exception& ex) {
      fs::remove_all(tmp, ec);
      emit_simple(node, "lkg_error", ex.what());
      return;
    }
    std::string evicted;
    bool ok = false;
    {
      std::lock_guard lk(mu_);
      auto& m = nodes_.at(node);
      // The process must still run this config and the source must not have changed while copying.
      if (m.spawn_hash == hash && config_hash(m) == hash && !find_gen(m, hash)) {
        m.gens.push_back({id, hash, argv, dir.string(), false, false, fh});
        while (m.gens.size() > kMaxGens) {
          auto victim = m.gens.begin();
          for (auto it = m.gens.begin(); it != m.gens.end(); ++it)
            if (it->rejected) { victim = it; break; }
          evicted = victim->dir;
          m.gens.erase(victim);
        }
        sync_lkg(m);
        ok = true;
      }
    }
    if (!ok) {
      fs::remove_all(dir, ec);
      return;
    }
    if (!evicted.empty()) fs::remove_all(evicted, ec);
    emit_simple(node, "lkg_config", "g" + std::to_string(id) + " " + std::to_string(idx.size()) + " file(s)");
  }

  void fetch_params(const std::string& name) {
    std::shared_ptr<rclcpp::AsyncParametersClient> c;
    uint64_t gen;
    pid_t pid;
    {
      std::lock_guard lk(mu_);
      auto& m = nodes_.at(name);
      c = m.params;
      gen = m.cand_gen;
      pid = m.pid;
    }
    auto fail = [this, name, gen] {
      std::lock_guard lk(mu_);
      auto& m = nodes_.at(name);
      if (m.cand_gen == gen) m.cand_busy = false;
    };
    try {
      c->list_parameters({}, 0, [this, c, name, gen, pid, fail](std::shared_future<rcl_interfaces::msg::ListParametersResult> f) {
        try {
          const auto names = f.get().names;
          c->get_parameters(names, [this, name, gen, pid, fail](std::shared_future<std::vector<rclcpp::Parameter>> g) {
            try {
              const auto ps = g.get();
              std::lock_guard lk(mu_);
              auto& m = nodes_.at(name);
              if (m.cand_gen != gen) return;  // a parameter changed or the process was replaced meanwhile
              m.cand_busy = false;
              if (m.pid != pid || !m.learn_ok) return;
              ParamMap pm;
              for (const auto& p : ps) pm[p.get_name()] = p.get_value_message();
              for (const auto& [k, v] : pm) m.cur_params[k] = v;
              m.cand_params = std::move(pm);
              m.cand_t = now();
              m.cand_pid = pid;
            } catch (const std::exception&) {
              fail();
            }
          });
        } catch (const std::exception&) {
          fail();
        }
      });
    } catch (const std::exception&) {
      fail();
    }
  }

  bool sleep_for(double s) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(s);
    while (std::chrono::steady_clock::now() < end) {
      if (stopping_) return false;
      std::this_thread::sleep_for(10ms);
    }
    return !stopping_;
  }

  void kill(const std::string& node, double grace_s = -1) {
    pid_t old;
    {
      std::lock_guard lk(mu_);
      auto& m = nodes_.at(node);
      old = m.pid;
      if (m.alive) m.death_t = now();
      m.alive = false;
      m.pid = -1;
      m.cpu = 0;
    }
    if (old > 0) sup_.terminate(old, grace_s >= 0 ? grace_s : spec_.p.term_grace_s);
  }

  bool spawn_and_bring_up(const std::string& node, bool initial) {
    if (stopping_) return false;
    // An attached node may already have been restarted by its own launcher: take that one
    // instead of starting a second copy.
    if (!nodes_.at(node).spec.attach.empty() && adopt_running(node)) {
      emit_simple(node, "attach", "pid " + std::to_string(nodes_.at(node).pid) + " (restarted elsewhere)");
      // The launcher restarted the process but not necessarily its helpers (a controller spawner
      // is a one-shot action in a launch file), so run them here too. A helper that the launcher
      // does re-run fails harmlessly on the second call (controller already loaded).
      std::vector<std::vector<std::string>> helpers;
      {
        std::lock_guard lk(mu_);
        helpers = nodes_.at(node).then;
      }
      run_then(node, helpers);
      return true;
    }
    pid_t pid;
    std::vector<std::vector<std::string>> helpers;
    {
      std::lock_guard lk(mu_);
      auto& m = nodes_.at(node);
      const std::string log = log_dir_.empty() ? "" : log_dir_ + "/" + node + ".log";
      hh::SpawnOpts o;
      if (m.adopted) {
        // Started elsewhere: run it the way it was run, and let it outlive the healer like the original would.
        o.set_env = !m.env.empty();
        o.env = m.env;
        o.cwd = m.cwd;
        o.detach = true;
      }
      if (m.argv.empty()) return false;  // attach node never seen running
      const std::vector<std::string>* argv = &m.argv;
      if (agent_) {
        if (!m.cfg_idx.empty()) {
          const uint64_t disk = config_hash(m);
          if (m.pinned && disk != m.reject_hash) m.pinned = false;  // a newer deploy replaces the rejected one
          if (m.pinned && !m.lkg_argv.empty()) argv = &m.lkg_argv;
          m.spawn_hash = m.pinned ? m.lkg_hash : disk;
        }
        m.cur_params.clear();
        invalidate(m);
        m.healthy_since = -1;
      }
      pid = sup_.spawn(*argv, log, o);
      helpers = m.then;
      m.pid = pid;
      m.alive = pid > 0;
      m.ours = true;
      m.started = true;
      m.start_t = now();
      m.rss_base = 0;
      m.rss_hist.clear();
      m.last_cpu_s = -1;
      m.cpu = 0;
    }
    emit_simple(node, "spawn", "pid " + std::to_string(pid));
    if (pid <= 0) return false;
    const auto& m = nodes_.at(node);
    // A freshly spawned process needs time to create its services.
    const bool ok =
        !m.spec.lifecycle ||
        bring_up(node, initial ? spec_.p.startup_grace_s : std::max(2.0, spec_.p.lifecycle_timeout_s));
    run_then(node, helpers);
    return ok;
  }

  void run_then(const std::string& node, const std::vector<std::vector<std::string>>& helpers) {
    for (const auto& h : helpers) {
      const std::string log = log_dir_.empty() ? "" : log_dir_ + "/" + node + ".then.log";
      emit_simple(node, "then", "pid " + std::to_string(sup_.spawn(h, log)) + " " + join(h));
    }
  }

  // attach mode: when the launcher of an adopted node exits (a launch shutdown), stop
  // supervising the node, so the healer does not keep a stack alive that its owner stopped.
  // A replacement the healer started is stopped too, because the launcher could not stop it.
  // The node is adopted again if the launcher starts it again.
  void release_orphans() {
    std::vector<std::pair<std::string, pid_t>> gone;  // node, our pid to stop (or -1)
    {
      std::lock_guard lk(mu_);
      for (auto& [name, m] : nodes_) {
        if (!m.adopted || !m.started || m.owner <= 1) continue;
        if (::kill(m.owner, 0) == 0 || errno != ESRCH) continue;
        if (std::any_of(jobs_.begin(), jobs_.end(), [&](const Job& j) { return j.node == name && !j.done->load(); }))
          continue;  // a repair is in flight: release after it reports
        const pid_t stop = (m.ours && m.alive) ? m.pid : -1;
        if (m.ours || !m.alive) m.pid = -1;  // a live launcher-owned orphan keeps its pid so it is not re-adopted
        m.alive = false;
        m.started = false;
        m.owner = -1;
        m.cpu = 0;
        gone.emplace_back(name, stop);
      }
    }
    for (const auto& [name, pid] : gone) {
      policy_->forget(name);
      emit_simple(name, "released", pid > 0 ? "launcher exited; stopping our pid " + std::to_string(pid)
                                            : "launcher exited");
      if (pid > 0) {
        const double g = spec_.p.term_grace_s;
        start_job(name, [this, pid, g] { sup_.terminate(pid, g); return true; }, false);
      }
    }
  }

  template <class Srv>
  typename Srv::Response::SharedPtr call(typename rclcpp::Client<Srv>::SharedPtr c,
                                          typename Srv::Request::SharedPtr req, double timeout) {
    auto fr = c->async_send_request(req);
    if (fr.future.wait_for(std::chrono::duration<double>(timeout)) != std::future_status::ready) {
      c->remove_pending_request(fr.request_id);
      return nullptr;
    }
    return fr.future.get();
  }

  uint8_t get_state(const Managed& m, double timeout) {
    auto r = call<lifecycle_msgs::srv::GetState>(m.get, std::make_shared<lifecycle_msgs::srv::GetState::Request>(),
                                                 timeout);
    return r ? r->current_state.id : 0;
  }

  bool transition(const Managed& m, uint8_t id) {
    auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    req->transition.id = id;
    auto r = call<lifecycle_msgs::srv::ChangeState>(m.change, req, spec_.p.lifecycle_timeout_s);
    return r && r->success;
  }

  bool bring_up(const std::string& node, double wait_s) {
    const auto& m = nodes_.at(node);
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(wait_s);
    while (!m.get->service_is_ready() || !m.change->service_is_ready()) {
      if (stopping_ || std::chrono::steady_clock::now() > end) return false;
      std::this_thread::sleep_for(20ms);
    }
    // After a SIGKILL the dead process's services can still look ready and a
    // request sent before the new server is matched is lost, so retry until
    // the deadline instead of trusting one call.
    uint8_t s = 0;
    int tries = 0;
    while (true) {
      ++tries;
      s = get_state(m, std::min(0.5, spec_.p.lifecycle_timeout_s));
      if (s != 0 || stopping_ || std::chrono::steady_clock::now() > end) break;
      std::this_thread::sleep_for(50ms);
    }
    if (tries > 1) RCLCPP_INFO(get_logger(), "%s get_state answered after %d tries (state %u)", node.c_str(), tries, s);
    if (s == State::PRIMARY_STATE_UNCONFIGURED) {
      if (!transition(m, Transition::TRANSITION_CONFIGURE)) return false;
      s = State::PRIMARY_STATE_INACTIVE;
    }
    if (s == State::PRIMARY_STATE_INACTIVE) return transition(m, Transition::TRANSITION_ACTIVATE);
    return s == State::PRIMARY_STATE_ACTIVE;
  }

  bool restart(const std::string& node, double delay, hh::Fault f = hh::Fault::None) {
    // A hung or spinning process will not shut down cleanly; waiting for it only adds to the outage.
    const bool hard = spec_.p.fast_repair > 0 && (f == hh::Fault::Hang || f == hh::Fault::CpuRunaway);
    kill(node, hard ? 0.0 : -1);
    if (!sleep_for(delay)) return false;
    return spawn_and_bring_up(node, false);
  }

  bool lifecycle_reset(const std::string& node) {
    const auto& m = nodes_.at(node);
    if (!m.get || !m.change) return false;
    const uint8_t s = get_state(m, spec_.p.lifecycle_timeout_s);
    if (s == 0) return false;  // not answering: the executor is stuck, a restart is needed
    if (s == State::PRIMARY_STATE_ACTIVE && !transition(m, Transition::TRANSITION_DEACTIVATE)) return false;
    if (s == State::PRIMARY_STATE_ACTIVE || s == State::PRIMARY_STATE_INACTIVE)
      if (!transition(m, Transition::TRANSITION_CLEANUP)) return false;
    if (!transition(m, Transition::TRANSITION_CONFIGURE)) return false;
    return transition(m, Transition::TRANSITION_ACTIVATE);
  }

  bool fallback(const std::string& node) {
    std::string backup;
    {
      std::lock_guard lk(mu_);
      auto& m = nodes_.at(node);
      m.retired = true;
      backup = m.spec.fallback;
    }
    if (backup.empty()) {
      // No replacement: stop supervising and ask the rest of the system to stop safely.
      publish_mode("SAFE_STOP " + node);
      return true;
    }
    kill(node);  // the backup takes over the same topics; two publishers would mask a failure
    const bool ok = spawn_and_bring_up(backup, true);
    publish_mode((ok ? "DEGRADED " : "SAFE_STOP ") + node + "->" + backup);
    return ok;
  }

  // ---------- members ----------
  std::chrono::steady_clock::time_point t0_;
  hh::Spec spec_;
  std::unique_ptr<hh::Policy> policy_;
  std::string log_dir_;
  std::ofstream events_, snaps_;
  std::mutex log_mu_;
  double last_snap_ = -1;

  hh::Supervisor sup_;
  std::mutex mu_;  // guards nodes_ fields that jobs touch
  std::map<std::string, Managed> nodes_;
  std::map<std::string, TopicState> topics_;  // executor thread only
  std::set<std::pair<std::string, std::string>> own_;  // (node, topic) currently advertised
  std::map<std::string, std::vector<std::string>> producers_;  // topic -> declaring nodes (const after ctor)
  std::map<std::pair<std::string, std::string>, EnvSlot> env_;  // executor thread only

  std::mutex res_mu_;
  std::vector<Result> results_;
  std::vector<Job> jobs_;
  std::atomic<bool> stopping_{false};

  // agent policy
  bool agent_ = false;
  double lkg_hold_s_ = kLkgHoldS;
  double gen_probation_s_ = 30.0;
  std::string gen_ack_dir_;
  struct AckFile { int64_t mtime = 0, size = -1; std::vector<uint64_t> hashes; };
  std::map<std::string, AckFile> acks_;  // by node, re-read only when the file changes

  // Hashes acknowledged for `node` (empty when there is no ack file). Called under mu_. *fresh is set
  // when the file was (re)read on this call.
  const std::vector<uint64_t>& acked(const std::string& node, bool* fresh = nullptr) {
    auto& a = acks_[node];
    const std::string path = gen_ack_dir_ + "/" + node + ".ack";
    struct stat st {};
    if (::stat(path.c_str(), &st) != 0) {
      a = AckFile{};
      return a.hashes;
    }
    const int64_t mt = static_cast<int64_t>(st.st_mtim.tv_sec) * 1000000000LL + st.st_mtim.tv_nsec;
    if (a.mtime != mt || a.size != st.st_size) {
      std::ifstream in(path);
      std::stringstream ss;
      ss << in.rdbuf();
      a = {mt, static_cast<int64_t>(st.st_size), helix_healer::parse_ack(ss.str())};
      if (fresh) *fresh = true;
    }
    return a.hashes;
  }
  std::unique_ptr<hh::Ranker> ranker_;
  std::mutex ranker_mu_, pend_mu_;
  std::map<std::string, Pending> pending_;
  std::map<std::string, std::string> ros_alias_;  // ROS fully qualified name -> spec name
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_sub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<helix_msgs::msg::HealEvent>::SharedPtr ev_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr tick_, graph_, res_, diag_;
};

int main(int argc, char** argv) {
  // Experiment runners record this so every result names the code that produced it.
  if (argc > 1 && std::string(argv[1]) == "--build-info") {
    printf("{\"commit\": \"%s\", \"dirty\": %s}\n", HELIX_BUILD_COMMIT, HELIX_BUILD_DIRTY ? "true" : "false");
    return 0;
  }
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    auto node = std::make_shared<Healer>();
    rclcpp::executors::SingleThreadedExecutor ex;
    ex.add_node(node);
    ex.spin();
    ex.remove_node(node);
  } catch (const std::exception& e) {
    fprintf(stderr, "helix_healer: %s\n", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
