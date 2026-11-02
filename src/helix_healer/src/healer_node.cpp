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
#include <helix_healer/envelope.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <signal.h>

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
#include <sstream>
#include <string>
#include <thread>
#include <vector>

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

struct TopicState {
  std::deque<double> arrivals;
  double last = -1;
  double first = -1;
  double learned_hz = 0;
  rclcpp::GenericSubscription::SharedPtr sub;
};

// Adaptive envelope of one (producer, topic). Reset when the producer's process changes.
struct EnvSlot {
  hh::Envelope e;
  double start_t = -1e18;
};

struct Managed {
  hh::NodeSpec spec;
  std::vector<std::string> argv;  // resolved
  pid_t pid = -1;
  bool alive = false;
  bool started = false;
  bool retired = false;
  double start_t = 0;
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
};

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
      nodes_.emplace(ns.name, std::move(m));
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
    RCLCPP_INFO(get_logger(), "helix_healer: policy=%s, %zu managed nodes", policy_name.c_str(), nodes_.size());
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
        if (slot.start_t != m.start_t) slot.e.reset_process(), slot.start_t = m.start_t;
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
      if (slot.start_t == m.start_t && slot.e.ready()) {
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
    for (const auto& d : out.decisions) dispatch(d);
    {
      std::lock_guard lk(mu_);
      for (auto& [name, m] : nodes_)
        m.learn_ok = m.alive && t - std::max(m.start_t, m.watch_t) >= spec_.p.startup_grace_s && policy_->healthy(name);
    }
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
    auto done = std::make_shared<std::atomic<bool>>(false);
    Job j{node, std::jthread([this, node, fn = std::move(fn), report, a, done] {
            bool ok = false;
            std::string err;
            try {
              ok = fn();
            } catch (const std::exception& e) {
              err = e.what();
            }
            if (report) {
              std::lock_guard lk(res_mu_);
              results_.push_back({node, a, ok, err});
            }
            done->store(true);
          }),
          done};
    jobs_.push_back(std::move(j));
  }

  void dispatch(const hh::Decision& d) {
    const std::string node = d.node;
    switch (d.action) {
      case hh::Action::Restart:
        start_job(node, [this, node, delay = d.delay_s, f = d.fault] { return restart(node, delay, f); }, true,
                  d.action);
        break;
      case hh::Action::LifecycleReset:
        start_job(node, [this, node] { return lifecycle_reset(node); }, true, d.action);
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
      case hh::Action::None:
        break;
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
      pid = sup_.spawn(m.argv, log, o);
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

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<helix_msgs::msg::HealEvent>::SharedPtr ev_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr tick_, graph_, res_, diag_;
};

int main(int argc, char** argv) {
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
