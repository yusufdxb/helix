// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// helix_go2_sport_sink (C++): arbiter output -> /api/sport/request.
// Native port of helix_arbiter/go2_sport_sink.py: same node name, read-only
// parameters, topics, QoS, trace JSON and SinkStatus. Mode is fixed at
// startup:
//
//   dry_run   (default) nothing is sent to the robot; every decision is traced
//   stop_only only StopMove (1003) is ever sent
//   armed     Move (1008) allowed within the sink limits
//
// On SIGINT/SIGTERM a StopMove burst is sent before exit (not in dry_run).
// Feedback (feedback:=true): /api/sport/response is matched to this sink's
// own request ids and SportModeState gives measured motion; both go out on
// /helix/sink/status (20 Hz and on ack/stop edges), stamped with this
// host's monotonic clock.

#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <charconv>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "helix_go2_sink_cpp/sink_core.hpp"
#include "helix_msgs/msg/sink_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

namespace helix_go2_sink_cpp
{
namespace
{

std::atomic<int> g_signal{0};

void on_signal(int sig) {g_signal.store(sig);}

double mono_now()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

double wall_now()
{
  return std::chrono::duration<double>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

// A number as Python's json.dumps writes it (NaN/Infinity included).
std::string jnum(double v)
{
  if (std::isnan(v)) {
    return "NaN";
  }
  if (std::isinf(v)) {
    return v > 0 ? "Infinity" : "-Infinity";
  }
  char buf[64];
  const auto r = std::to_chars(buf, buf + sizeof(buf), v);
  std::string s(buf, r.ptr);
  if (s.find_first_of(".e") == std::string::npos) {
    s += ".0";
  }
  return s;
}

// The core trips the deadman strictly after input_timeout_sec, so the
// deadline timer fires just past it.
constexpr double kDeadlineMarginSec = 0.002;

}  // namespace

class Go2SportSink : public rclcpp::Node
{
public:
  Go2SportSink()
  : Node("helix_go2_sport_sink")
  {
    rcl_interfaces::msg::ParameterDescriptor ro;
    ro.read_only = true;
    const auto mode = declare_parameter<std::string>("mode", "dry_run", ro);
    const auto input_topic = declare_parameter<std::string>("input_topic", "/cmd_vel", ro);
    const auto request_topic =
      declare_parameter<std::string>("request_topic", "/api/sport/request", ro);
    const auto trace_topic = declare_parameter<std::string>("trace_topic", "/helix/sink/trace", ro);
    Limits lim;
    lim.max_vx = declare_parameter<double>("max_vx", 0.25, ro);
    lim.max_vy = declare_parameter<double>("max_vy", 0.20, ro);
    lim.max_wz = declare_parameter<double>("max_wz", 0.50, ro);
    const double timeout = declare_parameter<double>("input_timeout_sec", 0.25, ro);
    const bool feedback = declare_parameter<bool>("feedback", true, ro);
    const auto response_topic =
      declare_parameter<std::string>("response_topic", "/api/sport/response", ro);
    const auto state_topic =
      declare_parameter<std::string>("state_topic", "/lf/sportmodestate", ro);
    const auto status_topic =
      declare_parameter<std::string>("status_topic", "/helix/sink/status", ro);
    StillThresholds still;
    still.max_speed_xy = declare_parameter<double>("still_max_speed_xy", 0.05, ro);
    still.max_yaw_speed = declare_parameter<double>("still_max_yaw_speed", 0.10, ro);
    still.still_s = declare_parameter<double>("still_s", 0.3, ro);

    logic_ = std::make_unique<SinkLogic>(parse_mode(mode), lim, timeout);
    fb_ = std::make_unique<SinkFeedback>(still);
    if (logic_->mode() != Mode::kDryRun) {
      pub_req_ = create_publisher<unitree_api::msg::Request>(request_topic, 10);
    }
    pub_trace_ = create_publisher<std_msgs::msg::String>(
      trace_topic, rclcpp::QoS(100).reliable().durability_volatile());
    req_id_ = static_cast<std::int64_t>(wall_now() * 1000.0) % 1000000000LL;
    sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic, rclcpp::QoS(10).reliable().durability_volatile(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {on_cmd(*m);});
    // The 50 ms tick catches silence at startup and repeats StopMove; the
    // deadline timer is restarted by every input and fires just past the
    // timeout, so the deadman trips at the deadline itself. Wall timers in
    // rclcpp run on the steady clock.
    tick_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {on_tick();});
    deadline_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(timeout + kDeadlineMarginSec)),
      [this]() {on_tick();});
    if (feedback) {
      const auto state_qos = rclcpp::QoS(10).best_effort().durability_volatile();
      pub_status_ = create_publisher<helix_msgs::msg::SinkStatus>(
        status_topic, rclcpp::QoS(20).reliable().durability_volatile());
      sub_resp_ = create_subscription<unitree_api::msg::Response>(
        response_topic, state_qos,
        [this](unitree_api::msg::Response::ConstSharedPtr m) {on_response(*m);});
      sub_state_ = create_subscription<unitree_go::msg::SportModeState>(
        state_topic, state_qos,
        [this](unitree_go::msg::SportModeState::ConstSharedPtr m) {on_state(*m);});
      status_timer_ = create_wall_timer(
        std::chrono::milliseconds(50), [this]() {publish_status();});
    }
    RCLCPP_WARN(get_logger(), "GO2 sport sink (C++) up in mode=%s", mode.c_str());
  }

  void stop_burst(int n = 3)
  {
    Request r;
    r.api_id = kApiStopMove;
    r.reason = "SHUTDOWN";
    for (int i = 0; i < n; ++i) {
      send(r, std::nullopt);
    }
  }

private:
  using Cmd = std::optional<std::array<double, 3>>;

  void on_cmd(const geometry_msgs::msg::Twist & m)
  {
    deadline_->reset();
    if (auto r = logic_->on_command(m.linear.x, m.linear.y, m.angular.z, mono_now())) {
      send(*r, std::array<double, 3>{m.linear.x, m.linear.y, m.angular.z});
    }
  }

  void on_tick()
  {
    if (auto r = logic_->on_tick(mono_now())) {
      send(*r, std::nullopt);
    }
  }

  void send(const Request & r, const Cmd & in)
  {
    if (r.api_id != kApiStopMove && r.api_id != kApiMove) {
      throw std::logic_error("api_id refused");   // never Damp (1001)
    }
    ++req_id_;
    bool sent = false;
    if (pub_req_) {
      req_msg_.header.identity.id = req_id_;
      req_msg_.header.identity.api_id = r.api_id;
      req_msg_.header.lease.id = 0;
      req_msg_.header.policy.priority = 0;
      req_msg_.header.policy.noreply = false;
      req_msg_.parameter = r.parameter;
      pub_req_->publish(req_msg_);
      sent = true;
      fb_->on_sent(req_id_, r.api_id, r.reason, mono_now());
    }
    std::string j = "{\"t_wall\": " + jnum(wall_now()) + ", \"t_mono\": " + jnum(mono_now()) +
      ", \"mode\": \"" + mode_name(logic_->mode()) + "\", \"api_id\": " +
      std::to_string(r.api_id) + ", \"reason\": \"" + r.reason + "\", \"x\": " + jnum(r.x) +
      ", \"y\": " + jnum(r.y) + ", \"z\": " + jnum(r.z) + ", \"input\": ";
    j += in ? "[" + jnum((*in)[0]) + ", " + jnum((*in)[1]) + ", " + jnum((*in)[2]) + "]" : "null";
    j += ", \"request_id\": " + std::to_string(req_id_) + ", \"sent_to_robot\": " +
      (sent ? "true" : "false") + ", \"impl\": \"cpp\"}";
    std_msgs::msg::String t;
    t.data = std::move(j);
    pub_trace_->publish(t);
    if (r.reason != "MOVE") {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000, "StopMove (%s) sent_to_robot=%s", r.reason.c_str(),
        sent ? "true" : "false");
    }
  }

  void on_response(const unitree_api::msg::Response & m)
  {
    const bool was = fb_->stop_acked;
    const bool mine = fb_->on_response(
      m.header.identity.id, static_cast<std::int32_t>(m.header.identity.api_id),
      m.header.status.code, mono_now());
    if (mine && fb_->stop_acked && !was) {
      publish_status();
    }
  }

  void on_state(const unitree_go::msg::SportModeState & m)
  {
    const bool was = fb_->stopped;
    fb_->on_state(m.velocity[0], m.velocity[1], m.yaw_speed, m.mode, mono_now());
    if (fb_->stopped != was) {
      publish_status();
    }
  }

  void publish_status()
  {
    if (!pub_status_) {
      return;
    }
    const double now = mono_now();
    const SinkFeedback & f = *fb_;
    helix_msgs::msg::SinkStatus s;
    s.mode = mode_name(logic_->mode());
    s.feedback_available = f.feedback_available(now);
    s.last_api_id = f.last_api_id;
    s.last_request_id = f.last_request_id;
    s.last_reason = f.last_reason;
    s.last_sent_at = f.last_sent_at;
    s.last_ack_request_id = f.last_ack_request_id;
    s.last_ack_code = f.last_ack_code;
    s.last_ack_at = f.last_ack_at;
    s.stop_acked = f.stop_acked;
    s.stop_ack_sent_at = f.stop_ack_sent_at;
    s.stop_acked_at = f.stop_acked_at;
    s.speed_xy = f.speed_xy;
    s.yaw_speed = f.yaw_speed;
    s.state_at = f.state_at;
    s.robot_mode = f.robot_mode;
    s.stopped = f.stopped_now(now);
    s.stopped_since = f.stopped_since;
    s.seq = ++status_seq_;
    pub_status_->publish(s);
  }

  std::unique_ptr<SinkLogic> logic_;
  std::unique_ptr<SinkFeedback> fb_;
  std::int64_t req_id_{0};
  std::uint64_t status_seq_{0};
  unitree_api::msg::Request req_msg_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr pub_req_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_trace_;
  rclcpp::Publisher<helix_msgs::msg::SinkStatus>::SharedPtr pub_status_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr sub_resp_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sub_state_;
  rclcpp::TimerBase::SharedPtr tick_;
  rclcpp::TimerBase::SharedPtr deadline_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace helix_go2_sink_cpp

int main(int argc, char ** argv)
{
  using helix_go2_sink_cpp::g_signal;
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, helix_go2_sink_cpp::on_signal);
  std::signal(SIGTERM, helix_go2_sink_cpp::on_signal);
  int rc = 0;
  {
    std::shared_ptr<helix_go2_sink_cpp::Go2SportSink> node;
    try {
      node = std::make_shared<helix_go2_sink_cpp::Go2SportSink>();
    } catch (const std::exception & e) {
      RCLCPP_FATAL(rclcpp::get_logger("helix_go2_sport_sink"), "%s", e.what());
      rclcpp::shutdown();
      return 1;
    }
    rclcpp::executors::SingleThreadedExecutor ex;
    ex.add_node(node);
    while (g_signal.load() == 0 && rclcpp::ok()) {
      ex.spin_once(std::chrono::milliseconds(50));
    }
    node->stop_burst();
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    while (std::chrono::steady_clock::now() < end) {
      ex.spin_once(std::chrono::milliseconds(20));
    }
    ex.remove_node(node);
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return rc;
}
