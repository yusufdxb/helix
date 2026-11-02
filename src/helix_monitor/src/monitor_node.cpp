// helix_monitor_node: observe-only ROS 2 wrapper around helix_monitor_core.
//
// Subscribes to the GO2 sensor topics, feeds every message to the same Monitor and StopVerifier
// that eval/helix_eval.py scores offline (identical payload layout), and publishes alarm
// transitions and stop verdicts. It has no publisher on any command topic: it never actuates.
//
// Payload layout per stream (must match eval/helix_eval.py load()):
//   lidar_imu  stamp = header.stamp            v = gyro xyz, accel xyz
//   sport      stamp = SportModeState.stamp    v = imu gyro xyz, imu accel xyz, velocity xyz, yaw_speed
//   low        stamp = LowState.tick * 1e-3    v = imu gyro xyz, imu accel xyz, motor dq 0..11
// A stop is commanded when /api/sport/request carries api_id 1003 (StopMove).
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>
#include <sensor_msgs/msg/imu.hpp>
#include <unitree_api/msg/request.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>

#include "helix_monitor/config.hpp"
#include "helix_msgs/msg/fault_event.hpp"
#include "helix_msgs/msg/stop_verdict.hpp"

namespace {

constexpr int64_t kStopMove = 1003;

template <typename Imu>
void push_imu(std::vector<double>& v, const Imu& imu) {
  for (int i = 0; i < 3; ++i) v.push_back(imu.gyroscope[i]);
  for (int i = 0; i < 3; ++i) v.push_back(imu.accelerometer[i]);
}

std::string upper(std::string s) {
  for (auto& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  return s;
}

}  // namespace

class HelixMonitorNode : public rclcpp::Node {
 public:
  HelixMonitorNode() : Node("helix_monitor"), clock_(RCL_STEADY_TIME) {
    std::string cfg = declare_parameter<std::string>("config", "");
    if (cfg.empty()) cfg = ament_index_cpp::get_package_share_directory("helix_monitor") + "/config/go2.cfg";
    const double tick_hz = declare_parameter<double>("tick_hz", 50.0);
    cfg_ = helix_monitor::load_config(cfg);
    sv_ = std::make_unique<helix_monitor::StopVerifier>(cfg_->groups, cfg_->stop);

    fault_pub_ = create_publisher<helix_msgs::msg::FaultEvent>("/helix/faults", rclcpp::QoS(50).reliable());
    stop_pub_ = create_publisher<helix_msgs::msg::StopVerdict>("/helix/stop_verdict", rclcpp::QoS(10).reliable());

    const auto sensor = rclcpp::SensorDataQoS();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/utlidar/imu", sensor, [this](const sensor_msgs::msg::Imu& m) {
          s_.stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9;
          // float64 on the wire; rounded to float32 like the offline harness so thresholds transfer exactly.
          auto f = [](double x) { return static_cast<double>(static_cast<float>(x)); };
          s_.v = {f(m.angular_velocity.x), f(m.angular_velocity.y), f(m.angular_velocity.z),
                  f(m.linear_acceleration.x), f(m.linear_acceleration.y), f(m.linear_acceleration.z)};
          feed("lidar_imu");
        });
    sport_sub_ = create_subscription<unitree_go::msg::SportModeState>(
        "/sportmodestate", sensor, [this](const unitree_go::msg::SportModeState& m) {
          s_.stamp = m.stamp.sec + m.stamp.nanosec * 1e-9;
          s_.v.clear();
          push_imu(s_.v, m.imu_state);
          for (int i = 0; i < 3; ++i) s_.v.push_back(m.velocity[i]);
          s_.v.push_back(m.yaw_speed);
          feed("sport");
        });
    low_sub_ = create_subscription<unitree_go::msg::LowState>(
        "/lowstate", sensor, [this](const unitree_go::msg::LowState& m) {
          s_.stamp = m.tick * 1e-3;
          s_.v.clear();
          push_imu(s_.v, m.imu_state);
          for (int i = 0; i < 12; ++i) s_.v.push_back(m.motor_state[i].dq);
          feed("low");
        });
    if (cfg_->has_stop) {
      req_sub_ = create_subscription<unitree_api::msg::Request>(
          "/api/sport/request", rclcpp::QoS(50), [this](const unitree_api::msg::Request& m) {
            if (m.header.identity.api_id != kStopMove) return;
            const double t = now_s();
            sv_->command(t);
            RCLCPP_INFO(get_logger(), "StopMove seen at %.3f: verifying", t);
          });
    }
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / tick_hz), [this] { step(now_s()); });
    t_start_ = now_s();
    const std::string rmw = rmw_get_implementation_identifier();
    if (rmw != "rmw_cyclonedds_cpp")
      RCLCPP_WARN(get_logger(), "RMW is %s, not rmw_cyclonedds_cpp: GO2 SportModeState samples are larger than the "
                  "unitree_go type and other RMWs can drop them silently", rmw.c_str());
    RCLCPP_INFO(get_logger(), "helix_monitor: %zu checks, %zu evidence groups, config %s (observe-only)",
                cfg_->monitor.checks().size(), cfg_->groups.size(), cfg.c_str());
  }

 private:
  double now_s() { return clock_.now().seconds(); }

  void feed(const char* stream) {
    ++seen_[stream];
    s_.t = now_s();
    ev_.clear();
    cfg_->monitor.on_sample(stream, s_, ev_);
    sv_->on_sample(stream, s_);
    publish(ev_);
  }

  void step(double t) {
    // A stream that never appears is not monitored by its rate check, so say so once, loudly.
    if (!silence_reported_ && t - t_start_ > 5.0) {
      silence_reported_ = true;
      for (const char* st : {"lidar_imu", "sport", "low"})
        if (!seen_[st]) RCLCPP_WARN(get_logger(), "no samples on %s after 5 s: its checks are inactive", st);
    }
    ev_.clear();
    cfg_->monitor.tick(t, ev_);
    publish(ev_);
    helix_monitor::StopResult r;
    if (!sv_->tick(t, r)) return;
    helix_msgs::msg::StopVerdict out;
    out.t_cmd = r.t_cmd;
    out.t = r.t;
    out.verdict = helix_monitor::to_string(r.verdict);
    for (const auto& g : sv_->groups()) out.groups.push_back(g.name);
    out.metrics = r.metric;
    stop_pub_->publish(out);
    RCLCPP_INFO(get_logger(), "stop at %.3f: %s after %.2f s", r.t_cmd, out.verdict.c_str(), r.t - r.t_cmd);
  }

  void publish(const std::vector<helix_monitor::Event>& ev) {
    for (const auto& e : ev) {
      helix_msgs::msg::FaultEvent f;
      f.node_name = get_name();
      f.fault_type = upper(e.check);
      f.severity = e.raised ? 2 : 0;
      f.timestamp = e.t;
      f.detail = e.check + " on " + e.stream + (e.raised ? " raised" : " cleared");
      f.context_keys = {"stream", "state", "value"};
      f.context_values = {e.stream, e.raised ? "raised" : "cleared", std::to_string(e.value)};
      fault_pub_->publish(f);
      if (e.raised) RCLCPP_WARN(get_logger(), "%s (value %.4g)", f.detail.c_str(), e.value);
    }
  }

  rclcpp::Clock clock_;
  double t_start_ = 0.0;
  bool silence_reported_ = false;
  std::map<std::string, std::size_t> seen_;
  std::unique_ptr<helix_monitor::Config> cfg_;
  std::unique_ptr<helix_monitor::StopVerifier> sv_;
  helix_monitor::Sample s_;
  std::vector<helix_monitor::Event> ev_;
  rclcpp::Publisher<helix_msgs::msg::FaultEvent>::SharedPtr fault_pub_;
  rclcpp::Publisher<helix_msgs::msg::StopVerdict>::SharedPtr stop_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_sub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_sub_;
  rclcpp::Subscription<unitree_api::msg::Request>::SharedPtr req_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HelixMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
