// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// C++ rclcpp_lifecycle drop-in replacement for
// helix_adapter.topic_rate_monitor. Subscribes to configured topics,
// counts callback arrivals in a sliding window, publishes each topic's
// rate as a labeled Float64MultiArray on /helix/metrics.
//
// Contract matches the Python node exactly:
//   node name:      helix_topic_rate_monitor
//   parameters:     window_sec (5.0), publish_period_sec (0.5),
//                   topics (GO2 default 6), sim_mode (false)
//   metric label:   rate_hz/<slug(topic)> where slug replaces '/' with '_'
//                   and strips leading underscores
//   qos:            best-effort volatile for sensor_msgs topics
//                   (Imu, PointCloud2); default reliable depth-10 otherwise
// Why C++: on the Jetson the 250 Hz /utlidar/imu callback in rclpy costs
// ~42 percentage points of one core. The callback itself is trivial; the
// cost is rclpy executor dispatch + message object construction. The
// typed C++ subscription path avoids both.

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include "helix_adapter_cpp/rate_window.hpp"

namespace
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

const std::vector<std::string> kDefaultTopics = {
  "/utlidar/imu", "/utlidar/robot_odom", "/utlidar/robot_pose",
  "/utlidar/cloud", "/gnss", "/multiplestate",
};

std::string slugify(const std::string & topic)
{
  std::string s;
  s.reserve(topic.size());
  for (char c : topic) {
    s.push_back(c == '/' ? '_' : c);
  }
  std::size_t start = 0;
  while (start < s.size() && s[start] == '_') {
    ++start;
  }
  return s.substr(start);
}

bool is_best_effort_topic(const std::string & topic)
{
  return topic == "/utlidar/imu" || topic == "/utlidar/cloud" ||
         topic == "/utlidar/cloud_throttled";
}

rclcpp::QoS qos_for(const std::string & topic)
{
  rclcpp::QoS qos(10);
  if (is_best_effort_topic(topic)) {
    qos.best_effort();
    qos.durability_volatile();
  }
  return qos;
}

}  // namespace

class TopicRateMonitor : public rclcpp_lifecycle::LifecycleNode
{
public:
  TopicRateMonitor()
  : LifecycleNode("helix_topic_rate_monitor")
  {
    declare_parameter<double>("window_sec", 5.0);
    declare_parameter<double>("publish_period_sec", 0.5);
    declare_parameter<std::vector<std::string>>("topics", kDefaultTopics);
    declare_parameter<bool>("sim_mode", false);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    const double window_sec = get_parameter("window_sec").as_double();
    const auto topics = resolve_topics();

    for (const auto & topic : topics) {
      if (!make_subscription(topic, window_sec)) {
        RCLCPP_ERROR(
          get_logger(),
          "Unknown topic type for '%s'; skipping", topic.c_str());
      }
    }

    metrics_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/helix/metrics", 10);

    RCLCPP_INFO(
      get_logger(),
      "TopicRateMonitor configured - %zu topics, window=%.1fs",
      windows_.size(), window_sec);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    LifecycleNode::on_activate(state);
    const double period_s = get_parameter("publish_period_sec").as_double();
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(period_s));
    timer_ = create_wall_timer(period, [this]() {publish_rates();});
    RCLCPP_INFO(get_logger(), "TopicRateMonitor activated.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    LifecycleNode::on_deactivate(state);
    if (timer_) {
      timer_->cancel();
      timer_.reset();
    }
    RCLCPP_INFO(get_logger(), "TopicRateMonitor deactivated.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    subs_.clear();
    windows_.clear();
    metrics_pub_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & s) override
  {
    return on_cleanup(s);
  }

private:
  std::vector<std::string> resolve_topics()
  {
    auto topics = get_parameter("topics").as_string_array();
    const bool sim_mode = get_parameter("sim_mode").as_bool();
    if (sim_mode) {
      for (auto & t : topics) {
        if (t == "/utlidar/cloud") {
          t = "/utlidar/cloud_throttled";
        }
      }
    }
    return topics;
  }

  template<typename MsgT>
  void subscribe_typed(const std::string & topic, double window_sec)
  {
    auto window = std::make_shared<helix_adapter_cpp::RateWindow>(window_sec);
    windows_.emplace(topic, window);
    auto sub = create_subscription<MsgT>(
      topic, qos_for(topic),
      [window](typename MsgT::ConstSharedPtr) {window->record();});
    subs_.push_back(std::move(sub));
  }

  bool make_subscription(const std::string & topic, double window_sec)
  {
    if (topic == "/utlidar/imu") {
      subscribe_typed<sensor_msgs::msg::Imu>(topic, window_sec);
      return true;
    }
    if (topic == "/utlidar/robot_odom") {
      subscribe_typed<nav_msgs::msg::Odometry>(topic, window_sec);
      return true;
    }
    if (topic == "/utlidar/robot_pose") {
      subscribe_typed<geometry_msgs::msg::PoseStamped>(topic, window_sec);
      return true;
    }
    if (topic == "/utlidar/cloud" || topic == "/utlidar/cloud_throttled") {
      subscribe_typed<sensor_msgs::msg::PointCloud2>(topic, window_sec);
      return true;
    }
    if (topic == "/gnss" || topic == "/multiplestate") {
      subscribe_typed<std_msgs::msg::String>(topic, window_sec);
      return true;
    }
    return false;
  }

  void publish_rates()
  {
    if (!metrics_pub_) {
      return;
    }
    for (const auto & [topic, window] : windows_) {
      std_msgs::msg::Float64MultiArray msg;
      std_msgs::msg::MultiArrayDimension dim;
      dim.label = "rate_hz/" + slugify(topic);
      dim.size = 1;
      dim.stride = 1;
      msg.layout.dim.push_back(std::move(dim));
      msg.data.push_back(window->rate_or_nan());
      metrics_pub_->publish(msg);
    }
  }

  std::unordered_map<std::string, std::shared_ptr<helix_adapter_cpp::RateWindow>>
  windows_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  std::shared_ptr<
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>>
  metrics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TopicRateMonitor>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
