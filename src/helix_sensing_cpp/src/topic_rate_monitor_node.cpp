// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// TopicRateMonitorNode: native port of helix_adapter.topic_rate_monitor.
// Same node name (helix_topic_rate_monitor), parameters, QoS choice and
// /helix/metrics output (rate_hz/<topic without slashes>, NaN when stale),
// so it drops in for the Python node.
//
// Subscriptions are generic (serialized): only arrival times are counted, so
// no message is ever deserialized, including each LiDAR PointCloud2.
//
// Extra parameter: stale_after_s (default 0.0 = reference behavior); see
// rate_window.hpp.

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "helix_sensing_cpp/heartbeat.hpp"
#include "helix_sensing_cpp/rate_window.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace helix_sensing_cpp
{

namespace
{

// Same defaults and type registry as the Python node.
const std::vector<std::string> kDefaultTopics{
  "/utlidar/imu", "/utlidar/robot_odom", "/utlidar/robot_pose", "/utlidar/cloud",
  "/gnss", "/multiplestate"};

const std::map<std::string, std::string> kTypeMap{
  {"/utlidar/imu", "sensor_msgs/msg/Imu"},
  {"/utlidar/robot_odom", "nav_msgs/msg/Odometry"},
  {"/utlidar/robot_pose", "geometry_msgs/msg/PoseStamped"},
  {"/utlidar/cloud", "sensor_msgs/msg/PointCloud2"},
  {"/utlidar/cloud_throttled", "sensor_msgs/msg/PointCloud2"},
  {"/gnss", "std_msgs/msg/String"},
  {"/multiplestate", "std_msgs/msg/String"},
};

double steady_now()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string metric_name(const std::string & topic)
{
  std::string clean = topic;
  for (auto & c : clean) {
    if (c == '/') {
      c = '_';
    }
  }
  const auto first = clean.find_first_not_of('_');
  return "rate_hz/" + (first == std::string::npos ? std::string() : clean.substr(first));
}

}  // namespace

class TopicRateMonitorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit TopicRateMonitorNode(const rclcpp::NodeOptions & options)
  : LifecycleNode("helix_topic_rate_monitor", options)
  {
    declare_parameter("window_sec", 5.0);
    declare_parameter("publish_period_sec", 0.5);
    declare_parameter("topics", kDefaultTopics);
    declare_parameter("topic_types", std::vector<std::string>{});
    declare_parameter("sim_mode", false);
    declare_parameter("stale_after_s", 0.0);
    heartbeat_ = std::make_unique<Heartbeat>(this);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    const double window = get_parameter("window_sec").as_double();
    const double stale = get_parameter("stale_after_s").as_double();
    auto topics = get_parameter("topics").as_string_array();
    const auto types = get_parameter("topic_types").as_string_array();
    if (!types.empty() && types.size() != topics.size()) {
      RCLCPP_ERROR(get_logger(), "topic_types must be empty or match topics in length");
      return CallbackReturn::FAILURE;
    }
    if (!(get_parameter("publish_period_sec").as_double() > 0.0)) {
      RCLCPP_ERROR(get_logger(), "publish_period_sec must be > 0");
      return CallbackReturn::FAILURE;
    }
    if (get_parameter("sim_mode").as_bool()) {
      for (auto & t : topics) {
        if (t == "/utlidar/cloud") {
          t = "/utlidar/cloud_throttled";
        }
      }
    }
    try {
      for (std::size_t i = 0; i < topics.size(); ++i) {
        const std::string & topic = topics[i];
        std::string type;
        if (!types.empty()) {
          type = types[i];
        } else if (auto it = kTypeMap.find(topic); it != kTypeMap.end()) {
          type = it->second;
        } else {
          RCLCPP_ERROR(get_logger(), "Unknown topic type for '%s'; skipping", topic.c_str());
          continue;
        }
        auto w = std::make_shared<Watched>(topic, window, stale);
        // Sensor streams are best-effort on the GO2 (as in the Python node).
        const bool best_effort = type == "sensor_msgs/msg/Imu" ||
          type == "sensor_msgs/msg/PointCloud2";
        rclcpp::QoS qos(10);
        if (best_effort) {
          qos.best_effort();
        } else {
          qos.reliable();
        }
        qos.durability_volatile();
        w->sub = create_generic_subscription(
          topic, type, qos,
          [w](std::shared_ptr<rclcpp::SerializedMessage>) {w->window.record(steady_now());});
        watched_.push_back(w);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "refusing to configure: %s", e.what());
      watched_.clear();
      return CallbackReturn::FAILURE;
    }
    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/helix/metrics", 10);
    RCLCPP_INFO(
      get_logger(), "TopicRateMonitor configured, %zu topics, window=%.1fs stale_after=%.2fs",
      watched_.size(), window, stale);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    LifecycleNode::on_activate(state);
    heartbeat_->start();
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(get_parameter("publish_period_sec").as_double())),
      [this]() {publish();});
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    heartbeat_->stop();
    if (timer_) {
      timer_->cancel();
      timer_.reset();
    }
    LifecycleNode::on_deactivate(state);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    watched_.clear();
    pub_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & s) override
  {
    return on_cleanup(s);
  }

private:
  struct Watched
  {
    Watched(std::string t, double window, double stale)
    : topic(std::move(t)), metric(metric_name(topic)), window(window, stale)
    {
      msg.layout.dim.resize(1);
      msg.layout.dim[0].label = metric;
      msg.layout.dim[0].size = 1;
      msg.layout.dim[0].stride = 1;
      msg.data.resize(1);
    }
    std::string topic;
    std::string metric;
    RateWindow window;
    std_msgs::msg::Float64MultiArray msg;   // reused: no allocation per publish
    rclcpp::GenericSubscription::SharedPtr sub;
  };

  void publish()
  {
    if (!pub_ || !pub_->is_activated()) {
      return;
    }
    const double now = steady_now();
    for (auto & w : watched_) {
      w->msg.data[0] = w->window.rate_or_nan(now);
      pub_->publish(w->msg);
    }
  }

  std::vector<std::shared_ptr<Watched>> watched_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<Heartbeat> heartbeat_;
};

}  // namespace helix_sensing_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(helix_sensing_cpp::TopicRateMonitorNode)
