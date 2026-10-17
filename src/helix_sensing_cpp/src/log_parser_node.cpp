// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// LogParserNode: native port of helix_core.log_parser.LogParser. Same node
// name (helix_log_parser), parameters (rules_file_path, dedup_window_sec),
// topics (/rosout in, /helix/faults out), FaultEvent fields and heartbeat.
// Matching lives in LogMatcher (log_matcher.hpp).

#include <chrono>
#include <exception>
#include <memory>
#include <string>

#include "helix_msgs/msg/fault_event.hpp"
#include "helix_sensing_cpp/heartbeat.hpp"
#include "helix_sensing_cpp/log_matcher.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace helix_sensing_cpp
{

class LogParserNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit LogParserNode(const rclcpp::NodeOptions & options)
  : LifecycleNode("helix_log_parser", options)
  {
    declare_parameter("rules_file_path", std::string());
    declare_parameter("dedup_window_sec", 5.0);
    heartbeat_ = std::make_unique<Heartbeat>(this);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    const double dedup = get_parameter("dedup_window_sec").as_double();
    const std::string path = get_parameter("rules_file_path").as_string();
    matcher_ = std::make_unique<LogMatcher>(dedup);
    if (path.empty()) {
      RCLCPP_WARN(get_logger(), "rules_file_path parameter is empty, LogParser will have no rules");
    } else {
      try {
        for (const auto & w : matcher_->load(path)) {
          RCLCPP_WARN(get_logger(), "%s", w.c_str());
        }
        RCLCPP_INFO(get_logger(), "Loaded %zu rules from '%s'", matcher_->size(), path.c_str());
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Failed to load rules from '%s': %s", path.c_str(), e.what());
        return CallbackReturn::FAILURE;
      }
    }
    fault_pub_ = create_publisher<helix_msgs::msg::FaultEvent>("/helix/faults", 10);
    rosout_sub_ = create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", 100, [this](rcl_interfaces::msg::Log::ConstSharedPtr m) {on_log(*m);});
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    LifecycleNode::on_activate(state);
    heartbeat_->start();
    RCLCPP_INFO(
      get_logger(), "LogParser activated with %zu rules, dedup_window=%.1fs", matcher_->size(),
      get_parameter("dedup_window_sec").as_double());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    heartbeat_->stop();
    LifecycleNode::on_deactivate(state);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    rosout_sub_.reset();
    fault_pub_.reset();
    if (matcher_) {
      matcher_->clear();
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & s) override {return on_cleanup(s);}

private:
  void on_log(const rcl_interfaces::msg::Log & log)
  {
    // The reference processes /rosout from configure on; publishing still
    // needs an active publisher.
    if (!fault_pub_ || !fault_pub_->is_activated() || log.level < kMinLogSeverity) {
      return;
    }
    const double now = std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    for (auto & f : matcher_->process(log.level, log.name, log.msg, log.file, now)) {
      helix_msgs::msg::FaultEvent m;
      m.node_name = f.node_name;
      m.fault_type = f.fault_type;
      m.severity = static_cast<std::uint8_t>(f.severity);
      m.detail = f.detail;
      m.timestamp = f.timestamp;
      m.context_keys = f.context_keys;
      m.context_values = f.context_values;
      fault_pub_->publish(m);
      RCLCPP_INFO(
        get_logger(), "FaultEvent emitted: %s for '%s' (rule='%s')", f.fault_type.c_str(),
        log.name.c_str(), f.context_values[0].c_str());
    }
  }

  std::unique_ptr<LogMatcher> matcher_;
  rclcpp_lifecycle::LifecyclePublisher<helix_msgs::msg::FaultEvent>::SharedPtr fault_pub_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub_;
  std::unique_ptr<Heartbeat> heartbeat_;
};

}  // namespace helix_sensing_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(helix_sensing_cpp::LogParserNode)
