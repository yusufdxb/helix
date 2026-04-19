// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// AnomalyDetectorNode — rclcpp_lifecycle C++ port of the HELIX Phase 1
// Python anomaly detector (helix_core.anomaly_detector). Drop-in
// behavioral replacement:
//   * node name:  helix_anomaly_detector
//   * subs:       /diagnostics (diagnostic_msgs/DiagnosticArray, depth 10)
//                 /helix/metrics (std_msgs/Float64MultiArray, depth 100)
//   * pub:        /helix/faults (helix_msgs/FaultEvent, depth 10)
//   * params:     zscore_threshold (double, default 3.0)
//                 consecutive_trigger (int, default 3)
//                 window_size (int, default 60)
//                 emit_cooldown_s (double, default 1.0; 0.0 = legacy flood)
//
// QoS: reliable depth 10 for all I/O (matches the Python node's default
// QoS). Q2 from the design doc resolved to "reliable, depth 10".
#ifndef HELIX_SENSING_CPP__ANOMALY_DETECTOR_NODE_HPP_
#define HELIX_SENSING_CPP__ANOMALY_DETECTOR_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "helix_msgs/msg/fault_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "helix_sensing_cpp/rolling_stats.hpp"

namespace helix_sensing_cpp
{

class AnomalyDetectorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit AnomalyDetectorNode(const rclcpp::NodeOptions & options);

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & s) override;

  // Exposed for tests so they don't need a live /diagnostics publisher.
  void process_sample_for_test(const std::string & metric_name, double value);
  std::size_t fault_count_for_test() const;

private:
  struct MetricState
  {
    RollingStats stats;
    int consecutive = 0;
    double last_emit_time = 0.0;  // seconds since epoch, 0.0 == never
    explicit MetricState(std::size_t win)
    : stats(win) {}
  };

  void on_diagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
  void on_metric(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  void process_sample(const std::string & metric_name, double value);

  void emit_anomaly_fault(
    const std::string & metric_name,
    double value, double mean, double std_, double zscore, int consecutive,
    MetricState & state);

  // Wall-clock in seconds since epoch — RCL_SYSTEM_TIME matches
  // Python time.time() for the FaultEvent.timestamp field.
  double system_time_now();

  // Parameters (latched at configure).
  double zscore_threshold_ = 3.0;
  int consecutive_trigger_ = 3;
  int window_size_ = 60;
  double emit_cooldown_s_ = 1.0;

  mutable std::mutex data_mutex_;
  std::unordered_map<std::string, MetricState> metrics_;

  // Test-visible monotonic emit count.
  std::size_t fault_count_ = 0;

  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr metrics_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<helix_msgs::msg::FaultEvent>>
  fault_pub_;

  rclcpp::Clock system_clock_{RCL_SYSTEM_TIME};
};

}  // namespace helix_sensing_cpp

#endif  // HELIX_SENSING_CPP__ANOMALY_DETECTOR_NODE_HPP_
