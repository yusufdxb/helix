// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// AnomalyDetectorNode: rclcpp_lifecycle C++ port of the HELIX Phase 1
// Python anomaly detector (helix_core.anomaly_detector). Drop-in
// behavioral replacement:
//   * node name:  helix_anomaly_detector
//   * subs:       /diagnostics (diagnostic_msgs/DiagnosticArray, depth 10)
//                 /helix/metrics (std_msgs/Float64MultiArray, depth 100)
//   * pub:        /helix/faults (helix_msgs/FaultEvent, depth 10)
//   * pub:        /helix/heartbeat (std_msgs/String, depth 10, 10 Hz
//                 while active). HeartbeatMonitor cannot report CRASH
//                 for a node that never registers itself; see
//                 heartbeat.hpp for the contract and its limits.
//   * params:     zscore_threshold (double, default 3.0)
//                 consecutive_trigger (int, default 3)
//                 window_size (int, default 60)
//                 emit_cooldown_s (double, default 1.0; 0.0 = legacy flood)
//                 min_anomaly_duration_s (double, default 2.0; 0.0 = no gate)
//
// QoS: reliable depth 10 for all I/O (matches the Python node's default
// QoS). Q2 from the design doc resolved to "reliable, depth 10".
//
// Detection logic lives in AnomalyCore (anomaly_core.hpp, no ROS), which is
// compared with the Python reference sample by sample. FaultEvent strings
// (detail, rounded context values) match the reference byte for byte, and
// DiagnosticArray values are parsed with Python float() semantics
// (pyfmt.hpp).
//
// Lifecycle: inputs are processed only while ACTIVE. The Python node keeps
// processing (and publishing) while inactive; here an inactive node neither
// publishes nor advances any streak or cooldown, so activation never starts
// with a cooldown consumed by a fault nobody received. Shutdown and error
// transitions stop the heartbeat as well as clearing state.
#ifndef HELIX_SENSING_CPP__ANOMALY_DETECTOR_NODE_HPP_
#define HELIX_SENSING_CPP__ANOMALY_DETECTOR_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "helix_msgs/msg/fault_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "helix_sensing_cpp/anomaly_core.hpp"
#include "helix_sensing_cpp/heartbeat.hpp"

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
  CallbackReturn on_error(const rclcpp_lifecycle::State & s) override;

  // Exposed for tests so they don't need a live /diagnostics publisher.
  void process_sample_for_test(const std::string & metric_name, double value);
  std::size_t fault_count_for_test() const;

private:
  void on_diagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
  void on_metric(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // Runs one sample through the core, logs like the reference and publishes
  // an emitted fault. Takes data_mutex_.
  void process_sample(const std::string & metric_name, double value);
  void publish_fault(const FaultRecord & fault);
  void stop_and_clear();

  // Wall-clock in seconds since epoch. RCL_SYSTEM_TIME matches
  // Python time.time() for the FaultEvent.timestamp field.
  double system_time_now();

  // Steady (monotonic) clock in seconds, used for duration gating so
  // that system-clock adjustments don't affect anomaly timing.
  double steady_time_now();

  mutable std::mutex data_mutex_;
  std::unique_ptr<AnomalyCore> core_;  // built at configure from the latched parameters
  bool active_ = false;

  // Test-visible monotonic emit count.
  std::size_t fault_count_ = 0;

  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr metrics_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<helix_msgs::msg::FaultEvent>>
  fault_pub_;

  // Constructed in the constructor (mirrors the Python node, which builds
  // its Heartbeat in __init__), started/stopped by the activate transitions.
  std::unique_ptr<Heartbeat> heartbeat_;

  rclcpp::Clock system_clock_{RCL_SYSTEM_TIME};
};

}  // namespace helix_sensing_cpp

#endif  // HELIX_SENSING_CPP__ANOMALY_DETECTOR_NODE_HPP_
