// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// See include/helix_sensing_cpp/anomaly_detector_node.hpp for contract.

#include "helix_sensing_cpp/anomaly_detector_node.hpp"

#include <chrono>
#include <cinttypes>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"

#include "helix_sensing_cpp/pyfmt.hpp"

namespace helix_sensing_cpp
{

AnomalyDetectorNode::AnomalyDetectorNode(const rclcpp::NodeOptions & options)
: LifecycleNode("helix_anomaly_detector", options)
{
  declare_parameter<double>("zscore_threshold", 3.0);
  declare_parameter<int>("consecutive_trigger", 3);
  declare_parameter<int>("window_size", 60);
  declare_parameter<double>("emit_cooldown_s", 1.0);
  declare_parameter<double>("min_anomaly_duration_s", 2.0);

  heartbeat_ = std::make_unique<Heartbeat>(this);
}

AnomalyDetectorNode::CallbackReturn
AnomalyDetectorNode::on_configure(const rclcpp_lifecycle::State &)
{
  AnomalyParams p;
  p.zscore_threshold = get_parameter("zscore_threshold").as_double();
  p.consecutive_trigger = get_parameter("consecutive_trigger").as_int();
  p.window_size = get_parameter("window_size").as_int();
  p.emit_cooldown_s = get_parameter("emit_cooldown_s").as_double();
  p.min_anomaly_duration_s = get_parameter("min_anomaly_duration_s").as_double();

  if (p.window_size <= 0) {
    RCLCPP_ERROR(
      get_logger(), "window_size must be > 0 (got %" PRId64 "); refusing to configure",
      p.window_size);
    return CallbackReturn::FAILURE;
  }
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    core_ = std::make_unique<AnomalyCore>(p);
  }

  fault_pub_ = create_publisher<helix_msgs::msg::FaultEvent>(
    "/helix/faults", rclcpp::QoS(10).reliable());

  diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::QoS(10).reliable(),
    std::bind(&AnomalyDetectorNode::on_diagnostics, this, std::placeholders::_1));

  metrics_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
    "/helix/metrics", rclcpp::QoS(100).reliable(),
    std::bind(&AnomalyDetectorNode::on_metric, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "AnomalyDetectorNode configured - zscore_threshold=%.3f consecutive_trigger=%" PRId64
    " window_size=%" PRId64 " emit_cooldown_s=%.3f min_anomaly_duration_s=%.3f",
    p.zscore_threshold, p.consecutive_trigger, p.window_size, p.emit_cooldown_s,
    p.min_anomaly_duration_s);
  return CallbackReturn::SUCCESS;
}

AnomalyDetectorNode::CallbackReturn
AnomalyDetectorNode::on_activate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_activate(state);
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    active_ = true;
  }
  heartbeat_->start();
  RCLCPP_INFO(
    get_logger(),
    "AnomalyDetectorNode activated (heartbeat publishing on %s at %.1f Hz).",
    kHeartbeatTopic, 1.0 / kHeartbeatPeriodSec);
  return CallbackReturn::SUCCESS;
}

AnomalyDetectorNode::CallbackReturn
AnomalyDetectorNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    active_ = false;
  }
  heartbeat_->stop();
  LifecycleNode::on_deactivate(state);
  RCLCPP_INFO(get_logger(), "AnomalyDetectorNode deactivated (heartbeat stopped).");
  return CallbackReturn::SUCCESS;
}

void AnomalyDetectorNode::stop_and_clear()
{
  heartbeat_->stop();
  fault_pub_.reset();
  diagnostics_sub_.reset();
  metrics_sub_.reset();
  std::lock_guard<std::mutex> lock(data_mutex_);
  active_ = false;
  core_.reset();
}

AnomalyDetectorNode::CallbackReturn
AnomalyDetectorNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  stop_and_clear();
  return CallbackReturn::SUCCESS;
}

AnomalyDetectorNode::CallbackReturn
AnomalyDetectorNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  // Shutdown is reachable from ACTIVE: the heartbeat must stop too, or a
  // finalized node would keep reporting itself alive.
  stop_and_clear();
  return CallbackReturn::SUCCESS;
}

AnomalyDetectorNode::CallbackReturn
AnomalyDetectorNode::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_ERROR(
    get_logger(), "lifecycle error from state '%s': stopping and unconfiguring",
    state.label().c_str());
  stop_and_clear();
  return CallbackReturn::SUCCESS;
}

void AnomalyDetectorNode::on_diagnostics(
  const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
  for (const auto & status : msg->status) {
    for (const auto & kv : status.values) {
      // float(kv.value) in the reference; a ValueError skips the entry.
      const auto v = parse_python_float(kv.value);
      if (v) {
        process_sample(status.name + "/" + kv.key, *v);
      }
    }
  }
}

void AnomalyDetectorNode::on_metric(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->layout.dim.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "Received Float64MultiArray with no dim labels, skipping");
    return;
  }
  const std::string & metric_name = msg->layout.dim[0].label;
  if (metric_name.empty()) {
    return;
  }
  for (double v : msg->data) {
    process_sample(metric_name, v);
  }
}

void AnomalyDetectorNode::process_sample(const std::string & metric_name, double value)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!core_ || !active_) {
    return;
  }
  const SampleResult r = core_->process(
    metric_name, value, steady_time_now(),
    system_time_now());
  const char * m = metric_name.c_str();
  const std::int64_t streak = r.consecutive;
  const bool violation = r.outcome == SampleOutcome::kViolation ||
    r.outcome == SampleOutcome::kStaleViolation ||
    r.outcome == SampleOutcome::kSuppressedDuration ||
    r.outcome == SampleOutcome::kSuppressedCooldown || r.outcome == SampleOutcome::kEmitted;
  if (violation && r.stale) {
    RCLCPP_WARN(
      get_logger(), "Metric '%s' stale (NaN), consecutive violation #%" PRId64, m,
      streak);
  } else if (violation) {
    RCLCPP_WARN(
      get_logger(), "Metric '%s' Z-score=%.2f (consecutive violation #%" PRId64 ")", m, r.zscore,
      streak);
  }
  switch (r.outcome) {
    case SampleOutcome::kFlat:
      RCLCPP_DEBUG(get_logger(), "Metric '%s' is flat (std=%.2e), skipping Z-score", m, r.std);
      break;
    case SampleOutcome::kNormal:
      if (r.streak_was_active) {
        RCLCPP_DEBUG(
          get_logger(), "Metric '%s' Z-score dropped to %.2f, resetting consecutive counter",
          m, r.zscore);
      }
      break;
    case SampleOutcome::kSuppressedDuration:
      RCLCPP_DEBUG(
        get_logger(), "Metric '%s'%s ANOMALY suppressed by min_anomaly_duration_s=%.3f "
        "(elapsed=%.3fs)", m, r.stale ? " stale" : "",
        core_->params().min_anomaly_duration_s, r.elapsed);
      break;
    case SampleOutcome::kSuppressedCooldown:
      RCLCPP_DEBUG(
        get_logger(), "Metric '%s'%s ANOMALY suppressed by emit_cooldown_s=%.3f "
        "(since last: %.3fs)", m, r.stale ? " stale" : "", core_->params().emit_cooldown_s,
        r.since_last_emit);
      break;
    case SampleOutcome::kEmitted:
      publish_fault(*r.fault);
      if (r.stale) {
        RCLCPP_INFO(
          get_logger(), "FaultEvent emitted: ANOMALY (stale) for '%s' consecutive=%" PRId64, m,
          streak);
      } else {
        RCLCPP_INFO(
          get_logger(),
          "FaultEvent emitted: ANOMALY for '%s' (zscore=%.2f, consecutive=%" PRId64 ")", m,
          r.zscore, streak);
      }
      break;
    default:
      break;  // insufficient history or a plain violation: nothing more to say
  }
}

void AnomalyDetectorNode::publish_fault(const FaultRecord & f)
{
  helix_msgs::msg::FaultEvent msg;
  msg.node_name = f.node_name;
  msg.fault_type = f.fault_type;
  msg.severity = f.severity;
  msg.detail = f.detail;
  msg.timestamp = f.timestamp;
  msg.context_keys = f.context_keys;
  msg.context_values = f.context_values;
  if (fault_pub_ && fault_pub_->is_activated()) {
    fault_pub_->publish(msg);
  }
  ++fault_count_;
}

double AnomalyDetectorNode::system_time_now()
{
  const rclcpp::Time t = system_clock_.now();
  return static_cast<double>(t.nanoseconds()) / 1e9;
}

double AnomalyDetectorNode::steady_time_now()
{
  const auto tp = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(tp.time_since_epoch()).count();
}

void AnomalyDetectorNode::process_sample_for_test(
  const std::string & metric_name, double value)
{
  process_sample(metric_name, value);
}

std::size_t AnomalyDetectorNode::fault_count_for_test() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return fault_count_;
}

}  // namespace helix_sensing_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(helix_sensing_cpp::AnomalyDetectorNode)
