// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// See include/helix_sensing_cpp/anomaly_detector_node.hpp for contract.

#include "helix_sensing_cpp/anomaly_detector_node.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

namespace helix_sensing_cpp
{

namespace
{

// Match Python's round(x, n) (banker's rounding in Python3) closely enough
// for human-readable FaultEvent context. We use the standard "round half
// away from zero" which matches Python for the precision we care about
// here (3+ dp on deterministic test inputs). The Python emitter also
// calls str(round(...)), so trailing zeros are stripped by Python's repr.
// We emit fixed-precision decimal that matches reasonable parity for logs.
std::string round_to_string(double value, int digits)
{
  if (std::isnan(value)) {
    return "nan";
  }
  if (std::isinf(value)) {
    return value > 0 ? "inf" : "-inf";
  }
  const double scale = std::pow(10.0, digits);
  const double rounded = std::round(value * scale) / scale;
  std::ostringstream os;
  os.precision(digits);
  os << std::fixed << rounded;
  // Match Python: strip trailing zeros and trailing '.'
  std::string s = os.str();
  if (s.find('.') != std::string::npos) {
    while (!s.empty() && s.back() == '0') {
      s.pop_back();
    }
    if (!s.empty() && s.back() == '.') {
      s.pop_back();
    }
  }
  if (s == "-0") {
    s = "0";
  }
  return s;
}

}  // namespace

AnomalyDetectorNode::AnomalyDetectorNode(const rclcpp::NodeOptions & options)
: LifecycleNode("helix_anomaly_detector", options)
{
  declare_parameter<double>("zscore_threshold", 3.0);
  declare_parameter<int>("consecutive_trigger", 3);
  declare_parameter<int>("window_size", 60);
  declare_parameter<double>("emit_cooldown_s", 1.0);
}

AnomalyDetectorNode::CallbackReturn
AnomalyDetectorNode::on_configure(const rclcpp_lifecycle::State &)
{
  zscore_threshold_ = get_parameter("zscore_threshold").as_double();
  consecutive_trigger_ = static_cast<int>(get_parameter("consecutive_trigger").as_int());
  window_size_ = static_cast<int>(get_parameter("window_size").as_int());
  emit_cooldown_s_ = get_parameter("emit_cooldown_s").as_double();

  if (window_size_ <= 0) {
    RCLCPP_ERROR(
      get_logger(),
      "window_size must be > 0 (got %d); refusing to configure",
      window_size_);
    return CallbackReturn::FAILURE;
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
    "AnomalyDetectorNode configured - zscore_threshold=%.3f consecutive_trigger=%d "
    "window_size=%d emit_cooldown_s=%.3f",
    zscore_threshold_, consecutive_trigger_, window_size_, emit_cooldown_s_);
  return CallbackReturn::SUCCESS;
}

AnomalyDetectorNode::CallbackReturn
AnomalyDetectorNode::on_activate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_activate(state);
  RCLCPP_INFO(get_logger(), "AnomalyDetectorNode activated.");
  return CallbackReturn::SUCCESS;
}

AnomalyDetectorNode::CallbackReturn
AnomalyDetectorNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_deactivate(state);
  RCLCPP_INFO(get_logger(), "AnomalyDetectorNode deactivated.");
  return CallbackReturn::SUCCESS;
}

AnomalyDetectorNode::CallbackReturn
AnomalyDetectorNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  fault_pub_.reset();
  diagnostics_sub_.reset();
  metrics_sub_.reset();
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    metrics_.clear();
  }
  return CallbackReturn::SUCCESS;
}

AnomalyDetectorNode::CallbackReturn
AnomalyDetectorNode::on_shutdown(const rclcpp_lifecycle::State & s)
{
  return on_cleanup(s);
}

void AnomalyDetectorNode::on_diagnostics(
  const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
  for (const auto & status : msg->status) {
    for (const auto & kv : status.values) {
      // Mirror Python's float(kv.value); skip on parse failure.
      try {
        std::size_t pos = 0;
        double v = std::stod(kv.value, &pos);
        // Require the entire string be consumed (stod is permissive).
        if (pos != kv.value.size()) {
          continue;
        }
        std::string metric_name = status.name + "/" + kv.key;
        process_sample(metric_name, v);
      } catch (const std::invalid_argument &) {
        continue;
      } catch (const std::out_of_range &) {
        continue;
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
      "Received Float64MultiArray with no dim labels - skipping");
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

  auto it = metrics_.find(metric_name);
  if (it == metrics_.end()) {
    it = metrics_.emplace(
      metric_name,
      MetricState(static_cast<std::size_t>(window_size_))).first;
  }
  MetricState & state = it->second;

  // Stale path: topic_rate_monitor emits NaN when the watched topic has no
  // samples in its rolling window. Z-score of NaN is NaN and fails every
  // threshold comparison, so without this branch a silent topic produces
  // zero anomalies. Treat NaN as a violation of the same shape as a
  // z-score breach (same counter, same consecutive_trigger gate, same
  // cooldown); do NOT push NaN to RollingStats — it would poison every
  // future mean/std for this metric.
  //
  // Mirrors helix_core.anomaly_detector._process_sample's isnan branch
  // (PR #7); keep the two in sync so the C++ port remains drop-in.
  if (std::isnan(value)) {
    state.consecutive += 1;
    const int consecutive = state.consecutive;
    RCLCPP_WARN(
      get_logger(),
      "Metric '%s' stale (NaN) - consecutive violation #%d",
      metric_name.c_str(), consecutive);
    if (consecutive >= consecutive_trigger_) {
      const double now = system_time_now();
      const bool cooldown_expired =
        emit_cooldown_s_ <= 0.0 ||
        state.last_emit_time == 0.0 ||
        (now - state.last_emit_time) >= emit_cooldown_s_;
      if (cooldown_expired) {
        state.last_emit_time = now;
        emit_stale_fault(metric_name, consecutive);
      } else {
        RCLCPP_DEBUG(
          get_logger(),
          "Metric '%s' stale ANOMALY suppressed by emit_cooldown_s=%.3f "
          "(since last: %.3fs)",
          metric_name.c_str(), emit_cooldown_s_, now - state.last_emit_time);
      }
    }
    return;
  }

  // Evaluate Z-score against the CURRENT window (before push), so a streak
  // of anomalies doesn't poison its own baseline — matches Python.
  const ZScoreResult r = state.stats.evaluate(value);

  if (r.status == ZScoreStatus::kOk) {
    if (r.zscore > zscore_threshold_) {
      state.consecutive += 1;
      const int consecutive = state.consecutive;

      RCLCPP_WARN(
        get_logger(),
        "Metric '%s' Z-score=%.2f (consecutive violation #%d)",
        metric_name.c_str(), r.zscore, consecutive);

      if (consecutive >= consecutive_trigger_) {
        // Cooldown gate: emit_cooldown_s_ <= 0 means legacy flood (emit
        // every sample). Otherwise only emit when now - last_emit >= cooldown.
        const double now = system_time_now();
        const bool cooldown_expired =
          emit_cooldown_s_ <= 0.0 ||
          state.last_emit_time == 0.0 ||
          (now - state.last_emit_time) >= emit_cooldown_s_;

        if (cooldown_expired) {
          state.last_emit_time = now;
          emit_anomaly_fault(
            metric_name, value, r.mean, r.std, r.zscore, consecutive, state);
        } else {
          RCLCPP_DEBUG(
            get_logger(),
            "Metric '%s' ANOMALY suppressed by emit_cooldown_s=%.3f "
            "(since last: %.3fs)",
            metric_name.c_str(), emit_cooldown_s_, now - state.last_emit_time);
        }
      }
    } else {
      if (state.consecutive > 0) {
        RCLCPP_DEBUG(
          get_logger(),
          "Metric '%s' Z-score dropped to %.2f - resetting consecutive counter",
          metric_name.c_str(), r.zscore);
      }
      state.consecutive = 0;
    }
  } else if (r.status == ZScoreStatus::kFlat) {
    RCLCPP_DEBUG(
      get_logger(),
      "Metric '%s' is flat (std=%.2e) - skipping Z-score",
      metric_name.c_str(), r.std);
  }
  // kInsufficient: silently wait for more samples (matches Python).

  // Always push after evaluation.
  state.stats.push(value);
}

void AnomalyDetectorNode::emit_anomaly_fault(
  const std::string & metric_name,
  double value, double mean, double std_, double zscore, int consecutive,
  MetricState & /*state*/)
{
  helix_msgs::msg::FaultEvent msg;
  msg.node_name = metric_name;
  msg.fault_type = "ANOMALY";
  msg.severity = 2;

  std::ostringstream detail;
  detail.precision(2);
  detail << std::fixed;
  detail << "Metric '" << metric_name << "' Z-score " << zscore
         << " exceeded threshold on " << consecutive_trigger_
         << " consecutive samples";
  msg.detail = detail.str();

  msg.timestamp = system_time_now();

  msg.context_keys = {
    "metric_name", "current_value", "window_mean",
    "window_std", "zscore", "consecutive_count",
  };
  msg.context_values = {
    metric_name,
    round_to_string(value, 4),   // 4 dp
    round_to_string(mean, 4),    // 4 dp
    round_to_string(std_, 6),    // 6 dp
    round_to_string(zscore, 2),  // 2 dp
    std::to_string(consecutive),
  };

  if (fault_pub_ && fault_pub_->is_activated()) {
    fault_pub_->publish(msg);
  }
  ++fault_count_;

  RCLCPP_INFO(
    get_logger(),
    "FaultEvent emitted: ANOMALY for '%s' (zscore=%.2f, consecutive=%d)",
    metric_name.c_str(), zscore, consecutive);
}

void AnomalyDetectorNode::emit_stale_fault(
  const std::string & metric_name, int consecutive)
{
  helix_msgs::msg::FaultEvent msg;
  msg.node_name = metric_name;
  msg.fault_type = "ANOMALY";
  msg.severity = 2;

  std::ostringstream detail;
  detail << "Metric '" << metric_name << "' stale - no samples in window on "
         << consecutive << " consecutive checks";
  msg.detail = detail.str();

  msg.timestamp = system_time_now();

  msg.context_keys = {"metric_name", "violation_type", "consecutive_count"};
  msg.context_values = {metric_name, "stale", std::to_string(consecutive)};

  if (fault_pub_ && fault_pub_->is_activated()) {
    fault_pub_->publish(msg);
  }
  ++fault_count_;

  RCLCPP_INFO(
    get_logger(),
    "FaultEvent emitted: ANOMALY (stale) for '%s' consecutive=%d",
    metric_name.c_str(), consecutive);
}

double AnomalyDetectorNode::system_time_now()
{
  const rclcpp::Time t = system_clock_.now();
  return static_cast<double>(t.nanoseconds()) / 1e9;
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
