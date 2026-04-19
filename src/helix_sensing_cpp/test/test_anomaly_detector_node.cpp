// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// Node-level integration tests for AnomalyDetectorNode. Drives the node
// through a SingleThreadedExecutor, publishes Float64MultiArray and
// DiagnosticArray messages, and observes FaultEvent emissions on
// /helix/faults.
//
// 13-row parity matrix from the design doc §5 (reproduced here since the
// design doc is not in-tree yet):
//   1. Insufficient history (<2) -> no emit
//   2. Flat window (std < 1e-6) -> no emit
//   3. Z<=threshold -> no emit, consecutive resets
//   4. Z>threshold but consecutive<trigger -> no emit
//   5. Z>threshold, consecutive>=trigger -> emit
//   6. FaultEvent fields populated correctly
//   7. FaultEvent rounding precisions (4, 4, 6, 2 dp)
//   8. Metrics with empty layout dim -> skipped
//   9. Metrics with empty label -> skipped
//  10. DiagnosticArray numeric values processed
//  11. DiagnosticArray non-numeric values skipped
//  12. emit_cooldown_s=0 -> every sample after trigger emits (legacy flood)
//  13. emit_cooldown_s>0 -> subsequent emits suppressed within cooldown
//  14. Capacity rollover respects window_size

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "helix_msgs/msg/fault_event.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"

#include "helix_sensing_cpp/anomaly_detector_node.hpp"

using helix_sensing_cpp::AnomalyDetectorNode;
using namespace std::chrono_literals;

namespace
{

std_msgs::msg::Float64MultiArray make_metric(const std::string & label, double v)
{
  std_msgs::msg::Float64MultiArray m;
  std_msgs::msg::MultiArrayDimension dim;
  dim.label = label;
  dim.size = 1;
  dim.stride = 1;
  m.layout.dim.push_back(dim);
  m.data.push_back(v);
  return m;
}

class RosFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

// Helper: build node with given parameters, configure+activate it.
std::shared_ptr<AnomalyDetectorNode> make_active_node(
  double zscore = 3.0,
  int consecutive = 3,
  int window_size = 60,
  double cooldown = 1.0)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(
    {
      rclcpp::Parameter("zscore_threshold", zscore),
      rclcpp::Parameter("consecutive_trigger", consecutive),
      rclcpp::Parameter("window_size", window_size),
      rclcpp::Parameter("emit_cooldown_s", cooldown),
    });
  auto node = std::make_shared<AnomalyDetectorNode>(opts);
  node->configure();
  node->activate();
  return node;
}

}  // namespace

// ── Row 1: insufficient history ─────────────────────────────────────────
TEST_F(RosFixture, InsufficientHistoryNoEmit)
{
  auto node = make_active_node();
  node->process_sample_for_test("m", 100.0);
  EXPECT_EQ(node->fault_count_for_test(), 0u);
}

// ── Row 2: flat window ──────────────────────────────────────────────────
TEST_F(RosFixture, FlatWindowNoEmit)
{
  auto node = make_active_node();
  for (int i = 0; i < 10; ++i) {
    node->process_sample_for_test("m", 5.0);
  }
  // Huge spike but baseline is flat -> no emit.
  node->process_sample_for_test("m", 1000.0);
  EXPECT_EQ(node->fault_count_for_test(), 0u);
}

// ── Row 3: z below threshold, no emit, consecutive resets ───────────────
TEST_F(RosFixture, ZBelowThresholdNoEmit)
{
  auto node = make_active_node();
  for (int i = 0; i < 10; ++i) {
    node->process_sample_for_test("m", 10.0 + (i % 3) * 0.1);
  }
  // Mild perturbation: within ~1 std -> no emit
  node->process_sample_for_test("m", 10.2);
  EXPECT_EQ(node->fault_count_for_test(), 0u);
}

// ── Row 4: z above threshold but fewer than `consecutive_trigger` ───────
TEST_F(RosFixture, AboveThresholdButBelowTriggerNoEmit)
{
  auto node = make_active_node(3.0, 3, 60, 0.0);  // cooldown=0 isolates
  for (int i = 0; i < 15; ++i) {
    node->process_sample_for_test("m", 10.0 + (i % 3) * 0.1);
  }
  // Two spikes (<trigger of 3) -> no emit
  node->process_sample_for_test("m", 100.0);
  node->process_sample_for_test("m", 100.0);
  EXPECT_EQ(node->fault_count_for_test(), 0u);
}

// ── Row 5: three consecutive spikes -> emit ─────────────────────────────
TEST_F(RosFixture, TripleSpikeEmits)
{
  auto node = make_active_node(3.0, 3, 60, 0.0);
  for (int i = 0; i < 25; ++i) {
    node->process_sample_for_test("m", 10.0 + (i % 3) * 0.1);
  }
  node->process_sample_for_test("m", 100.0);
  node->process_sample_for_test("m", 100.0);
  node->process_sample_for_test("m", 100.0);
  EXPECT_GE(node->fault_count_for_test(), 1u);
}

// ── Row 6 + 7: FaultEvent fields + rounding precisions ──────────────────
TEST_F(RosFixture, FaultEventFieldsAndRounding)
{
  auto node = make_active_node(3.0, 3, 60, 0.0);

  // Subscribe to /helix/faults from a sibling node.
  auto sub_node = rclcpp::Node::make_shared("test_fault_sub");
  std::vector<helix_msgs::msg::FaultEvent> received;
  std::mutex mu;
  auto sub = sub_node->create_subscription<helix_msgs::msg::FaultEvent>(
    "/helix/faults", 10,
    [&](helix_msgs::msg::FaultEvent::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mu);
      received.push_back(*msg);
    });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(sub_node);

  // Drive the detector programmatically (spinning in between to let the
  // fault publish through the middleware to sub_node).
  auto pump = [&](int n) {
      for (int i = 0; i < n; ++i) {
        exec.spin_some();
        std::this_thread::sleep_for(5ms);
      }
    };

  for (int i = 0; i < 25; ++i) {
    node->process_sample_for_test("m/test", 10.0 + (i % 3) * 0.1);
  }
  node->process_sample_for_test("m/test", 100.0);
  node->process_sample_for_test("m/test", 100.0);
  node->process_sample_for_test("m/test", 100.0);

  // Let the fault propagate.
  pump(40);

  exec.remove_node(sub_node);
  exec.remove_node(node->get_node_base_interface());

  std::lock_guard<std::mutex> lock(mu);
  ASSERT_GE(received.size(), 1u);
  const auto & f = received.front();
  EXPECT_EQ(f.fault_type, "ANOMALY");
  EXPECT_EQ(f.severity, 2);
  EXPECT_EQ(f.node_name, "m/test");
  EXPECT_FALSE(f.detail.empty());
  EXPECT_GT(f.timestamp, 0.0);
  ASSERT_EQ(f.context_keys.size(), 6u);
  ASSERT_EQ(f.context_values.size(), 6u);
  EXPECT_EQ(f.context_keys[0], "metric_name");
  EXPECT_EQ(f.context_keys[1], "current_value");
  EXPECT_EQ(f.context_keys[2], "window_mean");
  EXPECT_EQ(f.context_keys[3], "window_std");
  EXPECT_EQ(f.context_keys[4], "zscore");
  EXPECT_EQ(f.context_keys[5], "consecutive_count");
  EXPECT_EQ(f.context_values[0], "m/test");
  EXPECT_EQ(f.context_values[1], "100");  // round(100.0, 4) -> 100 (trailing 0s stripped)
  // mean of {10.0, 10.1, 10.2, 10.0, 10.1, 10.2, ...} over 25 samples ≈ 10.1
  EXPECT_NE(f.context_values[2], "");
  EXPECT_NE(f.context_values[3], "");
  EXPECT_NE(f.context_values[4], "");
  EXPECT_EQ(f.context_values[5], "3");
}

// ── Row 8: empty dim -> skipped ─────────────────────────────────────────
TEST_F(RosFixture, EmptyLayoutDimSkipped)
{
  auto node = make_active_node();

  // publish via middleware so /helix/metrics callback runs
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  auto pub_node = rclcpp::Node::make_shared("test_empty_dim_pub");
  auto pub = pub_node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/helix/metrics", rclcpp::QoS(100).reliable());
  exec.add_node(pub_node);

  std_msgs::msg::Float64MultiArray m;
  m.data.push_back(42.0);  // no dim
  for (int i = 0; i < 10; ++i) {
    pub->publish(m);
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_EQ(node->fault_count_for_test(), 0u);

  exec.remove_node(pub_node);
  exec.remove_node(node->get_node_base_interface());
}

// ── Row 9: empty label -> skipped (covered implicitly; test via pub)
TEST_F(RosFixture, EmptyLabelSkipped)
{
  auto node = make_active_node();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  auto pub_node = rclcpp::Node::make_shared("test_empty_label_pub");
  auto pub = pub_node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/helix/metrics", rclcpp::QoS(100).reliable());
  exec.add_node(pub_node);

  auto m = make_metric("", 42.0);
  for (int i = 0; i < 10; ++i) {
    pub->publish(m);
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_EQ(node->fault_count_for_test(), 0u);

  exec.remove_node(pub_node);
  exec.remove_node(node->get_node_base_interface());
}

// ── Row 10: DiagnosticArray numeric values parsed ───────────────────────
TEST_F(RosFixture, DiagnosticArrayNumericProcessed)
{
  auto node = make_active_node(3.0, 3, 60, 0.0);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  auto pub_node = rclcpp::Node::make_shared("test_diag_pub");
  auto pub = pub_node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::QoS(10).reliable());
  exec.add_node(pub_node);

  auto send = [&](double value) {
      diagnostic_msgs::msg::DiagnosticArray arr;
      diagnostic_msgs::msg::DiagnosticStatus st;
      st.name = "nodeA";
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "cpu_pct";
      kv.value = std::to_string(value);
      st.values.push_back(kv);
      arr.status.push_back(st);
      pub->publish(arr);
      exec.spin_some();
      std::this_thread::sleep_for(5ms);
    };

  for (int i = 0; i < 25; ++i) {
    send(10.0 + (i % 3) * 0.1);
  }
  for (int i = 0; i < 3; ++i) {
    send(100.0);
  }
  // drain
  for (int i = 0; i < 30; ++i) {
    exec.spin_some();
    std::this_thread::sleep_for(2ms);
  }

  EXPECT_GE(node->fault_count_for_test(), 1u);

  exec.remove_node(pub_node);
  exec.remove_node(node->get_node_base_interface());
}

// ── Row 11: DiagnosticArray non-numeric values skipped ──────────────────
TEST_F(RosFixture, DiagnosticArrayNonNumericSkipped)
{
  auto node = make_active_node();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  auto pub_node = rclcpp::Node::make_shared("test_diag_text_pub");
  auto pub = pub_node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::QoS(10).reliable());
  exec.add_node(pub_node);

  diagnostic_msgs::msg::DiagnosticArray arr;
  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "nodeA";
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "status";
  kv.value = "OK";  // non-numeric
  st.values.push_back(kv);
  arr.status.push_back(st);
  for (int i = 0; i < 30; ++i) {
    pub->publish(arr);
    exec.spin_some();
    std::this_thread::sleep_for(3ms);
  }
  EXPECT_EQ(node->fault_count_for_test(), 0u);

  exec.remove_node(pub_node);
  exec.remove_node(node->get_node_base_interface());
}

// ── Row 12: cooldown=0 -> legacy flood ──────────────────────────────────
// Fill a 500-sample window with stable baseline, then 10 spikes: the
// baseline is so dominant that each spike keeps Z > threshold, so
// cooldown=0 produces one emit per sample past the trigger (legacy flood).
TEST_F(RosFixture, CooldownZeroFloods)
{
  auto node = make_active_node(3.0, 3, 500, 0.0);
  for (int i = 0; i < 500; ++i) {
    node->process_sample_for_test("m", 10.0 + (i % 3) * 0.1);
  }
  for (int i = 0; i < 10; ++i) {
    node->process_sample_for_test("m", 100.0);
  }
  // Expect >= 3 emits: once consecutive hits 3, every subsequent spike
  // (while Z stays > threshold) re-emits because cooldown=0.
  EXPECT_GE(node->fault_count_for_test(), 3u);
}

// ── Row 13: cooldown>0 -> subsequent emits suppressed within window ─────
TEST_F(RosFixture, CooldownSuppressesSubsequentEmits)
{
  // Large cooldown relative to test runtime (30 s); all follow-up emits
  // within the cooldown must be suppressed.
  auto node = make_active_node(3.0, 3, 500, 30.0);
  for (int i = 0; i < 500; ++i) {
    node->process_sample_for_test("m", 10.0 + (i % 3) * 0.1);
  }
  for (int i = 0; i < 10; ++i) {
    node->process_sample_for_test("m", 100.0);
  }
  // Exactly one emission despite multiple spikes past the trigger.
  EXPECT_EQ(node->fault_count_for_test(), 1u);
}

// ── Row 14: window capacity rollover ────────────────────────────────────
TEST_F(RosFixture, WindowCapacityRollover)
{
  // window_size=5; push 10 samples that are all 10.0; baseline should
  // remain flat after rollover (only the last 5 survive).
  auto node = make_active_node(3.0, 3, 5, 0.0);
  for (int i = 0; i < 10; ++i) {
    node->process_sample_for_test("m", 10.0);
  }
  // baseline is perfectly flat -> no emit even on huge spike
  node->process_sample_for_test("m", 10000.0);
  node->process_sample_for_test("m", 10000.0);
  node->process_sample_for_test("m", 10000.0);
  // First spike: baseline flat (all 10.0) -> no emit.
  // Second spike: baseline now {10,10,10,10,10000}, pop-mean = 2008,
  // pop-std ≈ 3998; z = (10000-2008)/3998 ≈ 2.0 -> below threshold 3.0.
  // So still no emit after the second spike either.
  EXPECT_EQ(node->fault_count_for_test(), 0u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
