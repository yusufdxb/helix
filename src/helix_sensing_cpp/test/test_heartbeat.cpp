// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// Regression tests for the AnomalyDetectorNode heartbeat emitter.
//
// The C++ port shipped without a heartbeat while the Python node had one.
// HeartbeatMonitor only reports CRASH for a node it has already seen alive
// on /helix/heartbeat, so on the C++ path node death was undetectable.
// These tests fail if the emitter is removed or its contract drifts away
// from helix_core/heartbeat.py.

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "std_msgs/msg/string.hpp"

#include "helix_sensing_cpp/anomaly_detector_node.hpp"
#include "helix_sensing_cpp/heartbeat.hpp"

using helix_sensing_cpp::AnomalyDetectorNode;
using helix_sensing_cpp::kHeartbeatPeriodSec;
using helix_sensing_cpp::kHeartbeatTopic;
using namespace std::chrono_literals;

namespace
{

constexpr const char * kNodeUnderTest = "helix_anomaly_detector";

class HeartbeatFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    listener_ = rclcpp::Node::make_shared("heartbeat_test_listener");
    sub_ = listener_->create_subscription<std_msgs::msg::String>(
      kHeartbeatTopic, rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        // /helix/heartbeat is a shared bus: every HELIX node publishes its
        // own name on it, and colcon runs test binaries concurrently. Count
        // only the node under test, or a sibling suite's beats leak in and
        // the silence assertions below become meaningless.
        if (msg->data != kNodeUnderTest) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        beats_.push_back(msg->data);
      });
  }

  std::size_t beat_count()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return beats_.size();
  }

  std::vector<std::string> beats_copy()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return beats_;
  }

  void clear_beats()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    beats_.clear();
  }

  // Spin node + listener until predicate is true or the deadline passes.
  // Returns the final value of the predicate.
  template<typename Pred>
  bool spin_until(
    const std::shared_ptr<AnomalyDetectorNode> & node,
    Pred predicate,
    std::chrono::milliseconds timeout)
  {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node->get_node_base_interface());
    exec.add_node(listener_);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (predicate()) {
        break;
      }
      exec.spin_once(20ms);
    }
    const bool result = predicate();
    exec.remove_node(listener_);
    exec.remove_node(node->get_node_base_interface());
    return result;
  }

  // Spin for a fixed wall duration regardless of predicate.
  void spin_for(
    const std::shared_ptr<AnomalyDetectorNode> & node,
    std::chrono::milliseconds duration)
  {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node->get_node_base_interface());
    exec.add_node(listener_);
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      exec.spin_once(20ms);
    }
    exec.remove_node(listener_);
    exec.remove_node(node->get_node_base_interface());
  }

  std::shared_ptr<rclcpp::Node> listener_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::mutex mutex_;
  std::vector<std::string> beats_;
};

std::shared_ptr<AnomalyDetectorNode> make_node()
{
  rclcpp::NodeOptions opts;
  return std::make_shared<AnomalyDetectorNode>(opts);
}

}  // namespace

// The contract constants must match helix_core/heartbeat.py exactly.
// If either side is retuned, this fails and forces the other to follow.
TEST(HeartbeatContract, MatchesPythonEmitterConstants)
{
  EXPECT_STREQ(kHeartbeatTopic, "/helix/heartbeat");
  EXPECT_DOUBLE_EQ(kHeartbeatPeriodSec, 0.1);
}

// The core regression: an ACTIVE node must publish on /helix/heartbeat.
// Without the emitter this fails, and node death stays undetectable.
TEST_F(HeartbeatFixture, ActiveNodePublishesHeartbeat)
{
  auto node = make_node();
  node->configure();
  node->activate();

  ASSERT_TRUE(
    spin_until(node, [this]() {return beat_count() >= 3;}, 3000ms))
    << "AnomalyDetectorNode published no heartbeat while active. "
    << "HeartbeatMonitor cannot report CRASH for a node it never saw alive.";
}

// The payload is the node name: that is the key HeartbeatMonitor registers.
// A wrong or empty payload registers the wrong node and CRASH still misses.
TEST_F(HeartbeatFixture, HeartbeatPayloadIsNodeName)
{
  auto node = make_node();
  node->configure();
  node->activate();

  ASSERT_TRUE(spin_until(node, [this]() {return beat_count() >= 1;}, 3000ms));
  for (const auto & data : beats_copy()) {
    EXPECT_EQ(data, kNodeUnderTest);
  }
}

// Roughly 10 Hz: the monitor's default timeout is 0.3 s with a 3-miss
// threshold, so a materially slower emitter would be reported as crashed.
// Bounds are loose because this runs on a shared CI executor.
TEST_F(HeartbeatFixture, HeartbeatRateIsAboutTenHertz)
{
  auto node = make_node();
  node->configure();
  node->activate();

  // Let discovery settle before timing anything.
  ASSERT_TRUE(spin_until(node, [this]() {return beat_count() >= 1;}, 3000ms));
  clear_beats();
  spin_for(node, 1000ms);

  const std::size_t n = beat_count();
  EXPECT_GE(n, 5u) << "heartbeat far slower than 10 Hz: " << n << " beats in 1 s";
  EXPECT_LE(n, 20u) << "heartbeat far faster than 10 Hz: " << n << " beats in 1 s";
}

// An INACTIVE node must be silent, otherwise a configured-but-not-activated
// node registers itself with the monitor and is then reported as crashed
// the moment the operator is merely slow to activate it.
TEST_F(HeartbeatFixture, InactiveNodeIsSilent)
{
  auto node = make_node();
  node->configure();  // configured, deliberately NOT activated

  spin_for(node, 500ms);
  EXPECT_EQ(beat_count(), 0u)
    << "node published a heartbeat before activation";
}

// Deactivating must stop the beats: a deactivated node is not running, and
// a stale heartbeat would mask a genuine failure downstream.
TEST_F(HeartbeatFixture, DeactivateStopsHeartbeat)
{
  auto node = make_node();
  node->configure();
  node->activate();
  ASSERT_TRUE(spin_until(node, [this]() {return beat_count() >= 2;}, 3000ms));

  node->deactivate();
  clear_beats();
  spin_for(node, 500ms);

  EXPECT_EQ(beat_count(), 0u)
    << "node kept publishing heartbeats after deactivation";
}

// Python Heartbeat.start() creates a fresh timer after stop() destroys the
// previous one. The C++ lifecycle path must support the same reactivation.
TEST_F(HeartbeatFixture, ReactivationRestartsHeartbeat)
{
  auto node = make_node();
  node->configure();
  node->activate();
  ASSERT_TRUE(spin_until(node, [this]() {return beat_count() >= 2;}, 3000ms));

  node->deactivate();
  // Drain any beat already queued in DDS before measuring the second active
  // period, then discard the first period's observations.
  spin_for(node, 300ms);
  clear_beats();

  node->activate();
  EXPECT_TRUE(spin_until(node, [this]() {return beat_count() >= 2;}, 3000ms))
    << "node did not resume heartbeats after reactivation";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
