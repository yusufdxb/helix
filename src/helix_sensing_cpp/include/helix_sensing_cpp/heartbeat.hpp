// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// Heartbeat: C++ mirror of helix_core.heartbeat.Heartbeat (the publish side
// of helix_core.heartbeat_monitor.HeartbeatMonitor).
//
// HeartbeatMonitor builds its registry from whoever publishes on
// /helix/heartbeat, and only reports a CRASH for a node it has already seen
// alive. A node with no emitter is therefore never registered and its death
// is never detectable. The Python AnomalyDetector has carried an emitter
// since the CRASH-detection fix; the C++ port shipped without one, so on the
// C++ path node death was undetectable. This restores the contract.
//
// The contract is copied from helix_core/heartbeat.py, not invented:
//   topic:   /helix/heartbeat
//   type:    std_msgs/msg/String
//   payload: the owning node's name (node->get_name())
//   QoS:     depth 10 (rclpy's default: KEEP_LAST 10, RELIABLE, VOLATILE)
//   period:  0.1 s (DEFAULT_PERIOD_SEC), against the monitor's
//            heartbeat_timeout_sec=0.3 and miss_threshold=3
//
// What this does NOT do, stated plainly: a heartbeat says "this process is
// still scheduling timers", not "this node is healthy". Because the timer
// runs on the same executor as the node's callbacks, a wedged executor stops
// the heartbeat too, so the monitor reports CRASH for a live-but-wedged
// process. That is indistinguishable from real death at this layer, and it
// is the same limitation the Python emitter has. DDS liveliness
// (assert_liveliness) is NOT used here: in this codebase it would ride the
// same executor timer, so it would add an second name for the same
// guarantee rather than a new one.
#ifndef HELIX_SENSING_CPP__HEARTBEAT_HPP_
#define HELIX_SENSING_CPP__HEARTBEAT_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"

namespace helix_sensing_cpp
{

// 10 Hz against the monitor's 0.3 s timeout and 3-miss threshold leaves room
// for roughly two dropped beats before a node is called crashed. Matches
// helix_core.heartbeat.DEFAULT_PERIOD_SEC.
constexpr double kHeartbeatPeriodSec = 0.1;

// Matches helix_core.heartbeat.HEARTBEAT_TOPIC.
constexpr const char * kHeartbeatTopic = "/helix/heartbeat";

/// Publishes the owning node's name on /helix/heartbeat while active.
class Heartbeat
{
public:
  explicit Heartbeat(
    rclcpp_lifecycle::LifecycleNode * node,
    double period_sec = kHeartbeatPeriodSec)
  : node_(node), period_sec_(period_sec)
  {
    msg_.data = node_->get_name();
    // Depth 10, reliable, volatile: rclpy's create_publisher(..., 10) default.
    pub_ = node_->create_publisher<std_msgs::msg::String>(
      kHeartbeatTopic, rclcpp::QoS(10).reliable());
  }

  /// Activate the publisher and start beating. Call from on_activate.
  ///
  /// LifecycleNode::on_activate(state) activates every managed publisher, but
  /// this is called explicitly so the emitter is correct even if a future
  /// subclass forgets to chain to the base implementation.
  void start()
  {
    pub_->on_activate();
    if (!timer_) {
      // Wall timer, deliberately: a heartbeat measures whether the process is
      // still scheduling, which is a wall-clock property. A ROS-time timer
      // would stop beating whenever /clock stops under use_sim_time, and the
      // monitor would report CRASH for a paused simulation.
      timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(period_sec_),
        [this]() {pub_->publish(msg_);});
    }
  }

  /// Stop beating and deactivate the publisher. Call from on_deactivate.
  void stop()
  {
    if (timer_) {
      timer_->cancel();
      timer_.reset();
    }
    pub_->on_deactivate();
  }

private:
  rclcpp_lifecycle::LifecycleNode * node_;
  double period_sec_;
  std_msgs::msg::String msg_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace helix_sensing_cpp

#endif  // HELIX_SENSING_CPP__HEARTBEAT_HPP_
