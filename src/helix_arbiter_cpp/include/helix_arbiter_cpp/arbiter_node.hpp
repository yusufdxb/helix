// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// ArbiterNode: rclcpp_lifecycle wrapper around helix_arbiter_cpp::Arbiter, a
// drop-in alternative to the Python helix_arbiter node
// (helix_arbiter/arbiter_node.py, the reference). Same contract:
//
//   * node name:  helix_arbiter (preflight checks expect /helix_arbiter)
//   * params:     helix_arbiter/config/arbiter.yaml, parsed by parse_config
//                 at configure; arbiter_backend = "cpp" (read-only)
//   * subs:       every sources.<name>.topic (geometry_msgs/Twist) and
//                 hold_topic (helix_msgs/HelixHold); KEEP_LAST 10,
//                 BEST_EFFORT, VOLATILE; created on activate
//   * pubs:       output_topic (geometry_msgs/Twist; KEEP_LAST 1, RELIABLE,
//                 VOLATILE) and status_topic (helix_msgs/ArbiterStatus;
//                 KEEP_LAST 50, RELIABLE, VOLATILE); lifecycle publishers
//   * output:     one decision per timer tick at rate_hz, plus an immediate
//                 tick when a hold assertion arrives
//   * shutdown:   deactivate, shutdown, error, SIGINT and SIGTERM publish
//                 shutdown_zero_count zero commands (reason SHUTDOWN) while
//                 the publishers are still enabled, then go silent
//
// Clocks: receipt freshness uses std::chrono::steady_clock (monotonic), and
// the output timer is a steady wall timer. The Python node's timer runs on
// the ROS clock, so with use_sim_time a paused /clock silences it; this node
// keeps publishing. ArbiterStatus.stamp is wall-clock seconds, for tracing.
#ifndef HELIX_ARBITER_CPP__ARBITER_NODE_HPP_
#define HELIX_ARBITER_CPP__ARBITER_NODE_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "helix_msgs/msg/arbiter_status.hpp"
#include "helix_msgs/msg/helix_hold.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "helix_arbiter_cpp/arbiter_config.hpp"
#include "helix_arbiter_cpp/arbiter_core.hpp"

namespace helix_arbiter_cpp
{

/// Value of the read-only arbiter_backend parameter.
inline constexpr const char * kBackendName = "cpp";

/// QoS profiles, identical to INPUT_QOS / SOURCE_QOS / OUTPUT_QOS / STATUS_QOS in the
/// Python node. BEST_EFFORT input matches reliable and best-effort
/// publishers; RELIABLE output matches reliable and best-effort subscribers.
rclcpp::QoS input_qos();
rclcpp::QoS source_qos();
rclcpp::QoS output_qos();
rclcpp::QoS status_qos();

class ArbiterNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit ArbiterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ArbiterNode() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  /// Publish the zero burst while the publishers are still enabled, then
  /// stop producing output. No-op unless active. Called by main() on
  /// SIGINT/SIGTERM and by the deactivate/shutdown/error transitions.
  void publish_shutdown_zero();

  /// The autostart parameter if it is a boolean; a non-boolean value is
  /// logged and treated as false (the string "false" must not start).
  bool autostart_requested();

  /// Every parameter of this node, flattened for parse_config.
  ParamMap collect_parameters() const;

private:
  void on_source(std::size_t index, const geometry_msgs::msg::Twist & msg);
  void on_hold(const helix_msgs::msg::HelixHold & msg);
  void on_tick();
  void emit(const Decision & d);
  void teardown_runtime();
  void destroy_publishers();
  double now() const;

  std::optional<ArbiterConfig> cfg_;
  std::unique_ptr<Arbiter> arb_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>> pub_out_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<helix_msgs::msg::ArbiterStatus>>
  pub_status_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::uint64_t seq_{0};
  // Last decision published, so a source callback publishes only on change.
  std::optional<Decision> last_decision_;
  bool active_{false};
  std::optional<std::pair<Reason, std::string>> last_logged_;
  rclcpp::Clock::SharedPtr throttle_clock_;
};

}  // namespace helix_arbiter_cpp

#endif  // HELIX_ARBITER_CPP__ARBITER_NODE_HPP_
