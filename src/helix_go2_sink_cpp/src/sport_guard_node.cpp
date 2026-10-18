// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// EXPERIMENTAL, LOCALHOST SIMULATION ONLY. Not part of native launch.
// Sink: request_topic:=/helix/experimental/sport/intent and
// response_topic:=/helix/experimental/sport/response. The guard is the sole
// publisher on /helix/experimental/sport/request. The controller subscribes
// there and replies on the private response topic. Forwarded positive sink
// ids are preserved, so both sink and guard can independently match replies.
// Guard StopMove ids are negative and cannot acknowledge a sink request.
// Arbiter release establishes readiness only. A separate operator must publish
// the current guard_status.release_token String on the private release topic.
// Tokens bind to this random guard instance, protection event and readiness
// interval. A token read before health or an expired interval cannot be banked.
//
// This process protects against sink loss, not its own death, host loss,
// DDS loss, controller rejection, another controller publisher, or actuator
// failure. Publication and acknowledgement are NOT measured stop evidence.
#include <chrono>
#include <cstdlib>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>

#include "helix_go2_sink_cpp/sport_guard_core.hpp"
#include "helix_msgs/msg/arbiter_status.hpp"
#include "helix_msgs/msg/fault_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"

namespace helix_go2_sink_cpp
{
namespace
{
double monotonic_now()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

void require_private(const std::string & topic)
{
  if (topic.rfind("/helix/experimental/", 0) != 0) {
    throw std::invalid_argument("Sport guard topics must remain under /helix/experimental/");
  }
}

std::string random_guard_session()
{
  std::random_device random;
  if (random.entropy() == 0.0) {
    throw std::runtime_error("guard requires an operating-system random source");
  }
  std::ostringstream session;
  session << "guard";
  for (int i = 0; i < 4; ++i) {
    session << ":" << std::hex << random();
  }
  return session.str();
}
}  // namespace

class SportGuardNode : public rclcpp::Node
{
public:
  SportGuardNode()
  : Node("helix_sport_guard"), feedback_({}, 1.0)
  {
    rcl_interfaces::msg::ParameterDescriptor ro;
    ro.read_only = true;
    mode_ = parse_mode(declare_parameter<std::string>("mode", "dry_run", ro));
    const char * localhost = std::getenv("ROS_LOCALHOST_ONLY");
    if (mode_ != Mode::kDryRun && (!localhost || std::string(localhost) != "1")) {
      throw std::invalid_argument("non-dry guard requires ROS_LOCALHOST_ONLY=1");
    }
    const auto intent = declare_parameter<std::string>(
      "intent_topic", "/helix/experimental/sport/intent", ro);
    const auto request = declare_parameter<std::string>(
      "request_topic", "/helix/experimental/sport/request", ro);
    const auto response = declare_parameter<std::string>(
      "response_topic", "/helix/experimental/sport/response", ro);
    const auto arbiter = declare_parameter<std::string>(
      "arbiter_status_topic", "/helix/experimental/arbiter/status", ro);
    const auto status = declare_parameter<std::string>(
      "status_topic", "/helix/experimental/sport/guard_status", ro);
    const auto release = declare_parameter<std::string>(
      "release_topic", "/helix/experimental/sport/release", ro);
    const auto faults = declare_parameter<std::string>(
      "fault_topic", "/helix/experimental/faults", ro);
    for (const auto & topic : {intent, request, response, release}) {
      require_private(topic);
    }
    guard_ = std::make_unique<SportGuard>(
      random_guard_session(),
      declare_parameter<double>("move_timeout_s", 0.25, ro),
      declare_parameter<double>("idle_timeout_s", 0.75, ro),
      declare_parameter<double>("status_timeout_s", 0.30, ro),
      declare_parameter<double>("stop_period_s", 0.05, ro));
    generated_id_ = -std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    if (mode_ != Mode::kDryRun) {
      request_pub_ = create_publisher<unitree_api::msg::Request>(request, 20);
      require_private(request_pub_->get_topic_name());  // Also inspect resolved remaps.
    }
    status_pub_ = create_publisher<std_msgs::msg::String>(status, 20);
    fault_pub_ = create_publisher<helix_msgs::msg::FaultEvent>(faults, 20);
    intent_sub_ = create_subscription<unitree_api::msg::Request>(
      intent, rclcpp::QoS(20).reliable().durability_volatile(),
      [this](unitree_api::msg::Request::ConstSharedPtr message) {
        Request request;
        request.api_id = static_cast<std::int32_t>(message->header.identity.api_id);
        request.parameter = message->parameter;
        request.reason = "SINK_INTENT";
        const auto accepted = guard_->on_request(
          message->header.identity.id, request, monotonic_now());
        if (accepted && (mode_ == Mode::kArmed || accepted->api_id == kApiStopMove)) {
          send(*message, accepted->reason);
        }
      });
    require_private(intent_sub_->get_topic_name());
    response_sub_ = create_subscription<unitree_api::msg::Response>(
      response, rclcpp::QoS(20).best_effort().durability_volatile(),
      [this](unitree_api::msg::Response::ConstSharedPtr message) {
        feedback_.on_response(
          message->header.identity.id,
          static_cast<std::int32_t>(message->header.identity.api_id),
          message->header.status.code, monotonic_now());
      });
    require_private(response_sub_->get_topic_name());
    release_sub_ = create_subscription<std_msgs::msg::String>(
      release, rclcpp::QoS(10).reliable().durability_volatile(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        guard_->on_operator_release(message->data, monotonic_now());
      });
    require_private(release_sub_->get_topic_name());
    status_sub_ = create_subscription<helix_msgs::msg::ArbiterStatus>(
      arbiter, rclcpp::QoS(20).reliable().durability_volatile(),
      [this](helix_msgs::msg::ArbiterStatus::ConstSharedPtr message) {
        guard_->on_status(
          message->seq, message->release_challenge, message->protective_latched,
          message->hold_active, monotonic_now());
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {tick();});
    RCLCPP_WARN(get_logger(), "EXPERIMENTAL localhost Sport guard, mode=%s", mode_name(mode_));
  }

  ~SportGuardNode() override
  {
    if (timer_) {
      timer_->cancel();
    }
  }

private:
  void send(const unitree_api::msg::Request & request, const std::string & reason)
  {
    if (request_pub_) {
      request_pub_->publish(request);
      if (request.header.identity.id < 0) {
        // Track only this guard's monotonically increasing negative Stop ids.
        // Mixing them with sink ids would violate SinkFeedback's ordered-id
        // eviction contract. The sink independently tracks its positive ids.
        feedback_.on_sent(
          request.header.identity.id, kApiStopMove, reason, monotonic_now());
      } else if (request.header.identity.api_id == kApiMove) {
        feedback_ = SinkFeedback({}, 1.0);  // A newer Move invalidates every stop ack.
      }
    }
  }

  void tick()
  {
    const double now = monotonic_now();
    if (const auto stop = guard_->on_tick(now)) {
      unitree_api::msg::Request request;
      request.header.identity.id = ++generated_id_;
      request.header.identity.api_id = kApiStopMove;
      request.header.policy.noreply = false;
      send(request, stop->reason);
    }
    if (guard_->fault_required() && now - last_fault_at_ >= 0.10) {
      helix_msgs::msg::FaultEvent fault;
      fault.node_name = get_name();
      fault.fault_type = "LOG_PATTERN";
      fault.severity = 3;
      fault.detail = "EXPERIMENTAL Sport guard protective latch: " + guard_->reason();
      fault.timestamp = get_clock()->now().seconds();
      fault_pub_->publish(fault);
      last_fault_at_ = now;
    }
    std_msgs::msg::String status;
    status.data = std::string("{\"experimental_simulation_only\":true,\"latched\":") +
      (guard_->latched() ? "true" : "false") + ",\"fault_required\":" +
      (guard_->fault_required() ? "true" : "false") + ",\"reason\":\"" + guard_->reason() +
      "\",\"release_token\":\"" + guard_->release_token() + "\",\"release_ready\":" +
      (guard_->ready_for_release(now) ? "true" : "false") +
      ",\"mode\":\"" + mode_name(mode_) + "\",\"last_request_id\":" +
      std::to_string(feedback_.last_request_id) + ",\"stop_acked\":" +
      (feedback_.stop_acked ? "true" : "false") + ",\"measured_stopped\":false}";
    status_pub_->publish(status);
  }

  Mode mode_{Mode::kDryRun};
  std::unique_ptr<SportGuard> guard_;
  SinkFeedback feedback_;
  std::int64_t generated_id_{0};
  double last_fault_at_{0.0};
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<helix_msgs::msg::FaultEvent>::SharedPtr fault_pub_;
  rclcpp::Subscription<unitree_api::msg::Request>::SharedPtr intent_sub_;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr response_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr release_sub_;
  rclcpp::Subscription<helix_msgs::msg::ArbiterStatus>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace helix_go2_sink_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<helix_go2_sink_cpp::SportGuardNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("helix_sport_guard"), "%s", error.what());
    result = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
