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
#include <cmath>
#include <cstdlib>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>

#include "helix_go2_sink_cpp/sport_guard_core.hpp"
#include "helix_msgs/msg/arbiter_status.hpp"
#include "helix_msgs/msg/fault_event.hpp"
#include "helix_msgs/msg/sink_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

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
    feedback_enabled_ = declare_parameter<bool>("feedback_enabled", false, ro);
    const bool ack_watchdog = declare_parameter<bool>("controller_ack_watchdog", false, ro);
    const double ack_timeout = declare_parameter<double>("controller_ack_timeout_s", 0.3, ro);
    const double state_timeout = declare_parameter<double>("controller_state_timeout_s", 0.3, ro);
    if (feedback_enabled_ || ack_watchdog) {
      downstream_ = std::make_unique<ControllerWatchdog>(
        ack_timeout, state_timeout, true, true);
      guard_->on_output_health(false, "CONTROLLER_PROOF_MISSING", monotonic_now());
      const auto state = declare_parameter<std::string>(
        "state_topic", "/helix/experimental/sportmodestate", ro);
      require_private(state);
      state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
        state, rclcpp::SensorDataQoS(),
        [this](unitree_go::msg::SportModeState::ConstSharedPtr message) {
          const double now = monotonic_now();
          const bool valid = std::isfinite(message->velocity[0]) &&
          std::isfinite(message->velocity[1]) && std::isfinite(message->yaw_speed);
          downstream_->on_state(valid, now);
          feedback_.on_state(
            message->velocity[0], message->velocity[1], message->yaw_speed, message->mode, now);
          update_output_health(now);
        });
      require_private(state_sub_->get_topic_name());
    }
    if (feedback_enabled_) {
      const auto feedback_topic = declare_parameter<std::string>(
        "feedback_topic", "/helix/experimental/sport/feedback", ro);
      require_private(feedback_topic);
      feedback_pub_ = create_publisher<helix_msgs::msg::SinkStatus>(feedback_topic, 20);
      require_private(feedback_pub_->get_topic_name());
    }
    const auto heartbeat_topic = declare_parameter<std::string>(
      "heartbeat_topic", "/helix/experimental/heartbeat", ro);
    heartbeat_pub_ = create_publisher<std_msgs::msg::String>(heartbeat_topic, 10);
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
        update_output_health(monotonic_now());
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
        const double now = monotonic_now();
        if (downstream_) {
          downstream_->on_response(
            message->header.identity.id,
            static_cast<std::int32_t>(message->header.identity.api_id),
            message->header.status.code, now);
          update_output_health(now);
        }
        feedback_.on_response(
          message->header.identity.id,
          static_cast<std::int32_t>(message->header.identity.api_id),
          message->header.status.code, monotonic_now());
      });
    require_private(response_sub_->get_topic_name());
    release_sub_ = create_subscription<std_msgs::msg::String>(
      release, rclcpp::QoS(10).reliable().durability_volatile(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        update_output_health(monotonic_now());
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
      if (downstream_) {
        downstream_->on_sent(
          request.header.identity.id,
          static_cast<std::int32_t>(request.header.identity.api_id), monotonic_now());
      }
      if (request.header.identity.id < 0) {
        // Track only this guard's monotonically increasing negative Stop ids.
        // Mixing them with sink ids would violate SinkFeedback's ordered-id
        // eviction contract. The sink independently tracks its positive ids.
        feedback_.on_sent(
          request.header.identity.id, kApiStopMove, reason, monotonic_now());
      } else if (request.header.identity.api_id == kApiMove) {
        // Invalidate old Stop proof. Do not mix positive sink ids with this
        // tracker's ordered negative ids. The independent watchdog retains
        // controller state; the next state sample repopulates this tracker.
        feedback_ = SinkFeedback({}, 1.0);
      }
    }
  }

  void update_output_health(double now)
  {
    if (downstream_) {
      const auto health = downstream_->health(now);
      guard_->on_output_health(health.healthy, health.reason, now);
    }
  }

  void publish_feedback(double now)
  {
    if (!feedback_pub_) {
      return;
    }
    const auto & f = feedback_;
    helix_msgs::msg::SinkStatus status;
    status.mode = mode_name(mode_);
    status.feedback_available = guard_->output_healthy() && f.feedback_available(now);
    status.last_api_id = f.last_api_id;
    status.last_request_id = f.last_request_id;
    status.last_reason = f.last_reason;
    status.last_sent_at = f.last_sent_at;
    status.last_ack_request_id = f.last_ack_request_id;
    status.last_ack_code = f.last_ack_code;
    status.last_ack_at = f.last_ack_at;
    status.stop_acked = f.stop_acked;
    status.stop_ack_sent_at = f.stop_ack_sent_at;
    status.stop_acked_at = f.stop_acked_at;
    status.speed_xy = f.speed_xy;
    status.yaw_speed = f.yaw_speed;
    status.state_at = f.state_at;
    status.robot_mode = f.robot_mode;
    status.stopped = f.stopped_now(now);
    status.stopped_since = f.stopped_since;
    status.seq = ++feedback_seq_;
    feedback_pub_->publish(status);
  }

  void tick()
  {
    const double now = monotonic_now();
    update_output_health(now);
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
      (feedback_.stop_acked ? "true" : "false") + ",\"output_healthy\":" +
      (guard_->output_healthy() ? "true" : "false") + ",\"output_detections\":" +
      std::to_string(guard_->output_health_detections()) + ",\"output_detected_at\":" +
      std::to_string(guard_->output_health_detected_at()) + ",\"output_reason\":\"" +
      guard_->output_health_reason() + "\",\"detected_at\":" +
      std::to_string(guard_->detected_at()) + ",\"measured_stopped\":" +
      (feedback_enabled_ && feedback_.stopped_now(now) ? "true" : "false") + "}";
    status_pub_->publish(status);
    publish_feedback(now);
    if (now - last_heartbeat_at_ >= 0.1) {
      std_msgs::msg::String beat;
      beat.data = get_name();
      heartbeat_pub_->publish(beat);
      last_heartbeat_at_ = now;
    }
  }

  Mode mode_{Mode::kDryRun};
  std::unique_ptr<SportGuard> guard_;
  std::unique_ptr<ControllerWatchdog> downstream_;
  bool feedback_enabled_{false};
  SinkFeedback feedback_;
  std::uint64_t feedback_seq_{0};
  double last_heartbeat_at_{0.0};
  std::int64_t generated_id_{0};
  double last_fault_at_{0.0};
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<helix_msgs::msg::FaultEvent>::SharedPtr fault_pub_;
  rclcpp::Publisher<helix_msgs::msg::SinkStatus>::SharedPtr feedback_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;
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
