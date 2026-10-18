// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// helix_supervisor: lifecycle node around SupervisorCore.
//
// In:   /helix/faults        FaultEvent     faults from the detector, log parser, tools
//       /helix/fault_clear   FaultEvent     the detector's per-metric clearance proof
//       /helix/heartbeat     std_msgs/String  node liveness (10 Hz per node)
//       /helix/arbiter/status ArbiterStatus  arbiter acknowledgment of the hold
//       /helix/sink/status   SinkStatus     controller ack + measured robot motion
// Out:  /helix/hold          HelixHold      at hold_rate_hz and on every change
//       /helix/supervisor/status SupervisorStatus  on every change and at status_rate_hz
//       /helix/recovery_hints, /helix/recovery_actions  audit trail (legacy schema)
// Srv:  /helix/supervisor/release  helix_msgs/ReleaseHold  operator release
//
// actuation_enabled and auto_resume may be changed at runtime with
// `ros2 param set`; the change is applied to the live state machine at once.
// Everything runs on one single-threaded executor, so the core needs no lock.
// Time is the steady clock: the sink stamps its status on the same host's
// monotonic clock and the core compares the two directly.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "helix_msgs/msg/arbiter_status.hpp"
#include "helix_msgs/msg/fault_event.hpp"
#include "helix_msgs/msg/helix_hold.hpp"
#include "helix_msgs/msg/recovery_action.hpp"
#include "helix_msgs/msg/recovery_hint.hpp"
#include "helix_msgs/msg/sink_status.hpp"
#include "helix_msgs/msg/supervisor_status.hpp"
#include "helix_msgs/srv/release_hold.hpp"
#include "helix_supervisor_cpp/supervisor_core.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

namespace helix_supervisor_cpp
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace
{

double steady_now()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

rclcpp::QoS reliable(std::size_t depth)
{
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().durability_volatile();
}

}  // namespace

class SupervisorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit SupervisorNode(const rclcpp::NodeOptions & options)
  : LifecycleNode("helix_supervisor", options)
  {
    const Config d;
    declare_parameter("stop_metric_prefixes", d.stop_metric_prefixes);
    declare_parameter("stop_min_severity", d.stop_min_severity);
    declare_parameter("stop_on_critical_log", d.stop_on_critical_log);
    declare_parameter("critical_nodes", std::vector<std::string>{"*"});
    declare_parameter("expected_nodes", std::vector<std::string>{});
    declare_parameter("startup_grace_s", d.startup_grace_s);
    declare_parameter("heartbeat_timeout_s", d.heartbeat_timeout_s);
    declare_parameter("heartbeat_clear_s", d.heartbeat_clear_s);
    declare_parameter("clear_hold_s", d.clear_hold_s);
    declare_parameter("log_only_expiry_s", d.log_only_expiry_s);
    declare_parameter("auto_resume", d.auto_resume);
    declare_parameter("actuation_enabled", d.actuation_enabled);
    declare_parameter("require_controller_ack", d.require_controller_ack);
    declare_parameter("arbiter_ack_timeout_s", d.arbiter_ack_timeout_s);
    declare_parameter("controller_ack_timeout_s", d.controller_ack_timeout_s);
    declare_parameter("motion_stop_timeout_s", d.motion_stop_timeout_s);
    declare_parameter("hold_rate_hz", 20.0);
    declare_parameter("tick_rate_hz", 20.0);
    declare_parameter("status_rate_hz", 2.0);
    declare_parameter("hold_topic", std::string("/helix/hold"));
    declare_parameter("arbiter_status_topic", std::string("/helix/arbiter/status"));
    declare_parameter("sink_status_topic", std::string("/helix/sink/status"));

    epoch_ = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());

    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & ps) {return on_params(ps);});
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    Config c;
    c.stop_metric_prefixes = get_parameter("stop_metric_prefixes").as_string_array();
    c.stop_min_severity = get_parameter("stop_min_severity").as_int();
    c.stop_on_critical_log = get_parameter("stop_on_critical_log").as_bool();
    c.critical_nodes = get_parameter("critical_nodes").as_string_array();
    c.expected_nodes = get_parameter("expected_nodes").as_string_array();
    c.startup_grace_s = get_parameter("startup_grace_s").as_double();
    c.heartbeat_timeout_s = get_parameter("heartbeat_timeout_s").as_double();
    c.heartbeat_clear_s = get_parameter("heartbeat_clear_s").as_double();
    c.clear_hold_s = get_parameter("clear_hold_s").as_double();
    c.log_only_expiry_s = get_parameter("log_only_expiry_s").as_double();
    c.auto_resume = get_parameter("auto_resume").as_bool();
    c.actuation_enabled = get_parameter("actuation_enabled").as_bool();
    c.require_controller_ack = get_parameter("require_controller_ack").as_bool();
    c.arbiter_ack_timeout_s = get_parameter("arbiter_ack_timeout_s").as_double();
    c.controller_ack_timeout_s = get_parameter("controller_ack_timeout_s").as_double();
    c.motion_stop_timeout_s = get_parameter("motion_stop_timeout_s").as_double();
    try {
      core_ = std::make_unique<SupervisorCore>(c);
    } catch (const std::invalid_argument & e) {
      RCLCPP_ERROR(get_logger(), "refusing to configure: %s", e.what());
      return CallbackReturn::FAILURE;
    }
    for (const char * rate : {"hold_rate_hz", "tick_rate_hz", "status_rate_hz"}) {
      if (!(get_parameter(rate).as_double() > 0.0)) {
        RCLCPP_ERROR(get_logger(), "refusing to configure: %s must be > 0", rate);
        return CallbackReturn::FAILURE;
      }
    }

    pub_hold_ = create_publisher<helix_msgs::msg::HelixHold>(
      get_parameter("hold_topic").as_string(), reliable(10));
    pub_status_ = create_publisher<helix_msgs::msg::SupervisorStatus>(
      "/helix/supervisor/status", reliable(20));
    pub_hint_ = create_publisher<helix_msgs::msg::RecoveryHint>(
      "/helix/recovery_hints", reliable(20));
    pub_action_ = create_publisher<helix_msgs::msg::RecoveryAction>(
      "/helix/recovery_actions", reliable(50));
    pub_beat_ = create_publisher<std_msgs::msg::String>("/helix/heartbeat", reliable(10));

    RCLCPP_INFO(
      get_logger(), "configured: actuation_enabled=%s auto_resume=%s "
      "require_controller_ack=%s expected=%zu critical=%zu",
      c.actuation_enabled ? "true" : "false", c.auto_resume ? "true" : "false",
      c.require_controller_ack ? "true" : "false", c.expected_nodes.size(),
      c.critical_nodes.size());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    // A lifecycle restart is a new supervision session too. Core revision
    // may restart after cleanup/configure, so never reuse its authorization
    // epoch, even if the wall clock moved backward within this process.
    const auto activation_epoch = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    epoch_ = std::max(epoch_ + 1, activation_epoch);
    LifecycleNode::on_activate(state);
    core_->start(steady_now());
    arbiter_challenge_.clear();
    arbiter_protective_latched_ = false;
    arbiter_seen_at_ = 0.0;

    sub_fault_ = create_subscription<helix_msgs::msg::FaultEvent>(
      "/helix/faults", reliable(100),
      [this](helix_msgs::msg::FaultEvent::ConstSharedPtr m) {
        core_->on_fault(m->fault_type, m->node_name, m->severity, m->detail, steady_now());
        flush();
      });
    sub_clear_ = create_subscription<helix_msgs::msg::FaultEvent>(
      "/helix/fault_clear", reliable(100),
      [this](helix_msgs::msg::FaultEvent::ConstSharedPtr m) {
        core_->on_fault_clear(m->fault_type, m->node_name, steady_now());
        flush();
      });
    sub_beat_ = create_subscription<std_msgs::msg::String>(
      "/helix/heartbeat", reliable(200),
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        core_->on_heartbeat(m->data, steady_now());
      });
    sub_arbiter_ = create_subscription<helix_msgs::msg::ArbiterStatus>(
      get_parameter("arbiter_status_topic").as_string(), reliable(50),
      [this](helix_msgs::msg::ArbiterStatus::ConstSharedPtr m) {
        if (m->supervisor_epoch == epoch_) {
          arbiter_challenge_ = m->release_challenge;
          arbiter_protective_latched_ = m->protective_latched;
          arbiter_seen_at_ = steady_now();
        }
        core_->on_arbiter_status(m->reason, m->hold_fault_id, steady_now());
        flush();
      });
    sub_sink_ = create_subscription<helix_msgs::msg::SinkStatus>(
      get_parameter("sink_status_topic").as_string(), reliable(20),
      [this](helix_msgs::msg::SinkStatus::ConstSharedPtr m) {
        SinkFeedback fb;
        fb.feedback_available = m->feedback_available;
        fb.stop_acked = m->stop_acked;
        fb.stop_ack_sent_at = m->stop_ack_sent_at;
        fb.stop_acked_at = m->stop_acked_at;
        fb.stopped = m->stopped;
        fb.state_at = m->state_at;
        core_->on_sink_feedback(fb, steady_now());
        flush();
      });
    srv_release_ = create_service<helix_msgs::srv::ReleaseHold>(
      "/helix/supervisor/release",
      [this](const std::shared_ptr<helix_msgs::srv::ReleaseHold::Request> req,
      std::shared_ptr<helix_msgs::srv::ReleaseHold::Response> res) {
        const double now = steady_now();
        // Recheck liveness here, not at the preceding timer tick. An operator
        // request is never queued for later health or a later arbiter latch.
        core_->tick(now);
        std::string why;
        const bool protection_request = arbiter_protective_latched_ &&
        !arbiter_challenge_.empty() && now - arbiter_seen_at_ <= 0.5 &&
        req->supervisor_epoch == epoch_ && req->supervisor_revision == core_->revision() &&
        req->release_challenge == arbiter_challenge_;
        if (!protection_request) {
          why = "stale or mismatched operator authorization";
          res->success = false;
        } else if (!core_->missing_nodes(now).empty() || core_->state() == State::kStarting) {
          why = "fresh supervisor health required";
          res->success = false;
        } else if (core_->state() == State::kNominal) {
          res->success = true;
          why = "operator release requested";
        } else {
          res->success = core_->operator_release(now, why);
        }
        flush();
        if (res->success && protection_request && core_->state() == State::kNominal) {
          // First publish current health, then the one-shot authorization.
          // Periodic heartbeats and auto_resume never echo this challenge.
          publish_hold();
          publish_hold_value(false, "operator release requested", arbiter_challenge_);
          why = "operator release requested; check arbiter protective_latched";
        }
        res->message = why;
        RCLCPP_WARN(
          get_logger(), "operator release: %s (%s)",
          res->success ? "accepted" : "refused", why.c_str());
        flush();
      });

    using std::chrono::duration;
    tick_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        duration<double>(1.0 / get_parameter("tick_rate_hz").as_double())),
      [this]() {
        core_->tick(steady_now());
        flush();
      });
    hold_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        duration<double>(1.0 / get_parameter("hold_rate_hz").as_double())),
      [this]() {publish_hold();});
    status_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        duration<double>(1.0 / get_parameter("status_rate_hz").as_double())),
      [this]() {publish_status();});
    beat_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {
        std_msgs::msg::String m;
        m.data = get_name();
        pub_beat_->publish(m);
      });
    flush();
    publish_hold();
    RCLCPP_INFO(get_logger(), "active: %s", to_string(core_->state()));
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    // Tell the arbiter to hold now rather than wait for staleness; must go
    // out before the lifecycle publishers are disabled.
    publish_hold_value(true, "SUPERVISOR_DEACTIVATING");
    for (auto * t : {&tick_timer_, &hold_timer_, &status_timer_, &beat_timer_}) {
      if (*t) {
        (*t)->cancel();
        t->reset();
      }
    }
    sub_fault_.reset();
    sub_clear_.reset();
    sub_beat_.reset();
    sub_arbiter_.reset();
    sub_sink_.reset();
    srv_release_.reset();
    LifecycleNode::on_deactivate(state);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    pub_hold_.reset();
    pub_status_.reset();
    pub_hint_.reset();
    pub_action_.reset();
    pub_beat_.reset();
    core_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & s) override
  {
    if (pub_hold_ && pub_hold_->is_activated()) {
      publish_hold_value(true, "SUPERVISOR_SHUTDOWN");
    }
    return on_cleanup(s);
  }

private:
  rcl_interfaces::msg::SetParametersResult on_params(const std::vector<rclcpp::Parameter> & ps)
  {
    rcl_interfaces::msg::SetParametersResult r;
    r.successful = true;
    const bool active = core_ && get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    for (const auto & p : ps) {
      const auto & n = p.get_name();
      if (n == "actuation_enabled" || n == "auto_resume") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          r.successful = false;
          r.reason = n + " must be a bool";
          return r;
        }
        continue;
      }
      if (active) {
        // Everything else shapes the state machine and is fixed while active.
        r.successful = false;
        r.reason = n + " can only change while inactive";
        return r;
      }
    }
    if (active) {
      const double now = steady_now();
      for (const auto & p : ps) {
        if (p.get_name() == "actuation_enabled") {
          core_->set_actuation_enabled(p.as_bool(), now);
        } else if (p.get_name() == "auto_resume") {
          core_->set_auto_resume(p.as_bool(), now);
        }
      }
      // A parameter callback must not publish synchronously into its own
      // service response path; the next tick (<= 50 ms) flushes.
      pending_flush_ = true;
    }
    return r;
  }

  void flush()
  {
    pending_flush_ = false;
    for (const auto & e : core_->take_audit()) {
      if (e.kind == "HINT") {
        helix_msgs::msg::RecoveryHint h;
        h.fault_id = e.fault_id;
        h.suggested_action = e.action;
        h.confidence = static_cast<float>(e.confidence);
        h.reasoning = e.reason;
        h.rule_matched = e.rule;
        pub_hint_->publish(h);
      } else {
        helix_msgs::msg::RecoveryAction a;
        a.fault_id = e.fault_id;
        a.action = e.action;
        a.status = e.status;
        a.timestamp = e.stamp;
        a.reason = e.reason.empty() ? e.status : e.reason;
        pub_action_->publish(a);
        if (e.kind == "TRANSITION") {
          RCLCPP_INFO(get_logger(), "state %s: %s", e.action.c_str(), e.reason.c_str());
        } else {
          RCLCPP_WARN(
            get_logger(), "%s %s %s (%s)", e.action.c_str(), e.status.c_str(),
            e.fault_id.c_str(), e.reason.c_str());
        }
      }
    }
    const bool hold = core_->hold();
    if (hold != last_hold_ || core_->hold_id() != last_hold_id_) {
      publish_hold();   // hold changes go out from the triggering callback
    }
    if (core_->revision() != last_revision_) {
      publish_status();
    }
  }

  void publish_hold()
  {
    if (!core_ || !pub_hold_ || !pub_hold_->is_activated()) {
      return;
    }
    if (pending_flush_) {
      flush();
      return;
    }
    publish_hold_value(core_->hold(), to_string(core_->state()));
  }

  void publish_hold_value(
    bool hold, const std::string & reason, const std::string & release_challenge = "")
  {
    helix_msgs::msg::HelixHold m;
    m.hold = hold;
    m.fault_id = core_ ? core_->hold_id() : std::string();
    m.reason = reason;
    m.epoch = epoch_;
    m.seq = ++seq_;
    m.stamp = steady_now();
    m.asserted_stamp = hold && core_ ? core_->hold_requested_at() : 0.0;
    m.healthy = core_ && core_->state() == State::kNominal &&
      core_->missing_nodes(m.stamp).empty();
    m.release_challenge = release_challenge;
    pub_hold_->publish(m);
    last_hold_ = hold;
    last_hold_id_ = m.fault_id;
  }

  void publish_status()
  {
    if (!core_ || !pub_status_ || !pub_status_->is_activated()) {
      return;
    }
    const double now = steady_now();
    helix_msgs::msg::SupervisorStatus s;
    s.state = to_string(core_->state());
    s.hold = core_->hold();
    s.would_hold = core_->would_hold();
    s.actuation_enabled = core_->config().actuation_enabled;
    s.auto_resume = core_->config().auto_resume;
    s.hold_id = core_->hold_id();
    s.hold_cause = core_->hold_cause();
    s.reason = core_->reason();
    for (const Fault * f : core_->active_faults()) {
      s.active_faults.push_back(f->key);
      s.active_stop_worthy.push_back(f->stop_worthy ? 1 : 0);
    }
    s.missing_nodes = core_->missing_nodes(now);
    s.hold_requested_at = core_->hold_requested_at();
    s.arbiter_ack_at = core_->arbiter_ack_at();
    s.controller_ack_at = core_->controller_ack_at();
    s.motion_stopped_at = core_->motion_stopped_at();
    s.cleared_at = core_->cleared_at();
    s.stamp = now;
    s.seq = ++status_seq_;
    s.revision = core_->revision();
    s.hold_episodes = core_->hold_episodes();
    pub_status_->publish(s);
    last_revision_ = core_->revision();
  }

  std::unique_ptr<SupervisorCore> core_;
  std::string arbiter_challenge_;
  bool arbiter_protective_latched_{false};
  double arbiter_seen_at_{0.0};
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp_lifecycle::LifecyclePublisher<helix_msgs::msg::HelixHold>::SharedPtr pub_hold_;
  rclcpp_lifecycle::LifecyclePublisher<helix_msgs::msg::SupervisorStatus>::SharedPtr pub_status_;
  rclcpp_lifecycle::LifecyclePublisher<helix_msgs::msg::RecoveryHint>::SharedPtr pub_hint_;
  rclcpp_lifecycle::LifecyclePublisher<helix_msgs::msg::RecoveryAction>::SharedPtr pub_action_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr pub_beat_;
  rclcpp::Subscription<helix_msgs::msg::FaultEvent>::SharedPtr sub_fault_;
  rclcpp::Subscription<helix_msgs::msg::FaultEvent>::SharedPtr sub_clear_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_beat_;
  rclcpp::Subscription<helix_msgs::msg::ArbiterStatus>::SharedPtr sub_arbiter_;
  rclcpp::Subscription<helix_msgs::msg::SinkStatus>::SharedPtr sub_sink_;
  rclcpp::Service<helix_msgs::srv::ReleaseHold>::SharedPtr srv_release_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr hold_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr beat_timer_;
  std::uint64_t epoch_{0};
  std::uint64_t seq_{0};
  std::uint64_t status_seq_{0};
  std::uint64_t last_revision_{0};
  bool last_hold_{true};
  std::string last_hold_id_;
  bool pending_flush_{false};
};

}  // namespace helix_supervisor_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(helix_supervisor_cpp::SupervisorNode)
