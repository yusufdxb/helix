// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// See include/helix_arbiter_cpp/arbiter_node.hpp for the contract.

#include "helix_arbiter_cpp/arbiter_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace helix_arbiter_cpp
{

namespace
{

constexpr const char * kBackendParam = "arbiter_backend";

rclcpp::ParameterValue to_ros(const ParamValue & v)
{
  if (const auto * b = std::get_if<bool>(&v)) {
    return rclcpp::ParameterValue(*b);
  }
  if (const auto * i = std::get_if<std::int64_t>(&v)) {
    return rclcpp::ParameterValue(*i);
  }
  if (const auto * d = std::get_if<double>(&v)) {
    return rclcpp::ParameterValue(*d);
  }
  if (const auto * s = std::get_if<std::string>(&v)) {
    return rclcpp::ParameterValue(*s);
  }
  return rclcpp::ParameterValue();
}

ParamValue from_ros(const rclcpp::Parameter & p)
{
  switch (p.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL: return p.as_bool();
    case rclcpp::ParameterType::PARAMETER_INTEGER: return p.as_int();
    case rclcpp::ParameterType::PARAMETER_DOUBLE: return p.as_double();
    case rclcpp::ParameterType::PARAMETER_STRING: return p.as_string();
    default: return Unsupported{};
  }
}

double seconds_since_epoch(std::chrono::system_clock::time_point t)
{
  return std::chrono::duration<double>(t.time_since_epoch()).count();
}

bool same_decision(const Decision & a, const Decision & b)
{
  return a.command == b.command && a.reason == b.reason && a.source == b.source &&
         a.hold_fault_id == b.hold_fault_id;
}

}  // namespace

rclcpp::QoS input_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
}

// Velocity sources keep only their newest command: a backlog of superseded
// commands would be applied in turn, each stamped fresh on receipt. The hold
// topic keeps input_qos(), because its transitions must not be dropped.
rclcpp::QoS source_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

rclcpp::QoS output_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

rclcpp::QoS status_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(50)).reliable().durability_volatile();
}

ArbiterNode::ArbiterNode(const rclcpp::NodeOptions & options)
: LifecycleNode(
    "helix_arbiter",
    rclcpp::NodeOptions(options)
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)),
  throttle_clock_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME))
{
  // Parameters from the YAML were declared from the overrides with their
  // YAML types; only the missing ones get the defaults, as in the Python node.
  for (const auto & [name, value] : default_parameters()) {
    if (!has_parameter(name)) {
      declare_parameter(name, to_ros(value));
    }
  }
  if (has_parameter(kBackendParam)) {
    // A parameter file cannot relabel the implementation.
    undeclare_parameter(kBackendParam);
  }
  rcl_interfaces::msg::ParameterDescriptor backend;
  backend.read_only = true;
  backend.description = "implementation of this node";
  declare_parameter(
    kBackendParam, rclcpp::ParameterValue(std::string(kBackendName)), backend, true);
}

ArbiterNode::~ArbiterNode()
{
  teardown_runtime();
}

ParamMap ArbiterNode::collect_parameters() const
{
  ParamMap out;
  const auto names = list_parameters({}, 0).names;  // depth 0: recursive
  for (const auto & p : get_parameters(names)) {
    out.emplace(p.get_name(), from_ros(p));
  }
  return out;
}

bool ArbiterNode::autostart_requested()
{
  const rclcpp::Parameter p = get_parameter("autostart");
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
    RCLCPP_ERROR(
      get_logger(), "autostart must be a boolean (got %s); not starting",
      p.value_to_string().c_str());
    return false;
  }
  return p.as_bool();
}

// -- lifecycle ---------------------------------------------------------------

ArbiterNode::CallbackReturn ArbiterNode::on_configure(const rclcpp_lifecycle::State &)
{
  const ParamMap params = collect_parameters();
  ArbiterConfig cfg;
  std::unique_ptr<Arbiter> arb;
  try {
    cfg = parse_config(params);
    arb = std::make_unique<Arbiter>(cfg.sources, cfg.hold_timeout_sec, cfg.limits);
    // Compare RESOLVED names, so a relative alias ("cmd_vel" in the root
    // namespace) or a remapping cannot feed the arbiter its own output.
    const auto topics = get_node_topics_interface();
    auto resolve = [&topics](const std::string & name) {
        return topics->resolve_topic_name(name);
      };
    std::vector<std::pair<std::string, std::string>> source_topics;
    for (const auto & s : cfg.sources) {
      source_topics.emplace_back(s.name, resolve(s.topic));
    }
    check_topic_layout(
      resolve(cfg.output_topic), resolve(cfg.status_topic), resolve(cfg.hold_topic),
      source_topics);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "bad arbiter configuration: %s", e.what());
    return CallbackReturn::FAILURE;
  }
  const auto unknown = unknown_parameters(params);
  if (!unknown.empty()) {
    std::string joined;
    for (const auto & n : unknown) {
      joined += (joined.empty() ? "" : ", ") + n;
    }
    RCLCPP_WARN(get_logger(), "ignoring unknown parameters [%s]", joined.c_str());
  }
  try {
    pub_out_ = create_publisher<geometry_msgs::msg::Twist>(cfg.output_topic, output_qos());
    pub_status_ = create_publisher<helix_msgs::msg::ArbiterStatus>(
      cfg.status_topic, status_qos());
  } catch (const std::exception & e) {
    destroy_publishers();
    RCLCPP_ERROR(get_logger(), "cannot create publishers: %s", e.what());
    return CallbackReturn::FAILURE;
  }
  cfg_ = std::move(cfg);
  arb_ = std::move(arb);
  std::string sources;
  for (const auto & s : cfg_->sources) {
    sources += (sources.empty() ? "" : ", ") + s.name + " " + s.topic + " prio=" +
      std::to_string(s.priority) + " timeout=" + std::to_string(s.timeout_sec);
  }
  RCLCPP_INFO(
    get_logger(), "configured: output=%s sources=[%s] hold_timeout=%.2fs rate=%.1fHz",
    cfg_->output_topic.c_str(), sources.c_str(), arb_->hold_timeout_sec(), cfg_->rate_hz);
  return CallbackReturn::SUCCESS;
}

ArbiterNode::CallbackReturn ArbiterNode::on_activate(const rclcpp_lifecycle::State & state)
{
  arb_->reset();
  for (std::size_t i = 0; i < arb_->source_count(); ++i) {
    subs_.push_back(
      create_subscription<geometry_msgs::msg::Twist>(
        arb_->spec(i).topic, source_qos(),
        [this, i](geometry_msgs::msg::Twist::ConstSharedPtr m) {on_source(i, *m);}));
  }
  subs_.push_back(
    create_subscription<helix_msgs::msg::HelixHold>(
      cfg_->hold_topic, input_qos(),
      [this](helix_msgs::msg::HelixHold::ConstSharedPtr m) {on_hold(*m);}));
  LifecycleNode::on_activate(state);  // enables the managed publishers

  // Best effort only: discovery is asynchronous, so a competing publisher
  // that is not yet discovered is not reported. Preflight C5 checks the
  // live graph before any hardware stage; launch selection is exclusive.
  const auto own = pub_out_->get_gid();
  std::size_t others = 0;
  for (const auto & info : get_publishers_info_by_topic(pub_out_->get_topic_name())) {
    const auto & gid = info.endpoint_gid();
    if (std::memcmp(gid.data(), own.data, std::min(gid.size(), sizeof(own.data))) != 0) {
      ++others;
    }
  }
  if (others > 0) {
    RCLCPP_WARN(
      get_logger(), "%zu other publisher(s) on %s: the arbiter must be its only publisher",
      others, pub_out_->get_topic_name());
  }

  last_decision_.reset();
  active_ = true;
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / cfg_->rate_hz));
  timer_ = create_wall_timer(period, [this]() {on_tick();});
  return CallbackReturn::SUCCESS;
}

ArbiterNode::CallbackReturn ArbiterNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  publish_shutdown_zero();
  teardown_runtime();
  LifecycleNode::on_deactivate(state);
  return CallbackReturn::SUCCESS;
}

ArbiterNode::CallbackReturn ArbiterNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  teardown_runtime();
  destroy_publishers();
  arb_.reset();
  cfg_.reset();
  return CallbackReturn::SUCCESS;
}

ArbiterNode::CallbackReturn ArbiterNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  publish_shutdown_zero();
  return on_cleanup(state);
}

ArbiterNode::CallbackReturn ArbiterNode::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_ERROR(
    get_logger(), "lifecycle error from state '%s': zeroing output and unconfiguring",
    state.label().c_str());
  publish_shutdown_zero();
  return on_cleanup(state);
}

void ArbiterNode::teardown_runtime()
{
  active_ = false;
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
  subs_.clear();
  if (arb_) {
    arb_->reset();
  }
}

void ArbiterNode::destroy_publishers()
{
  pub_out_.reset();
  pub_status_.reset();
}

// -- callbacks ---------------------------------------------------------------

double ArbiterNode::now() const
{
  // Monotonic receipt clock: immune to wall-clock steps on the robot payload.
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

void ArbiterNode::on_source(std::size_t index, const geometry_msgs::msg::Twist & msg)
{
  if (!arb_) {
    return;
  }
  Rejection why = Rejection::kNone;
  const Twist6 in{msg.linear.x, msg.linear.y, msg.linear.z,
    msg.angular.x, msg.angular.y, msg.angular.z};
  if (!arb_->on_source(index, in, now(), &why)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *throttle_clock_, 1000, "rejected malformed command from %s (%s)",
      arb_->spec(index).name.c_str(), rejection_name(why));
  }
  // Publish at once when this input changes what the arbiter outputs
  // (including a rejected command invalidating its source), instead of
  // waiting up to one timer period. The full decision runs as on a tick; the
  // timer still refreshes the output at rate_hz.
  if (active_) {
    const Decision d = arb_->decide(now());
    if (!last_decision_ || !same_decision(d, *last_decision_)) {
      emit(d);
    }
  }
}

void ArbiterNode::on_hold(const helix_msgs::msg::HelixHold & msg)
{
  if (!arb_) {
    return;
  }
  arb_->on_hold(msg.hold, msg.fault_id, msg.epoch, msg.seq, now());
  // Apply a hold assertion on the very next publish, not up to 1/rate later.
  if (msg.hold && active_) {
    on_tick();
  }
}

void ArbiterNode::on_tick()
{
  if (!active_ || !arb_) {
    return;
  }
  emit(arb_->decide(now()));
}

void ArbiterNode::emit(const Decision & d)
{
  geometry_msgs::msg::Twist out;
  out.linear.x = d.command.vx;
  out.linear.y = d.command.vy;
  out.angular.z = d.command.wz;
  pub_out_->publish(out);
  last_decision_ = d;
  ++seq_;

  helix_msgs::msg::ArbiterStatus st;
  st.selected_source = d.source;
  st.reason = reason_name(d.reason);
  st.hold_active = d.helix_forced();
  st.hold_fault_id = d.hold_fault_id;
  st.out_linear_x = d.command.vx;
  st.out_linear_y = d.command.vy;
  st.out_angular_z = d.command.wz;
  st.stamp = seconds_since_epoch(std::chrono::system_clock::now());
  st.seq = seq_;
  // uint32 / int32 fields: saturate instead of wrapping.
  const std::uint64_t rejected = arb_ ? arb_->counters().rejected : 0;
  st.rejected_total = static_cast<std::uint32_t>(
    std::min<std::uint64_t>(rejected, std::numeric_limits<std::uint32_t>::max()));
  st.sink_subscribers = static_cast<std::int32_t>(
    std::min<std::size_t>(
      pub_out_->get_subscription_count(),
      static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max())));
  pub_status_->publish(st);

  if (!last_logged_ || last_logged_->first != d.reason || last_logged_->second != d.source) {
    RCLCPP_INFO(
      get_logger(), "selected reason=%s source=%s cmd=(%+.3f,%+.3f,%+.3f) fault=%s",
      reason_name(d.reason), d.source.empty() ? "-" : d.source.c_str(),
      d.command.vx, d.command.vy, d.command.wz,
      d.hold_fault_id.empty() ? "-" : d.hold_fault_id.c_str());
    last_logged_ = std::make_pair(d.reason, d.source);
  }
}

void ArbiterNode::publish_shutdown_zero()
{
  if (!active_ || !pub_out_ || !cfg_) {
    return;
  }
  Decision zero;
  zero.reason = Reason::kShutdown;
  for (std::int64_t i = 0; i < cfg_->shutdown_zero_count; ++i) {
    emit(zero);
  }
  active_ = false;
}

}  // namespace helix_arbiter_cpp
