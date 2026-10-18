// Copyright 2026 Yusuf Guenena
//
// Use of this source code is governed by an MIT-style license.
//
// helix_arbiter_cpp executable.
//
// rclcpp's default SIGINT/SIGTERM handlers shut the context down before user
// code runs, after which nothing can be published. Like the Python node, this
// main() therefore installs its own handlers (async-signal-safe: they only
// record the signal number), and on SIGINT or SIGTERM publishes
// shutdown_zero_count zero commands, spins briefly so the reliable burst is
// flushed, and exits 0. SIGKILL and power loss cannot be handled by any
// process; the downstream sink's deadman covers them.
//
// Not a composable component on purpose: inside a component container the
// container owns the signal handlers, and the zero burst on SIGINT/SIGTERM
// would be lost.

#include <signal.h>

#include <chrono>
#include <csignal>
#include <memory>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "helix_arbiter_cpp/arbiter_node.hpp"

namespace
{

volatile std::sig_atomic_t g_signal = 0;

extern "C" void record_signal(int signum)
{
  g_signal = signum;
}

void install_handlers()
{
  struct sigaction sa {};
  sa.sa_handler = record_signal;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);
}

}  // namespace

int main(int argc, char ** argv)
{
  using namespace std::chrono_literals;
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  install_handlers();
  {
    auto node = std::make_shared<helix_arbiter_cpp::ArbiterNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    if (node->autostart_requested()) {
      if (node->configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        node->activate();
      } else {
        RCLCPP_ERROR(node->get_logger(), "autostart: configure failed; staying unconfigured");
      }
    }
    while (g_signal == 0 && rclcpp::ok()) {
      executor.spin_once(50ms);
    }
    RCLCPP_INFO(
      node->get_logger(), "signal %d: publishing zero burst before exit",
      static_cast<int>(g_signal));
    node->publish_shutdown_zero();
    // Give DDS a moment to flush the reliable zero burst.
    const auto end = std::chrono::steady_clock::now() + 200ms;
    while (std::chrono::steady_clock::now() < end && rclcpp::ok()) {
      executor.spin_once(20ms);
    }
    executor.remove_node(node->get_node_base_interface());
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
