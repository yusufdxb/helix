// A configurable pipeline stage for controlled fault-injection experiments.
// One binary plays every role (driver, filter, planner, ...) by parameters, and a
// fault can be injected at runtime on /helix_demo/inject as "node:fault[:arg]".
// The faults are synthetic stand-ins for real failure modes; which repair fixes
// each one is fixed by construction and documented in docs/v3/.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using Str = std_msgs::msg::String;

class DemoNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  DemoNode() : LifecycleNode("demo_node") {
    outputs_ = declare_parameter<std::vector<std::string>>("outputs", std::vector<std::string>{});
    inputs_ = declare_parameter<std::vector<std::string>>("inputs", std::vector<std::string>{});
    rate_hz_ = declare_parameter<double>("rate_hz", 0.0);
    payload_ = declare_parameter<int>("payload", 64);
    pub_best_effort_ = declare_parameter<bool>("pub_best_effort", false);
    sub_reliable_ = declare_parameter<bool>("sub_reliable", false);
    input_timeout_s_ = declare_parameter<double>("input_timeout_s", 0.0);
    work_us_ = declare_parameter<int>("work_us", 0);
    latch_s_ = declare_parameter<double>("latch_on_input_loss_s", 0.0);
    fault_file_ = declare_parameter<std::string>("fault_file", "");
    declare_parameter<std::string>("tag", "");
    lifecycle_ = declare_parameter<bool>("lifecycle", false);

    // Persistent faults survive a restart: they model a broken device or a bad deployed config.
    if (!fault_file_.empty()) {
      std::ifstream f(fault_file_);
      for (std::string line; std::getline(f, line);) {
        if (line == "device_missing") {
          RCLCPP_FATAL(get_logger(), "device not found (persistent fault)");
          std::exit(1);
        }
        if (line == "qos_mismatch") sub_reliable_ = true;
      }
    }
    // The inject channel is outside the managed endpoints so it survives endpoint_loss and cleanup.
    inject_ = create_subscription<Str>("/helix_demo/inject", rclcpp::QoS(10).reliable(),
                                       [this](Str::ConstSharedPtr m) { on_inject(m->data); });
    msg_.data.assign(static_cast<std::size_t>(std::max(0, payload_)), 'x');
  }

  bool lifecycle() const { return lifecycle_; }

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    degrade_ = 1.0;
    acc_ = 0;
    latched_ = false;
    create_endpoints();
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    active_ = true;
    last_in_.assign(inputs_.size(), now().seconds());
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    active_ = false;
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
    destroy_endpoints();
    return CallbackReturn::SUCCESS;
  }

 private:
  void create_endpoints() {
    rclcpp::QoS pq(10), sq(10);
    if (pub_best_effort_) pq.best_effort(); else pq.reliable();
    if (sub_reliable_) sq.reliable(); else sq.best_effort();
    pubs_.clear();
    subs_.clear();
    for (const auto& o : outputs_) pubs_.push_back(rclcpp::create_publisher<Str>(*this, o, pq));
    last_in_.assign(inputs_.size(), now().seconds());
    for (std::size_t k = 0; k < inputs_.size(); ++k)
      subs_.push_back(create_subscription<Str>(inputs_[k], sq, [this, k](Str::ConstSharedPtr) { on_input(k); }));
    if (rate_hz_ > 0) timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz_), [this] { on_timer(); });
  }
  void destroy_endpoints() {
    pubs_.clear();
    subs_.clear();
    timer_.reset();
  }

  // Like a fusion node, the stage needs every input: the oldest one decides.
  double last_input() const {
    double m = 1e300;
    for (double v : last_in_) m = std::min(m, v);
    return last_in_.empty() ? now().seconds() : m;
  }
  bool input_ok(double t) const { return input_timeout_s_ <= 0 || t - last_input() <= input_timeout_s_; }

  void on_input(std::size_t k) {
    last_in_[k] = now().seconds();
    if (rate_hz_ <= 0) emit();
  }

  void on_timer() {
    const double t = now().seconds();
    if (latch_s_ > 0 && !inputs_.empty() && t - last_input() > latch_s_) latched_ = true;
    if (!input_ok(t)) return;
    emit();
  }

  void emit() {
    if (!active_ || latched_) return;
    acc_ += degrade_;
    if (acc_ < 1.0) return;
    acc_ -= 1.0;
    if (work_us_ > 0) {
      const auto end = std::chrono::steady_clock::now() + std::chrono::microseconds(work_us_);
      while (std::chrono::steady_clock::now() < end) {
      }
    }
    for (auto& p : pubs_) p->publish(msg_);
  }

  void persist(const std::string& what) {
    if (fault_file_.empty()) return;
    std::ofstream f(fault_file_, std::ios::app);
    f << what << "\n";
  }

  void on_inject(const std::string& s) {
    const auto c1 = s.find(':');
    if (c1 == std::string::npos || s.substr(0, c1) != get_name()) return;
    std::string fault = s.substr(c1 + 1), arg;
    if (const auto c2 = fault.find(':'); c2 != std::string::npos) {
      arg = fault.substr(c2 + 1);
      fault.resize(c2);
    }
    RCLCPP_WARN(get_logger(), "inject %s %s", fault.c_str(), arg.c_str());
    if (fault == "hang") {
      for (;;) std::this_thread::sleep_for(1s);  // the executor thread never returns
    } else if (fault == "endpoint_loss") {
      destroy_endpoints();
    } else if (fault == "cpu_hog") {
      const int n = arg.empty() ? 1 : std::atoi(arg.c_str());
      for (int i = 0; i < n; ++i)
        std::thread([] {
          volatile unsigned long x = 0;
          for (;;) x = x + 1;
        }).detach();
    } else if (fault == "leak") {
      const double mbps = arg.empty() ? 50.0 : std::atof(arg.c_str());
      leak_timer_ = create_wall_timer(100ms, [this, mbps] {
        const std::size_t n = static_cast<std::size_t>(mbps * 1e5);  // MB/s * 1e6 / 10 Hz
        auto* p = static_cast<char*>(std::malloc(n));
        if (p) std::memset(p, 1, n);
        leaked_.push_back(p);
      });
    } else if (fault == "degrade") {
      degrade_ = arg.empty() ? 0.3 : std::atof(arg.c_str());
    } else if (fault == "segv") {
      std::raise(SIGSEGV);
    } else if (fault == "qos_mismatch") {
      persist("qos_mismatch");
      sub_reliable_ = true;
      create_endpoints();
    } else if (fault == "device_missing") {
      persist("device_missing");
      RCLCPP_FATAL(get_logger(), "device lost");
      std::exit(1);
    } else if (fault == "exit") {
      std::exit(arg.empty() ? 2 : std::atoi(arg.c_str()));
    }
  }

  std::vector<std::string> outputs_, inputs_;
  double rate_hz_ = 0, input_timeout_s_ = 0, latch_s_ = 0;
  int payload_ = 64, work_us_ = 0;
  bool pub_best_effort_ = false, sub_reliable_ = false, lifecycle_ = false;
  std::string fault_file_;
  bool active_ = false, latched_ = false;
  double degrade_ = 1.0, acc_ = 0;
  std::vector<double> last_in_;
  Str msg_;
  std::vector<rclcpp::Publisher<Str>::SharedPtr> pubs_;
  std::vector<rclcpp::Subscription<Str>::SharedPtr> subs_;
  rclcpp::Subscription<Str>::SharedPtr inject_;
  rclcpp::TimerBase::SharedPtr timer_, leak_timer_;
  std::vector<void*> leaked_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DemoNode>();
  if (!node->lifecycle()) {
    node->configure();
    node->activate();
  }
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(node->get_node_base_interface());
  ex.spin();
  rclcpp::shutdown();
  return 0;
}
