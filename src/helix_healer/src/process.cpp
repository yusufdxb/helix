#include "helix_healer/process.hpp"

#include <fcntl.h>
#include <signal.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <fstream>
#include <future>
#include <sstream>
#include <thread>

namespace helix_healer {

std::string ExitInfo::describe() const {
  if (WIFEXITED(status)) return "exit " + std::to_string(WEXITSTATUS(status));
  if (WIFSIGNALED(status)) {
    const int sig = WTERMSIG(status);
    return "signal " + std::to_string(sig) + " (" + strsignal(sig) + ")";
  }
  return "status " + std::to_string(status);
}

Supervisor::Supervisor() {
  forker_ = std::thread([this] {
    std::unique_lock<std::mutex> lk(fork_mu_);
    for (;;) {
      fork_cv_.wait(lk, [&] { return fork_stop_ || !fork_q_.empty(); });
      if (fork_q_.empty()) return;  // stop requested and nothing left
      auto job = std::move(fork_q_.front());
      fork_q_.pop_front();
      lk.unlock();
      job();
      lk.lock();
    }
  });
}

Supervisor::~Supervisor() {
  {
    std::lock_guard<std::mutex> lk(fork_mu_);
    fork_stop_ = true;
  }
  fork_cv_.notify_all();
  forker_.join();
  terminate_all(1.0);
}

pid_t Supervisor::spawn(const std::vector<std::string>& argv, const std::string& log_path) {
  if (argv.empty()) return -1;
  if (std::this_thread::get_id() == forker_.get_id()) return do_spawn(argv, log_path);
  std::promise<pid_t> done;
  auto fut = done.get_future();
  {
    std::lock_guard<std::mutex> lk(fork_mu_);
    if (fork_stop_) return -1;
    fork_q_.push_back([&] { done.set_value(do_spawn(argv, log_path)); });
  }
  fork_cv_.notify_one();
  return fut.get();
}

pid_t Supervisor::do_spawn(const std::vector<std::string>& argv, const std::string& log_path) {
  std::vector<char*> args;
  for (const auto& a : argv) args.push_back(const_cast<char*>(a.c_str()));
  args.push_back(nullptr);
  const pid_t parent = getpid();
  const pid_t pid = fork();
  if (pid < 0) return -1;
  if (pid == 0) {
    setpgid(0, 0);
    prctl(PR_SET_PDEATHSIG, SIGTERM);
    if (getppid() != parent) _exit(127);  // parent died between fork and prctl
    if (!log_path.empty()) {
      const int fd = open(log_path.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
      if (fd >= 0) {
        dup2(fd, STDOUT_FILENO);
        dup2(fd, STDERR_FILENO);
        close(fd);
      }
    }
    execvp(args[0], args.data());
    _exit(127);
  }
  setpgid(pid, pid);  // race-free from both sides
  std::lock_guard<std::mutex> lk(mu_);
  live_[pid] = true;
  return pid;
}

bool Supervisor::reap_one(pid_t pid) {
  int status = 0;
  const pid_t r = waitpid(pid, &status, WNOHANG);
  if (r == pid || (r < 0 && errno == ECHILD)) {
    live_.erase(pid);
    exited_.push_back({pid, r == pid ? status : 0});
    return true;
  }
  return false;
}

std::vector<ExitInfo> Supervisor::poll() {
  std::lock_guard<std::mutex> lk(mu_);
  std::vector<pid_t> pids;
  for (const auto& [pid, _] : live_) pids.push_back(pid);
  for (pid_t pid : pids) reap_one(pid);
  std::vector<ExitInfo> out;
  out.swap(exited_);
  return out;
}

void Supervisor::terminate(pid_t pid, double grace_s) {
  if (pid <= 0) return;
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (!live_.count(pid)) return;
  }
  kill(-pid, SIGTERM);
  kill(-pid, SIGCONT);
  auto gone = [&] {
    std::lock_guard<std::mutex> lk(mu_);
    return !live_.count(pid) || reap_one(pid);
  };
  const auto t0 = std::chrono::steady_clock::now();
  while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < grace_s) {
    if (gone()) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  kill(-pid, SIGKILL);
  for (int i = 0; i < 300 && !gone(); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void Supervisor::terminate_all(double grace_s) {
  std::vector<pid_t> pids;
  {
    std::lock_guard<std::mutex> lk(mu_);
    for (const auto& [pid, _] : live_) pids.push_back(pid);
  }
  for (pid_t pid : pids) {
    kill(-pid, SIGTERM);
    kill(-pid, SIGCONT);
  }
  for (pid_t pid : pids) terminate(pid, grace_s);
}

ProcSample read_proc(pid_t pid) {
  ProcSample s;
  std::ifstream f("/proc/" + std::to_string(pid) + "/stat");
  std::string line;
  if (!f || !std::getline(f, line)) return s;
  const auto rp = line.rfind(')');  // comm may contain spaces
  if (rp == std::string::npos) return s;
  std::istringstream rest(line.substr(rp + 2));
  std::vector<std::string> fld;
  for (std::string x; rest >> x;) fld.push_back(x);
  if (fld.size() < 22) return s;
  // fld[0] = field 3 (state); utime = field 14, stime = field 15, rss = field 24 (pages)
  static const double tck = static_cast<double>(sysconf(_SC_CLK_TCK));
  static const double page_mb = static_cast<double>(sysconf(_SC_PAGESIZE)) / (1024.0 * 1024.0);
  s.state = fld[0].empty() ? '?' : fld[0][0];
  s.cpu_s = (std::stod(fld[11]) + std::stod(fld[12])) / tck;
  s.rss_mb = std::stod(fld[21]) * page_mb;
  s.ok = true;
  return s;
}

}  // namespace helix_healer
