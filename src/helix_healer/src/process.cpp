#include "helix_healer/process.hpp"

#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/syscall.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <future>
#include <sstream>
#include <thread>

namespace helix_healer {

namespace {

int pidfd_open(pid_t pid) { return static_cast<int>(syscall(SYS_pidfd_open, pid, 0)); }
int pidfd_signal(int fd, int sig) { return static_cast<int>(syscall(SYS_pidfd_send_signal, fd, sig, nullptr, 0)); }
bool pidfd_exited(int fd, int timeout_ms) {
  pollfd p{fd, POLLIN, 0};
  return ::poll(&p, 1, timeout_ms) > 0 && (p.revents & (POLLIN | POLLHUP | POLLERR));
}

std::string slurp(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

std::vector<std::string> split_nul(const std::string& s) {
  std::vector<std::string> out;
  std::size_t i = 0;
  while (i < s.size()) {
    const auto j = s.find('\0', i);
    out.push_back(s.substr(i, j == std::string::npos ? std::string::npos : j - i));
    if (j == std::string::npos) break;
    i = j + 1;
  }
  return out;
}

}  // namespace

std::string ExitInfo::describe() const {
  if (adopted) return "exited (adopted process, status unknown)";
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

pid_t Supervisor::spawn(const std::vector<std::string>& argv, const std::string& log_path, const SpawnOpts& opts) {
  if (argv.empty()) return -1;
  if (std::this_thread::get_id() == forker_.get_id()) return do_spawn(argv, log_path, opts);
  std::promise<pid_t> done;
  auto fut = done.get_future();
  {
    std::lock_guard<std::mutex> lk(fork_mu_);
    if (fork_stop_) return -1;
    fork_q_.push_back([&] { done.set_value(do_spawn(argv, log_path, opts)); });
  }
  fork_cv_.notify_one();
  return fut.get();
}

pid_t Supervisor::do_spawn(const std::vector<std::string>& argv, const std::string& log_path, const SpawnOpts& opts) {
  std::vector<char*> args, envp;
  for (const auto& a : argv) args.push_back(const_cast<char*>(a.c_str()));
  args.push_back(nullptr);
  for (const auto& e : opts.env) envp.push_back(const_cast<char*>(e.c_str()));
  envp.push_back(nullptr);
  const pid_t parent = getpid();
  const pid_t pid = fork();
  if (pid < 0) return -1;
  if (pid == 0) {
    setpgid(0, 0);
    if (!opts.detach) {
      prctl(PR_SET_PDEATHSIG, SIGTERM);
      if (getppid() != parent) _exit(127);  // parent died between fork and prctl
    }
    if (!opts.cwd.empty() && chdir(opts.cwd.c_str()) != 0) _exit(127);
    if (!log_path.empty()) {
      const int fd = open(log_path.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
      if (fd >= 0) {
        dup2(fd, STDOUT_FILENO);
        dup2(fd, STDERR_FILENO);
        close(fd);
      }
    }
    if (opts.set_env) execvpe(args[0], args.data(), envp.data());
    else execvp(args[0], args.data());
    _exit(127);
  }
  setpgid(pid, pid);  // race-free from both sides
  std::lock_guard<std::mutex> lk(mu_);
  live_[pid] = opts.detach;
  return pid;
}

bool Supervisor::adopt(pid_t pid) {
  const int fd = pidfd_open(pid);
  if (fd < 0) return false;
  if (pidfd_exited(fd, 0)) {
    close(fd);
    return false;
  }
  std::lock_guard<std::mutex> lk(mu_);
  if (auto it = adopted_.find(pid); it != adopted_.end()) close(it->second);
  adopted_[pid] = fd;
  return true;
}

void Supervisor::terminate_adopted(pid_t pid, int fd, double grace_s) {
  pidfd_signal(fd, SIGTERM);
  pidfd_signal(fd, SIGCONT);
  if (!pidfd_exited(fd, static_cast<int>(grace_s * 1000))) {
    pidfd_signal(fd, SIGKILL);
    pidfd_exited(fd, 3000);
  }
  close(fd);
  std::lock_guard<std::mutex> lk(mu_);
  exited_.push_back({pid, -1, true});
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
  for (auto it = adopted_.begin(); it != adopted_.end();) {
    if (pidfd_exited(it->second, 0)) {
      close(it->second);
      exited_.push_back({it->first, -1, true});
      it = adopted_.erase(it);
    } else {
      ++it;
    }
  }
  std::vector<ExitInfo> out;
  out.swap(exited_);
  return out;
}

void Supervisor::terminate(pid_t pid, double grace_s) {
  if (pid <= 0) return;
  {
    std::unique_lock<std::mutex> lk(mu_);
    if (auto it = adopted_.find(pid); it != adopted_.end()) {
      const int fd = it->second;
      adopted_.erase(it);
      lk.unlock();
      terminate_adopted(pid, fd, grace_s);
      return;
    }
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
    for (const auto& [pid, detached] : live_)
      if (!detached) pids.push_back(pid);
    for (auto& [pid, fd] : adopted_) close(fd);  // adopted processes are not ours to stop
    adopted_.clear();
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

bool core_dumping(pid_t pid) {
  const std::string st = slurp("/proc/" + std::to_string(pid) + "/status");
  const auto k = st.find("CoreDumping:");
  if (k == std::string::npos) return false;
  const auto v = st.find_first_not_of(" \t", k + 12);
  return v != std::string::npos && st[v] == '1';
}

ProcInfo find_process(const std::string& pattern) {
  struct Hit {
    pid_t pid, ppid;
    double start;
  };
  std::vector<Hit> hits;
  ProcInfo out;
  if (pattern.empty()) return out;
  const pid_t self = getpid();
  DIR* d = opendir("/proc");
  if (!d) return out;
  while (dirent* e = readdir(d)) {
    char* end = nullptr;
    const long v = std::strtol(e->d_name, &end, 10);
    if (*end != '\0' || v <= 0 || v == self) continue;
    const pid_t pid = static_cast<pid_t>(v);
    const auto argv = split_nul(slurp("/proc/" + std::to_string(pid) + "/cmdline"));
    if (argv.empty()) continue;  // kernel thread or gone
    std::string joined;
    for (const auto& a : argv) joined += (joined.empty() ? "" : " ") + a;
    if (joined.find(pattern) == std::string::npos || joined.find("helix_healer") != std::string::npos) continue;
    const std::string stat = slurp("/proc/" + std::to_string(pid) + "/stat");
    const auto rp = stat.rfind(')');
    if (rp == std::string::npos) continue;
    std::istringstream rest(stat.substr(rp + 2));
    std::vector<std::string> f;
    for (std::string x; rest >> x;) f.push_back(x);
    if (f.size() < 20 || f[0] == "Z" || f[0] == "X") continue;
    hits.push_back({pid, static_cast<pid_t>(std::stol(f[1])), std::stod(f[19])});  // ppid, starttime
  }
  closedir(d);
  out.matches = static_cast<int>(hits.size());
  const Hit* best = nullptr;
  for (const auto& h : hits) {
    const bool has_child =
        std::any_of(hits.begin(), hits.end(), [&](const Hit& o) { return o.ppid == h.pid; });
    if (!has_child && (!best || h.start > best->start)) best = &h;
  }
  if (!best) return out;
  const std::string base = "/proc/" + std::to_string(best->pid);
  out.pid = best->pid;
  out.argv = split_nul(slurp(base + "/cmdline"));
  out.env = split_nul(slurp(base + "/environ"));
  char buf[4096];
  const ssize_t n = readlink((base + "/cwd").c_str(), buf, sizeof(buf) - 1);
  if (n > 0) out.cwd.assign(buf, static_cast<std::size_t>(n));
  static const double tck = static_cast<double>(sysconf(_SC_CLK_TCK));
  double up = 0;
  std::ifstream("/proc/uptime") >> up;
  out.age_s = std::max(0.0, up - best->start / tck);
  return out;
}

}  // namespace helix_healer
