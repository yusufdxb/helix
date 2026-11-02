// Child-process supervision and /proc sampling. POSIX only, no ROS.
#pragma once

#include <sys/types.h>

#include <condition_variable>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace helix_healer {

struct ExitInfo {
  pid_t pid = -1;
  int status = 0;  // raw waitpid status
  std::string describe() const;  // "exit 1" / "signal 9 (Killed)"
};

class Supervisor {
 public:
  Supervisor();
  ~Supervisor();
  // fork+exec in a new process group; the child dies with the supervisor (PDEATHSIG).
  // Thread-safe. All forks happen on one long-lived thread because PDEATHSIG fires when
  // the forking *thread* exits, and callers are short-lived repair jobs.
  // stdout/stderr go to log_path when it is non-empty. Returns -1 on failure.
  pid_t spawn(const std::vector<std::string>& argv, const std::string& log_path);
  // SIGTERM (+SIGCONT, so stopped processes see it) to the group, SIGKILL after grace_s. Blocks until reaped.
  void terminate(pid_t pid, double grace_s);
  // Non-blocking reap of every child; returns the ones that exited since the last call.
  std::vector<ExitInfo> poll();
  void terminate_all(double grace_s);

 private:
  pid_t do_spawn(const std::vector<std::string>& argv, const std::string& log_path);
  bool reap_one(pid_t pid);  // caller holds mu_
  std::mutex fork_mu_;
  std::condition_variable fork_cv_;
  std::deque<std::function<void()>> fork_q_;
  bool fork_stop_ = false;
  std::thread forker_;
  std::mutex mu_;
  std::map<pid_t, bool> live_;  // pid -> alive
  std::vector<ExitInfo> exited_;
};

struct ProcSample {
  bool ok = false;
  char state = '?';
  double cpu_s = 0;  // utime + stime, all threads
  double rss_mb = 0;
};
ProcSample read_proc(pid_t pid);

}  // namespace helix_healer
