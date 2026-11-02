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
  int status = 0;        // raw waitpid status
  bool adopted = false;  // not our child: the exit status is unknown
  std::string describe() const;  // "exit 1" / "signal 9 (Killed)"
};

struct SpawnOpts {
  bool set_env = false;          // replace the environment with `env`
  std::vector<std::string> env;  // KEY=VALUE
  std::string cwd;               // empty = inherit
  bool detach = false;           // no PDEATHSIG, and terminate_all() leaves it running
};

// A running process found by its command line, with what is needed to start it again.
struct ProcInfo {
  pid_t pid = -1;
  std::vector<std::string> argv;
  std::vector<std::string> env;  // empty when /proc/PID/environ is unreadable
  std::string cwd;
  double age_s = 0;
  int matches = 0;  // how many processes matched; >1 means the pattern is ambiguous
};
// Finds a process whose argv, joined by spaces, contains `pattern`. A match that has a
// matching child is skipped (so `ros2 run pkg exe` resolves to exe, not the python
// wrapper); among the rest the newest wins. Skips this process, zombies, and any command
// line that mentions helix_healer.
ProcInfo find_process(const std::string& pattern);

class Supervisor {
 public:
  Supervisor();
  ~Supervisor();
  // fork+exec in a new process group; the child dies with the supervisor (PDEATHSIG).
  // Thread-safe. All forks happen on one long-lived thread because PDEATHSIG fires when
  // the forking *thread* exits, and callers are short-lived repair jobs.
  // stdout/stderr go to log_path when it is non-empty. Returns -1 on failure.
  pid_t spawn(const std::vector<std::string>& argv, const std::string& log_path, const SpawnOpts& opts = {});
  // Track a process we did not start, through a pidfd. poll() reports its exit with
  // adopted=true, and terminate() signals the pid only (not its group). False if it is gone.
  bool adopt(pid_t pid);
  // SIGTERM (+SIGCONT, so stopped processes see it) to the group, SIGKILL after grace_s. Blocks until reaped.
  void terminate(pid_t pid, double grace_s);
  // Non-blocking reap of every child; returns the ones that exited since the last call.
  std::vector<ExitInfo> poll();
  void terminate_all(double grace_s);

 private:
  pid_t do_spawn(const std::vector<std::string>& argv, const std::string& log_path, const SpawnOpts& opts);
  bool reap_one(pid_t pid);  // caller holds mu_
  void terminate_adopted(pid_t pid, int fd, double grace_s);
  std::mutex fork_mu_;
  std::condition_variable fork_cv_;
  std::deque<std::function<void()>> fork_q_;
  bool fork_stop_ = false;
  std::thread forker_;
  std::mutex mu_;
  std::map<pid_t, bool> live_;  // our children: pid -> detached
  std::map<pid_t, int> adopted_;  // pid -> pidfd
  std::vector<ExitInfo> exited_;
};

struct ProcSample {
  bool ok = false;
  char state = '?';
  double cpu_s = 0;  // utime + stime, all threads
  double rss_mb = 0;
};
ProcSample read_proc(pid_t pid);
// True while the process is writing a core dump (/proc/PID/status CoreDumping: 1, Linux >= 4.15).
bool core_dumping(pid_t pid);

}  // namespace helix_healer
