#include <gtest/gtest.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <thread>

#include "helix_healer/process.hpp"

using namespace helix_healer;

namespace {
std::vector<ExitInfo> wait_exits(Supervisor& s, double timeout_s) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < end) {
    auto e = s.poll();
    if (!e.empty()) return e;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return {};
}
}  // namespace

TEST(Process, ReportsExitStatus) {
  Supervisor s;
  const pid_t p = s.spawn({"/bin/sh", "-c", "exit 3"}, "");
  ASSERT_GT(p, 0);
  auto e = wait_exits(s, 3.0);
  ASSERT_EQ(e.size(), 1u);
  EXPECT_EQ(e[0].pid, p);
  ASSERT_TRUE(WIFEXITED(e[0].status));
  EXPECT_EQ(WEXITSTATUS(e[0].status), 3);
  EXPECT_EQ(e[0].describe(), "exit 3");
}

TEST(Process, TerminateEscalatesToKill) {
  Supervisor s;
  // Ignores SIGTERM, so only SIGKILL ends it.
  const pid_t p = s.spawn({"/bin/sh", "-c", "trap '' TERM; while :; do sleep 0.05; done"}, "");
  ASSERT_GT(p, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  const auto t0 = std::chrono::steady_clock::now();
  s.terminate(p, 0.2);
  const double took = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  EXPECT_GE(took, 0.19);
  EXPECT_LT(took, 2.0);
  EXPECT_FALSE(read_proc(p).ok);
}

TEST(Process, ExecFailureExits) {
  Supervisor s;
  const pid_t p = s.spawn({"/nonexistent/binary"}, "");
  ASSERT_GT(p, 0);
  auto e = wait_exits(s, 3.0);
  ASSERT_EQ(e.size(), 1u);
  EXPECT_FALSE(WIFEXITED(e[0].status) && WEXITSTATUS(e[0].status) == 0);
}

TEST(Process, ReadsProcSelf) {
  const auto ps = read_proc(getpid());
  ASSERT_TRUE(ps.ok);
  EXPECT_GT(ps.rss_mb, 0.5);
  EXPECT_GE(ps.cpu_s, 0.0);
}

TEST(Process, CoreDumpingIsFalseForALiveOrMissingProcess) {
  EXPECT_FALSE(core_dumping(getpid()));
  EXPECT_FALSE(core_dumping(999999999));
}

TEST(Process, ChildOutlivesTheThreadThatAskedForIt) {
  Supervisor sup;
  pid_t pid = -1;
  std::thread([&] {
    pid = sup.spawn({"sleep", "5"}, "");
    // Outlive the child's prctl() like a real repair job does, then exit.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }).join();
  ASSERT_GT(pid, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_TRUE(sup.poll().empty());  // PDEATHSIG must not have fired when the caller thread exited
  sup.terminate(pid, 0.2);
}

TEST(Process, AdoptsAProcessItDidNotStart) {
  // An outside process (here: started by the shell, not by the Supervisor).
  const std::string tag = "helixadopt" + std::to_string(getpid());
  const pid_t outside = fork();
  ASSERT_GE(outside, 0);
  if (outside == 0) {
    execlp("/bin/bash", "bash", "-c", ("exec -a " + tag + " sleep 30").c_str(), nullptr);
    _exit(127);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  const ProcInfo info = find_process(tag);
  ASSERT_EQ(info.pid, outside);
  EXPECT_EQ(info.matches, 1);
  ASSERT_FALSE(info.argv.empty());
  EXPECT_EQ(info.argv[0], tag);
  EXPECT_FALSE(info.cwd.empty());
  EXPECT_FALSE(info.env.empty());
  EXPECT_LT(info.age_s, 5.0);

  Supervisor s;
  ASSERT_TRUE(s.adopt(outside));
  EXPECT_TRUE(s.poll().empty());
  kill(outside, SIGKILL);
  waitpid(outside, nullptr, 0);  // we are its real parent in this test
  auto e = wait_exits(s, 3.0);
  ASSERT_EQ(e.size(), 1u);
  EXPECT_EQ(e[0].pid, outside);
  EXPECT_TRUE(e[0].adopted);
  EXPECT_NE(e[0].describe().find("adopted"), std::string::npos);
}

TEST(Process, TerminatesAnAdoptedProcess) {
  const pid_t outside = fork();
  ASSERT_GE(outside, 0);
  if (outside == 0) {
    execlp("sleep", "sleep", "30", nullptr);
    _exit(127);
  }
  Supervisor s;
  ASSERT_TRUE(s.adopt(outside));
  kill(outside, SIGSTOP);  // a hung process still dies: SIGTERM is pending, SIGCONT delivers it
  s.terminate(outside, 1.0);
  int st = 0;
  ASSERT_EQ(waitpid(outside, &st, 0), outside);
  EXPECT_TRUE(WIFSIGNALED(st));
  auto e = s.poll();
  ASSERT_EQ(e.size(), 1u);
  EXPECT_TRUE(e[0].adopted);
}

TEST(Process, FindPrefersTheLeafOverItsWrapper) {
  const std::string tag = "helixleaf" + std::to_string(getpid());
  Supervisor s;
  // sh (argv mentions tag) -> sleep with argv[0]=tag: both match, the leaf must win.
  const pid_t wrapper = s.spawn({"/bin/bash", "-c", "(exec -a " + tag + " sleep 30); true " + tag}, "");
  ASSERT_GT(wrapper, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const ProcInfo info = find_process(tag);
  EXPECT_EQ(info.matches, 2);
  EXPECT_NE(info.pid, wrapper);
  ASSERT_FALSE(info.argv.empty());
  EXPECT_EQ(info.argv[0], tag);
  s.terminate(wrapper, 0.5);
}

TEST(Process, SpawnHonoursEnvAndCwd) {
  Supervisor s;
  SpawnOpts o;
  o.set_env = true;
  o.env = {"HELIX_T=42", "PATH=/usr/bin:/bin"};
  o.cwd = "/tmp";
  const pid_t p = s.spawn({"/bin/sh", "-c", "[ \"$HELIX_T\" = 42 ] && [ \"$(pwd)\" = /tmp ] && exit 5; exit 1"}, "", o);
  ASSERT_GT(p, 0);
  auto e = wait_exits(s, 3.0);
  ASSERT_EQ(e.size(), 1u);
  EXPECT_TRUE(WIFEXITED(e[0].status));
  EXPECT_EQ(WEXITSTATUS(e[0].status), 5);
}

TEST(Process, DetachedChildSurvivesTerminateAll) {
  pid_t p;
  {
    Supervisor s;
    SpawnOpts o;
    o.detach = true;
    p = s.spawn({"sleep", "30"}, "", o);
    ASSERT_GT(p, 0);
    s.terminate_all(0.2);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(kill(p, 0), 0);
  kill(p, SIGKILL);
}
