#include <gtest/gtest.h>
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
