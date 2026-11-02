#include <gtest/gtest.h>

#include "helix_healer/spec.hpp"

using namespace helix_healer;

TEST(Spec, ParsesNodesOutputsAndParams) {
  const auto s = parse_spec(R"(
healer tick_hz=50 max_attempts=2   # comment
node cam lifecycle=1 fallback=cam_b cpu_max=0.5
out cam /image 30
exec cam @helix_demo/demo_node --ros-args -p rate_hz:=30.0
node cam_b autostart=0
out cam_b /image
exec cam_b @helix_demo/demo_node
node det critical=0 ros_node=/perception/detector
in det /image
exec det /bin/true
)");
  EXPECT_DOUBLE_EQ(s.p.tick_hz, 50);
  EXPECT_EQ(s.p.max_attempts, 2);
  ASSERT_EQ(s.nodes.size(), 3u);
  const NodeSpec* cam = s.find("cam");
  ASSERT_NE(cam, nullptr);
  EXPECT_TRUE(cam->lifecycle);
  EXPECT_EQ(cam->fallback, "cam_b");
  EXPECT_DOUBLE_EQ(cam->cpu_max, 0.5);
  ASSERT_EQ(cam->outputs.size(), 1u);
  EXPECT_DOUBLE_EQ(cam->outputs[0].hz, 30);
  EXPECT_EQ(cam->argv.size(), 4u);
  EXPECT_DOUBLE_EQ(s.find("cam_b")->outputs[0].hz, 0);
  EXPECT_FALSE(s.find("det")->critical);
  EXPECT_EQ(s.find("det")->ros_node, "/perception/detector");
  EXPECT_TRUE(cam->ros_node.empty());
  EXPECT_EQ(s.find("det")->inputs.at(0), "/image");
}

TEST(Spec, RejectsBadInput) {
  EXPECT_THROW(parse_spec("frobnicate x"), std::runtime_error);
  EXPECT_THROW(parse_spec("node a\n"), std::runtime_error);                       // no exec
  EXPECT_THROW(parse_spec("out a /x\n"), std::runtime_error);                     // before node
  EXPECT_THROW(parse_spec("node a fallback=b\nexec a x\nnode b\nexec b y\n"), std::runtime_error);  // autostart backup
  EXPECT_THROW(parse_spec("node a fallback=zz\nexec a x\n"), std::runtime_error);
  EXPECT_THROW(parse_spec("healer max_attempts=0\n"), std::runtime_error);
  EXPECT_THROW(parse_spec("healer bogus=1\n"), std::runtime_error);
  EXPECT_THROW(parse_spec("node a restart=maybe\nexec a x\n"), std::runtime_error);
  try {
    parse_spec("\n\nnode a cpu_max=lots\n");
    FAIL();
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("line 3"), std::string::npos);
  }
}
