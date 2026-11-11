#include <gtest/gtest.h>

#include <sstream>

#include "fieldpose/log_loader.hpp"

using namespace fieldpose;

TEST(LogLoader, ParsesPathsPosesAndTags) {
  std::stringstream ss;
  ss << "globalTracer.start_Info: Path=TrcPath(degrees=true, "
        "[TrcWaypoint(pose=(x=11.8,y=61.6,angle=180.0),vel=0.00), "
        "TrcWaypoint(pose=(x=11.8,y=35.3,angle=270.0),vel=0.00)])\n"
     << "x StateInfo RobotPose=(x=12.4,y=34.1,angle=267.8) TargetPose=(x=1,y=2,angle=3) "
        "Path=TrcPath(degrees=true, [TrcWaypoint(pose=(x=0.0,y=0.0,angle=0.0),vel=0.00)]) />\n"
     << "globalTracer.CmdAuto_Info: AprilTag 3 found at (x=6.3,y=23.2,z=-3.7,yaw=3.2,pitch=-32.0,roll=1.1) "
        "(absPose=(x=56.5,y=29.3,angle=632.8))\n";
  const LogData log = parse_log(ss);
  ASSERT_EQ(log.start_paths.size(), 1u);
  ASSERT_EQ(log.start_paths[0].size(), 2u);
  EXPECT_NEAR(log.start_paths[0][0].y, 61.6, 1e-9);
  EXPECT_NEAR(log.start_paths[0][1].theta, wrap_angle(270.0 * kDegToRad), 1e-9);
  ASSERT_EQ(log.robot_poses.size(), 1u);
  EXPECT_NEAR(log.robot_poses[0].x, 12.4, 1e-9);
  ASSERT_EQ(log.tag_fixes.size(), 1u);
  EXPECT_EQ(log.tag_fixes[0].tag_id, 3);
  // 632.8 deg is wrapped into (-180, 180]: 632.8 - 720 = -87.2
  EXPECT_NEAR(log.tag_fixes[0].abs_pose.theta * kRadToDeg, -87.2, 1e-6);
}

TEST(LogLoader, JoinDropsDuplicates) {
  LogData log;
  log.start_paths = {{{0, 0, 0}, {10, 0, 0}}, {{10.1, 0, 0}, {10, 10, 0}}};
  EXPECT_EQ(join_paths(log).size(), 3u);
}
