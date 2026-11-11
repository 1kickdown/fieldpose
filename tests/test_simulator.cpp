#include <gtest/gtest.h>

#include "fieldpose/simulator.hpp"

using namespace fieldpose;

namespace {
SimConfig perfect() {
  SimConfig c;
  c.odom_scale_sigma = c.odom_rot_scale_sigma = 0;
  c.odom_trans_noise_frac = c.odom_trans_noise_floor = 0;
  c.odom_rot_noise_frac = c.odom_rot_noise_floor = 0;
  c.imu_sigma = c.imu_bias0_sigma = c.imu_bias_walk = 0;
  c.tag_enabled = false;
  return c;
}
}  // namespace

TEST(Trajectory, HitsWaypointsAndStopsThere) {
  Trajectory t({{0, 0, 0}, {30, 0, 0}, {30, 20, 1.0}});
  EXPECT_NEAR(t.at(0).x, 0, 1e-9);
  EXPECT_NEAR(t.at(t.duration()).y, 20, 1e-9);
  const Pose2D mid = t.at(t.duration() * 0.5);
  EXPECT_GT(mid.x, 0.0);
}

TEST(Simulator, PerfectSensorsGiveExactDeadReckoning) {
  Trajectory t({{0, 0, 0}, {40, 10, 1.2}, {10, 30, -2.5}});
  const SimResult r = simulate(t, perfect(), 1);
  ASSERT_EQ(r.truth.size(), r.raw_odometry.size());
  for (size_t i = 0; i < r.truth.size(); ++i) {
    EXPECT_NEAR(r.raw_odometry[i].x, r.truth[i].x, 1e-6);
    EXPECT_NEAR(r.raw_odometry[i].y, r.truth[i].y, 1e-6);
  }
}

TEST(Simulator, SameSeedIsDeterministicAndOdometryDrifts) {
  Trajectory t({{0, 0, 0}, {80, 0, 0}, {80, 60, 1.5}});
  SimConfig c;
  const SimResult a = simulate(t, c, 7), b = simulate(t, c, 7);
  EXPECT_EQ(a.events.size(), b.events.size());
  EXPECT_DOUBLE_EQ(a.raw_odometry.back().x, b.raw_odometry.back().x);
  EXPECT_GT(distance(a.raw_odometry.back(), a.truth.back()), 0.1);
}

TEST(Simulator, TagsOnlyInTagZone) {
  Trajectory t({{0, 0, 0}, {60, 0, 0}});
  const SimResult r = simulate(t, SimConfig{}, 3);
  int tags = 0;
  for (const auto& e : r.events)
    if (e.type == SimEvent::Type::Tag) {
      ++tags;
      EXPECT_GE(r.truth[e.step].x, SimConfig{}.tag_min_x);
    }
  EXPECT_GT(tags, 0);
}
