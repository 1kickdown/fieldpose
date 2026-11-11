#include <gtest/gtest.h>

#include "fieldpose/evaluation.hpp"
#include "fieldpose/pose_fusion.hpp"

using namespace fieldpose;

namespace {
Trajectory test_traj() {
  return Trajectory({{10, -60, 0}, {10, -35, 1.57}, {55, -35, 1.57}, {55, -58, 3.14}});
}
}  // namespace

TEST(PoseFusion, BeatsRawOdometry) {
  double raw_sum = 0, filt_sum = 0;
  for (uint32_t seed = 1; seed <= 20; ++seed) {
    const SimResult sim = simulate(test_traj(), SimConfig{}, seed);
    const FusionRun run = run_fusion(sim, FusionConfig{});
    raw_sum += compute_stats(sim.raw_odometry, sim.truth, 2.0).pos_rmse;
    filt_sum += compute_stats(run.estimate, sim.truth, 2.0).pos_rmse;
  }
  EXPECT_LT(filt_sum, 0.7 * raw_sum);
}

TEST(PoseFusion, GateRejectsGrossOutlier) {
  PoseFusion f({0, 0, 0}, FusionConfig{});
  EXPECT_FALSE(f.updateTag(25.0, 0.0, 0.0));
  EXPECT_NEAR(f.pose().x, 0.0, 1e-9);
  EXPECT_TRUE(f.updateTag(0.5, -0.3, 0.01));
}

TEST(PoseFusion, WithoutGateOutlierIsAccepted) {
  FusionConfig cfg;
  cfg.gating = false;
  PoseFusion f({0, 0, 0}, cfg);
  EXPECT_TRUE(f.updateTag(25.0, 0.0, 0.0));
  EXPECT_GT(f.pose().x, 5.0);
}

TEST(PoseFusion, HeadingWrapAroundIsHandled) {
  PoseFusion f({0, 0, 3.10}, FusionConfig{});
  // Measurement just across the +/-pi seam must not look like a 6 rad error.
  EXPECT_TRUE(f.updateImu(-3.12));
  EXPECT_NEAR(std::cos(f.pose().theta), std::cos(3.11), 0.02);
}
