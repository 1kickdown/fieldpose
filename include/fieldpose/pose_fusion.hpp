#pragma once

#include "fieldpose/geometry.hpp"
#include "fieldpose/kalman_filter.hpp"
#include "fieldpose/simulator.hpp"

namespace fieldpose {

struct FusionConfig {
  // Process noise from odometry: std = frac * step + floor.
  double odom_trans_frac = 0.03;
  double odom_trans_floor = 0.01;  // in
  double odom_rot_frac = 0.02;
  double odom_rot_floor = 1e-4;    // rad

  double imu_sigma = 0.5 * kDegToRad;
  double tag_pos_sigma = 0.8;
  double tag_theta_sigma = 1.5 * kDegToRad;

  // Initial uncertainty (start pose is known to this precision).
  double init_pos_sigma = 0.5;
  double init_theta_sigma = 1.0 * kDegToRad;

  // Outlier gating on squared Mahalanobis distance. Defaults are the 99.9%
  // chi-square quantiles for 3 dof (tag) and 1 dof (IMU).
  bool gating = true;
  double tag_gate = 16.27;
  double imu_gate = 10.83;
};

// State [x, y, theta]. Odometry drives the prediction (its increments are
// rotated into the field frame with the current heading estimate), while the
// IMU heading and AprilTag poses are measurement updates that may arrive at
// any time and at different rates.
class PoseFusion {
 public:
  PoseFusion(const Pose2D& start, const FusionConfig& cfg);

  void predictOdometry(double dx_body, double dy_body, double dtheta);
  // Return false if the measurement was rejected by the gate.
  bool updateImu(double theta);
  bool updateTag(double x, double y, double theta);

  Pose2D pose() const;
  const Eigen::MatrixXd& covariance() const { return kf_.covariance(); }

 private:
  FusionConfig cfg_;
  KalmanFilter kf_;
};

// Feed a simulation's events through the filter; returns the estimate after
// every step (index 0 = start). `accepted_tags` / `rejected_tags` count the
// tag fixes the gate let through or dropped.
struct FusionRun {
  std::vector<Pose2D> estimate;
  int accepted_tags = 0;
  int rejected_tags = 0;
  int rejected_outliers = 0;  // rejected AND truly an outlier
  int accepted_outliers = 0;  // accepted although truly an outlier
};

FusionRun run_fusion(const SimResult& sim, const FusionConfig& cfg);

}  // namespace fieldpose
