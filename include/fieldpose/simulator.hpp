#pragma once

#include <cstdint>
#include <vector>

#include "fieldpose/geometry.hpp"
#include "fieldpose/trajectory.hpp"

namespace fieldpose {

struct SimConfig {
  double dt = 0.02;  // odometry / IMU period, s (50 Hz)

  // Wheel odometry: body-frame increments with a per-run systematic scale
  // error (calibration) and per-step noise proportional to the step size.
  double odom_scale_sigma = 0.01;   // std of the x/y scale error around 1
  double odom_rot_scale_sigma = 0.01;
  double odom_trans_noise_frac = 0.02;
  double odom_trans_noise_floor = 0.002;  // in per step
  double odom_rot_noise_frac = 0.01;
  double odom_rot_noise_floor = 2e-5;     // rad per step

  // IMU heading: white noise plus a slowly wandering bias.
  double imu_sigma = 0.5 * kDegToRad;
  double imu_bias0_sigma = 0.3 * kDegToRad;
  double imu_bias_walk = 0.02 * kDegToRad;  // random walk, rad / sqrt(s)

  // AprilTag absolute pose: only while the robot is in the backdrop zone
  // (x >= tag_min_x), at tag_rate_hz, with dropouts and gross outliers.
  double tag_rate_hz = 8.0;
  double tag_min_x = 45.0;
  double tag_pos_sigma = 0.8;                  // in
  double tag_theta_sigma = 1.5 * kDegToRad;
  double tag_dropout_prob = 0.15;
  double tag_outlier_prob = 0.03;
  double tag_outlier_min = 8.0;   // in, outlier position offset range
  double tag_outlier_max = 20.0;
  double tag_outlier_theta = 30.0 * kDegToRad;
  bool tag_enabled = true;

  double tail_time = 1.0;  // seconds simulated after the trajectory ends
};

struct SimEvent {
  enum class Type { Odom, Imu, Tag };
  Type type;
  int step;          // index into SimResult::truth
  double t;          // s
  double a = 0, b = 0, c = 0;  // Odom: body dx, dy, dtheta; Imu: a=theta; Tag: x, y, theta
  bool outlier = false;        // Tag only: ground truth label for statistics
};

struct SimResult {
  double dt = 0.0;
  std::vector<SimEvent> events;        // time-ordered
  std::vector<Pose2D> truth;           // pose after step i (index 0 = start)
  std::vector<Pose2D> raw_odometry;    // dead reckoning from odometry only
  std::vector<bool> tag_visible;       // tag zone flag per step
};

SimResult simulate(const Trajectory& traj, const SimConfig& cfg, uint32_t seed);

}  // namespace fieldpose
