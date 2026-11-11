#pragma once

#include <vector>

#include "fieldpose/geometry.hpp"

namespace fieldpose {

// Time-parameterised path through waypoints. Each segment starts and ends at
// rest (smoothstep easing), like the waypoint-to-waypoint moves in the logs.
class Trajectory {
 public:
  // `cruise_speed` in in/s and `cruise_omega` in rad/s set the segment
  // durations; the slower of the translation and rotation limits wins.
  Trajectory(std::vector<Pose2D> waypoints, double cruise_speed = 25.0,
             double cruise_omega = 2.0);

  double duration() const { return total_; }
  Pose2D at(double t) const;
  const std::vector<Pose2D>& waypoints() const { return wps_; }

  // Synthetic trajectories for tests and extra scenarios.
  static Trajectory line(Pose2D from, Pose2D to);
  static Trajectory arc(double cx, double cy, double radius, double start_angle,
                        double sweep, int segments = 24);

 private:
  std::vector<Pose2D> wps_;
  std::vector<double> t_end_;  // cumulative end time of each segment
  double total_ = 0.0;
};

}  // namespace fieldpose
