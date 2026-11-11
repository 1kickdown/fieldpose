#include "fieldpose/simulator.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace fieldpose {

SimResult simulate(const Trajectory& traj, const SimConfig& cfg, uint32_t seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> n01(0.0, 1.0);
  std::uniform_real_distribution<double> u01(0.0, 1.0);
  auto gauss = [&](double sigma) { return sigma * n01(rng); };

  const double scale_x = 1.0 + gauss(cfg.odom_scale_sigma);
  const double scale_y = 1.0 + gauss(cfg.odom_scale_sigma);
  const double scale_th = 1.0 + gauss(cfg.odom_rot_scale_sigma);
  double imu_bias = gauss(cfg.imu_bias0_sigma);

  SimResult res;
  res.dt = cfg.dt;
  const int steps =
      static_cast<int>(std::ceil((traj.duration() + cfg.tail_time) / cfg.dt));
  Pose2D prev = traj.at(0.0);
  res.truth.push_back(prev);
  res.raw_odometry.push_back(prev);
  res.tag_visible.push_back(false);
  Pose2D raw = prev;
  double next_tag = 0.0;

  for (int k = 1; k <= steps; ++k) {
    const double t = k * cfg.dt;
    const Pose2D cur = traj.at(t);
    const double dth = wrap_angle(cur.theta - prev.theta);
    const double mid = prev.theta + 0.5 * dth;
    const double dxf = cur.x - prev.x, dyf = cur.y - prev.y;
    const double c = std::cos(mid), s = std::sin(mid);
    const double dxb = c * dxf + s * dyf;   // rotate field -> body
    const double dyb = -s * dxf + c * dyf;
    const double d = std::hypot(dxf, dyf);

    // Wheel odometry increment as measured.
    const double sig_t = cfg.odom_trans_noise_frac * d + cfg.odom_trans_noise_floor;
    const double sig_r = cfg.odom_rot_noise_frac * std::abs(dth) + cfg.odom_rot_noise_floor;
    const double mdx = scale_x * dxb + gauss(sig_t);
    const double mdy = scale_y * dyb + gauss(sig_t);
    const double mdth = scale_th * dth + gauss(sig_r);
    res.events.push_back({SimEvent::Type::Odom, k, t, mdx, mdy, mdth, false});

    // Dead reckoning from the same increments.
    const double rmid = raw.theta + 0.5 * mdth;
    raw.x += std::cos(rmid) * mdx - std::sin(rmid) * mdy;
    raw.y += std::sin(rmid) * mdx + std::cos(rmid) * mdy;
    raw.theta += mdth;

    // IMU heading.
    imu_bias += gauss(cfg.imu_bias_walk * std::sqrt(cfg.dt));
    res.events.push_back({SimEvent::Type::Imu, k, t,
                          cur.theta + imu_bias + gauss(cfg.imu_sigma), 0, 0, false});

    // AprilTag fix.
    const bool visible = cfg.tag_enabled && cur.x >= cfg.tag_min_x;
    if (visible && t >= next_tag) {
      next_tag = t + 1.0 / cfg.tag_rate_hz;
      if (u01(rng) >= cfg.tag_dropout_prob) {
        SimEvent e{SimEvent::Type::Tag, k, t, 0, 0, 0, false};
        if (u01(rng) < cfg.tag_outlier_prob) {
          const double r = cfg.tag_outlier_min +
                           u01(rng) * (cfg.tag_outlier_max - cfg.tag_outlier_min);
          const double ang = 2.0 * kPi * u01(rng);
          e.a = cur.x + r * std::cos(ang);
          e.b = cur.y + r * std::sin(ang);
          e.c = cur.theta + (2.0 * u01(rng) - 1.0) * cfg.tag_outlier_theta;
          e.outlier = true;
        } else {
          e.a = cur.x + gauss(cfg.tag_pos_sigma);
          e.b = cur.y + gauss(cfg.tag_pos_sigma);
          e.c = cur.theta + gauss(cfg.tag_theta_sigma);
        }
        res.events.push_back(e);
      }
    }

    res.truth.push_back(cur);
    res.raw_odometry.push_back(raw);
    res.tag_visible.push_back(visible);
    prev = cur;
  }
  return res;
}

}  // namespace fieldpose
