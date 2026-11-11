#include "fieldpose/pose_fusion.hpp"

#include <cmath>

namespace fieldpose {
namespace {

Eigen::MatrixXd diag3(double a, double b, double c) {
  Eigen::MatrixXd m = Eigen::MatrixXd::Zero(3, 3);
  m(0, 0) = a;
  m(1, 1) = b;
  m(2, 2) = c;
  return m;
}

}  // namespace

PoseFusion::PoseFusion(const Pose2D& start, const FusionConfig& cfg)
    : cfg_(cfg),
      kf_(Eigen::Vector3d(start.x, start.y, start.theta),
          diag3(cfg.init_pos_sigma * cfg.init_pos_sigma,
                cfg.init_pos_sigma * cfg.init_pos_sigma,
                cfg.init_theta_sigma * cfg.init_theta_sigma)) {}

void PoseFusion::predictOdometry(double dx, double dy, double dth) {
  const double mid = kf_.state()(2) + 0.5 * dth;
  const double c = std::cos(mid), s = std::sin(mid);
  Eigen::Vector3d u(c * dx - s * dy, s * dx + c * dy, dth);
  const double d = std::hypot(dx, dy);
  const double st = cfg_.odom_trans_frac * d + cfg_.odom_trans_floor;
  const double sr = cfg_.odom_rot_frac * std::abs(dth) + cfg_.odom_rot_floor;
  // The floor keeps Q positive even when the robot stands still.
  kf_.predict(Eigen::MatrixXd::Identity(3, 3), u, diag3(st * st, st * st, sr * sr));
}

bool PoseFusion::updateImu(double theta) {
  Eigen::MatrixXd H(1, 3);
  H << 0, 0, 1;
  const Eigen::MatrixXd R = Eigen::MatrixXd::Constant(1, 1, cfg_.imu_sigma * cfg_.imu_sigma);
  // Unwrap the measurement next to the (unwrapped) state heading.
  const double th = kf_.state()(2);
  const Eigen::VectorXd z = Eigen::VectorXd::Constant(1, th + wrap_angle(theta - th));
  if (cfg_.gating && kf_.mahalanobis2(z, H, R) > cfg_.imu_gate) return false;
  kf_.update(z, H, R);
  return true;
}

bool PoseFusion::updateTag(double x, double y, double theta) {
  const Eigen::MatrixXd H = Eigen::MatrixXd::Identity(3, 3);
  const double ps = cfg_.tag_pos_sigma * cfg_.tag_pos_sigma;
  const Eigen::MatrixXd R = diag3(ps, ps, cfg_.tag_theta_sigma * cfg_.tag_theta_sigma);
  const double th = kf_.state()(2);
  const Eigen::Vector3d z(x, y, th + wrap_angle(theta - th));
  if (cfg_.gating && kf_.mahalanobis2(z, H, R) > cfg_.tag_gate) return false;
  kf_.update(z, H, R);
  return true;
}

Pose2D PoseFusion::pose() const {
  return {kf_.state()(0), kf_.state()(1), wrap_angle(kf_.state()(2))};
}

FusionRun run_fusion(const SimResult& sim, const FusionConfig& cfg) {
  FusionRun out;
  PoseFusion f(sim.truth.front(), cfg);
  out.estimate.push_back(f.pose());
  size_t i = 0;
  for (int k = 1; k < static_cast<int>(sim.truth.size()); ++k) {
    for (; i < sim.events.size() && sim.events[i].step == k; ++i) {
      const SimEvent& e = sim.events[i];
      switch (e.type) {
        case SimEvent::Type::Odom: f.predictOdometry(e.a, e.b, e.c); break;
        case SimEvent::Type::Imu: f.updateImu(e.a); break;
        case SimEvent::Type::Tag: {
          const bool ok = f.updateTag(e.a, e.b, e.c);
          (ok ? out.accepted_tags : out.rejected_tags)++;
          if (e.outlier) (ok ? out.accepted_outliers : out.rejected_outliers)++;
          break;
        }
      }
    }
    out.estimate.push_back(f.pose());
  }
  return out;
}

}  // namespace fieldpose
