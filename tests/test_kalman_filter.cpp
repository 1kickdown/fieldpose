#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "fieldpose/kalman_filter.hpp"

using fieldpose::KalmanFilter;

// 1D robot driving at constant velocity, position measured with noise.
// The filter's position estimate must be closer to the truth than the raw
// measurements, and the velocity estimate must converge to the real one.
TEST(KalmanFilter, ConstantVelocityBeatsRawMeasurements) {
  const double dt = 0.05;
  const double true_v = 20.0;      // inches / second
  const double meas_sigma = 2.0;   // inches
  const int steps = 400;

  Eigen::MatrixXd F(2, 2);
  F << 1, dt, 0, 1;
  Eigen::MatrixXd Q = 1e-3 * Eigen::MatrixXd::Identity(2, 2);
  Eigen::MatrixXd H(1, 2);
  H << 1, 0;
  Eigen::MatrixXd R(1, 1);
  R << meas_sigma * meas_sigma;

  KalmanFilter kf(Eigen::Vector2d(0.0, 0.0), 100.0 * Eigen::MatrixXd::Identity(2, 2));

  std::mt19937 rng(42);
  std::normal_distribution<double> noise(0.0, meas_sigma);

  double raw_sq = 0.0, filt_sq = 0.0;
  const int warmup = 50;
  for (int i = 1; i <= steps; ++i) {
    const double truth = true_v * dt * i;
    const double z = truth + noise(rng);
    kf.predict(F, Q);
    kf.update(Eigen::VectorXd::Constant(1, z), H, R);
    if (i > warmup) {
      raw_sq += (z - truth) * (z - truth);
      const double e = kf.state()(0) - truth;
      filt_sq += e * e;
    }
  }
  const int n = steps - warmup;
  const double raw_rmse = std::sqrt(raw_sq / n);
  const double filt_rmse = std::sqrt(filt_sq / n);

  EXPECT_LT(filt_rmse, 0.6 * raw_rmse);
  EXPECT_NEAR(kf.state()(1), true_v, 1.0);
}

namespace {
KalmanFilter make_2d() {
  return KalmanFilter(Eigen::Vector2d(0.0, 0.0), Eigen::MatrixXd::Identity(2, 2));
}
Eigen::MatrixXd pos_H() {
  Eigen::MatrixXd H(1, 2);
  H << 1, 0;
  return H;
}
}  // namespace

TEST(KalmanFilter, HugeRIgnoresMeasurement) {
  auto kf = make_2d();
  kf.update(Eigen::VectorXd::Constant(1, 100.0), pos_H(),
            Eigen::MatrixXd::Constant(1, 1, 1e12));
  EXPECT_NEAR(kf.state()(0), 0.0, 1e-6);
}

TEST(KalmanFilter, TinyRFollowsMeasurement) {
  auto kf = make_2d();
  kf.update(Eigen::VectorXd::Constant(1, 100.0), pos_H(),
            Eigen::MatrixXd::Constant(1, 1, 1e-12));
  EXPECT_NEAR(kf.state()(0), 100.0, 1e-4);
}

TEST(KalmanFilter, CovarianceStaysSymmetricPositiveSemiDefinite) {
  const double dt = 0.05;
  Eigen::MatrixXd F(2, 2);
  F << 1, dt, 0, 1;
  const Eigen::MatrixXd Q = 1e-3 * Eigen::MatrixXd::Identity(2, 2);
  const Eigen::MatrixXd R = Eigen::MatrixXd::Constant(1, 1, 4.0);
  auto kf = make_2d();
  std::mt19937 rng(1);
  std::normal_distribution<double> noise(0.0, 2.0);
  for (int i = 0; i < 2000; ++i) {
    kf.predict(F, Q);
    kf.update(Eigen::VectorXd::Constant(1, noise(rng)), pos_H(), R);
    const Eigen::MatrixXd& P = kf.covariance();
    ASSERT_LT((P - P.transpose()).norm(), 1e-12);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P);
    ASSERT_GE(es.eigenvalues().minCoeff(), -1e-12);
  }
}

TEST(KalmanFilter, MissingMeasurementIncreasesUncertainty) {
  const double dt = 0.05;
  Eigen::MatrixXd F(2, 2);
  F << 1, dt, 0, 1;
  const Eigen::MatrixXd Q = 1e-2 * Eigen::MatrixXd::Identity(2, 2);
  auto kf = make_2d();
  double prev = kf.covariance().trace();
  for (int i = 0; i < 20; ++i) {
    kf.predict(F, Q);
    EXPECT_GT(kf.covariance().trace(), prev);
    prev = kf.covariance().trace();
  }
}
