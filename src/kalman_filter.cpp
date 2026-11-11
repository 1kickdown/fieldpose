#include "fieldpose/kalman_filter.hpp"

namespace fieldpose {

KalmanFilter::KalmanFilter(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0)
    : x_(x0), P_(P0) {}

void KalmanFilter::predict(const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q) {
  x_ = F * x_;
  P_ = F * P_ * F.transpose() + Q;
}

void KalmanFilter::update(const Eigen::VectorXd& z, const Eigen::MatrixXd& H,
                          const Eigen::MatrixXd& R) {
  const Eigen::VectorXd y = z - H * x_;
  const Eigen::MatrixXd S = H * P_ * H.transpose() + R;
  // K = P H^T S^-1, computed by solving S K^T = H P (S is symmetric).
  const Eigen::MatrixXd K = S.ldlt().solve(H * P_).transpose();
  x_ += K * y;
  // Joseph form keeps P symmetric and positive semi-definite under rounding.
  const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(P_.rows(), P_.cols());
  const Eigen::MatrixXd A = I - K * H;
  P_ = A * P_ * A.transpose() + K * R * K.transpose();
  P_ = 0.5 * (P_ + P_.transpose());
}

}  // namespace fieldpose
