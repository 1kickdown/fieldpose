#pragma once

#include <Eigen/Dense>

namespace fieldpose {

// Linear Kalman filter with a dynamically sized state.
//
// State estimate x (n x 1) and covariance P (n x n).
//   predict: x = F x,  P = F P F^T + Q
//   update:  y = z - H x,  S = H P H^T + R,  K = P H^T S^-1
//            x = x + K y
//            P = (I-KH) P (I-KH)^T + K R K^T   (Joseph form)
class KalmanFilter {
 public:
  KalmanFilter(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);

  void predict(const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q);
  // Predict with an additive control term: x = F x + u.
  void predict(const Eigen::MatrixXd& F, const Eigen::VectorXd& u,
               const Eigen::MatrixXd& Q);
  void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& H,
              const Eigen::MatrixXd& R);

  // Squared Mahalanobis distance of the innovation, y^T S^-1 y.
  // Compare against a chi-square quantile to gate outliers.
  double mahalanobis2(const Eigen::VectorXd& z, const Eigen::MatrixXd& H,
                      const Eigen::MatrixXd& R) const;

  const Eigen::VectorXd& state() const { return x_; }
  const Eigen::MatrixXd& covariance() const { return P_; }

 private:
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;
};

}  // namespace fieldpose
