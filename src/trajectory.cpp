#include "fieldpose/trajectory.hpp"

#include <algorithm>
#include <stdexcept>

namespace fieldpose {

Trajectory::Trajectory(std::vector<Pose2D> waypoints, double cruise_speed,
                       double cruise_omega)
    : wps_(std::move(waypoints)) {
  if (wps_.size() < 2) throw std::invalid_argument("need >= 2 waypoints");
  double t = 0.0;
  for (size_t i = 1; i < wps_.size(); ++i) {
    const double d = distance(wps_[i - 1], wps_[i]);
    const double dth = std::abs(wrap_angle(wps_[i].theta - wps_[i - 1].theta));
    // Smoothstep peaks at 1.5x the mean speed, so scale durations by 1.5.
    const double seg = 1.5 * std::max(d / cruise_speed, dth / cruise_omega);
    t += std::max(seg, 0.05);
    t_end_.push_back(t);
  }
  total_ = t;
}

Pose2D Trajectory::at(double t) const {
  if (t <= 0.0) return wps_.front();
  if (t >= total_) return wps_.back();
  const size_t i = std::upper_bound(t_end_.begin(), t_end_.end(), t) - t_end_.begin();
  const double t0 = i == 0 ? 0.0 : t_end_[i - 1];
  const double u = (t - t0) / (t_end_[i] - t0);
  const double s = u * u * (3.0 - 2.0 * u);
  const Pose2D& a = wps_[i];
  const Pose2D& b = wps_[i + 1];
  Pose2D p;
  p.x = a.x + s * (b.x - a.x);
  p.y = a.y + s * (b.y - a.y);
  p.theta = a.theta + s * wrap_angle(b.theta - a.theta);
  return p;
}

Trajectory Trajectory::line(Pose2D from, Pose2D to) {
  return Trajectory({from, to});
}

Trajectory Trajectory::arc(double cx, double cy, double radius,
                           double start_angle, double sweep, int segments) {
  std::vector<Pose2D> w;
  for (int i = 0; i <= segments; ++i) {
    const double a = start_angle + sweep * i / segments;
    const double tangent = a + (sweep >= 0 ? kPi / 2 : -kPi / 2);
    w.push_back({cx + radius * std::cos(a), cy + radius * std::sin(a),
                 wrap_angle(tangent)});
  }
  return Trajectory(std::move(w));
}

}  // namespace fieldpose
