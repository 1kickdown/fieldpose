#include "fieldpose/evaluation.hpp"

#include <cmath>

namespace fieldpose {

ErrorStats compute_stats(const std::vector<Pose2D>& est,
                         const std::vector<Pose2D>& truth, double thr) {
  ErrorStats s;
  double sx = 0, sy = 0, st = 0;
  int below = 0;
  const size_t n = est.size();
  for (size_t i = 0; i < n; ++i) {
    const double ex = est[i].x - truth[i].x, ey = est[i].y - truth[i].y;
    const double et = wrap_angle(est[i].theta - truth[i].theta);
    sx += ex * ex;
    sy += ey * ey;
    st += et * et;
    if (std::hypot(ex, ey) < thr) ++below;
  }
  s.x_rmse = std::sqrt(sx / n);
  s.y_rmse = std::sqrt(sy / n);
  s.pos_rmse = std::sqrt((sx + sy) / n);
  s.theta_rmse_deg = std::sqrt(st / n) * kRadToDeg;
  s.frac_below = static_cast<double>(below) / n;
  s.final_pos_err = distance(est.back(), truth.back());
  return s;
}

std::vector<double> recovery_times(const SimResult& sim, const FusionRun& run,
                                   double thr, double min_gap_s) {
  // Steps at which a tag fix arrived (rejected or not), in time order.
  std::vector<int> fix_steps;
  for (const SimEvent& e : sim.events)
    if (e.type == SimEvent::Type::Tag) fix_steps.push_back(e.step);

  std::vector<double> out;
  const int gap_steps = static_cast<int>(min_gap_s / sim.dt);
  for (size_t j = 1; j < fix_steps.size(); ++j) {
    if (fix_steps[j] - fix_steps[j - 1] < gap_steps) continue;
    int k = fix_steps[j];
    const int n = static_cast<int>(run.estimate.size());
    while (k < n && distance(run.estimate[k], sim.truth[k]) >= thr) ++k;
    out.push_back(k >= n ? (n - fix_steps[j]) * sim.dt : (k - fix_steps[j]) * sim.dt);
  }
  return out;
}

}  // namespace fieldpose
