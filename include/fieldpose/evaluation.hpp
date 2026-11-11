#pragma once

#include <vector>

#include "fieldpose/pose_fusion.hpp"
#include "fieldpose/simulator.hpp"

namespace fieldpose {

struct ErrorStats {
  double pos_rmse = 0.0;       // in (Euclidean)
  double x_rmse = 0.0;
  double y_rmse = 0.0;
  double theta_rmse_deg = 0.0;
  double frac_below = 0.0;     // share of steps with position error < threshold
  double final_pos_err = 0.0;  // in, at the last step
};

ErrorStats compute_stats(const std::vector<Pose2D>& est,
                         const std::vector<Pose2D>& truth, double threshold_in);

// For each tag blackout of at least `min_gap_s` (gap between consecutive tag
// measurements), the time (s) after the tag returns until the position error
// is under `threshold_in`. 0 means it was already below the threshold.
std::vector<double> recovery_times(const SimResult& sim, const FusionRun& run,
                                   double threshold_in, double min_gap_s);

}  // namespace fieldpose
