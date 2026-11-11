#pragma once

#include <filesystem>
#include <iosfwd>
#include <string>
#include <vector>

#include "fieldpose/geometry.hpp"

namespace fieldpose {

struct AprilTagFix {
  int tag_id = 0;
  Pose2D abs_pose;  // field-frame pose reported by the vision code
};

// Numbers extracted from a team match log. Positions are inches and angles
// are radians (the logs use degrees).
struct LogData {
  std::vector<Pose2D> robot_poses;               // RobotPose=(...) samples
  std::vector<std::vector<Pose2D>> start_paths;  // absolute-frame paths
  std::vector<AprilTagFix> tag_fixes;
};

LogData parse_log(std::istream& in);
LogData load_log(const std::filesystem::path& file);

// All start paths of a match joined into one polyline, dropping consecutive
// near-duplicate points.
std::vector<Pose2D> join_paths(const LogData& log);

// CSV with header `x_in,y_in,angle_deg`.
void write_waypoints_csv(const std::filesystem::path& file,
                         const std::vector<Pose2D>& wps);
std::vector<Pose2D> read_waypoints_csv(const std::filesystem::path& file);

}  // namespace fieldpose
