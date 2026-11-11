#include "fieldpose/log_loader.hpp"

#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace fieldpose {
namespace {

const char* kNum = R"(([-+]?[0-9]*\.?[0-9]+))";

Pose2D make_pose(const std::smatch& m, size_t i) {
  return {std::stod(m[i]), std::stod(m[i + 1]),
          wrap_angle(std::stod(m[i + 2]) * kDegToRad)};
}

}  // namespace

LogData parse_log(std::istream& in) {
  const std::string n = kNum;
  const std::regex robot_re("RobotPose=\\(x=" + n + ",y=" + n + ",angle=" + n + "\\)");
  const std::regex wp_re("TrcWaypoint\\(pose=\\(x=" + n + ",y=" + n +
                         ",angle=" + n + "\\)");
  const std::regex tag_re("AprilTag ([0-9]+) found at .*absPose=\\(x=" + n +
                          ",y=" + n + ",angle=" + n + "\\)");
  LogData out;
  std::string line;
  while (std::getline(in, line)) {
    std::smatch m;
    // Only start_Info paths are in the absolute field frame; the paths inside
    // StateInfo events are relative to the robot and are ignored.
    if (line.find("start_Info: Path=TrcPath") != std::string::npos) {
      std::vector<Pose2D> path;
      for (auto it = std::sregex_iterator(line.begin(), line.end(), wp_re);
           it != std::sregex_iterator(); ++it)
        path.push_back(make_pose(*it, 1));
      if (!path.empty()) out.start_paths.push_back(std::move(path));
    } else if (std::regex_search(line, m, tag_re)) {
      out.tag_fixes.push_back({std::stoi(m[1]), make_pose(m, 2)});
    } else if (std::regex_search(line, m, robot_re)) {
      out.robot_poses.push_back(make_pose(m, 1));
    }
  }
  return out;
}

LogData load_log(const std::filesystem::path& file) {
  std::ifstream in(file);
  if (!in) throw std::runtime_error("cannot open " + file.string());
  return parse_log(in);
}

std::vector<Pose2D> join_paths(const LogData& log) {
  std::vector<Pose2D> out;
  for (const auto& path : log.start_paths)
    for (const auto& p : path)
      if (out.empty() || distance(out.back(), p) > 0.5 ||
          std::abs(wrap_angle(out.back().theta - p.theta)) > 0.05)
        out.push_back(p);
  return out;
}

void write_waypoints_csv(const std::filesystem::path& file,
                         const std::vector<Pose2D>& wps) {
  std::ofstream out(file);
  if (!out) throw std::runtime_error("cannot write " + file.string());
  out << "x_in,y_in,angle_deg\n" << std::fixed << std::setprecision(2);
  for (const auto& p : wps)
    out << p.x << ',' << p.y << ',' << p.theta * kRadToDeg << '\n';
}

std::vector<Pose2D> read_waypoints_csv(const std::filesystem::path& file) {
  std::ifstream in(file);
  if (!in) throw std::runtime_error("cannot open " + file.string());
  std::string line;
  std::getline(in, line);  // header
  std::vector<Pose2D> out;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::string a, b, c;
    std::getline(ss, a, ',');
    std::getline(ss, b, ',');
    std::getline(ss, c, ',');
    out.push_back({std::stod(a), std::stod(b), wrap_angle(std::stod(c) * kDegToRad)});
  }
  return out;
}

}  // namespace fieldpose
