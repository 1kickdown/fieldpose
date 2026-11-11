// Extracts waypoint numbers from team match logs into data/trajectories/*.csv.
// Only numbers are written; no team code is copied.
#include <filesystem>
#include <iostream>

#include "fieldpose/log_loader.hpp"

namespace fs = std::filesystem;

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: extract_trajectories <log_dir> <out_dir>\n";
    return 2;
  }
  fs::create_directories(argv[2]);
  for (const auto& entry : fs::directory_iterator(argv[1])) {
    if (entry.path().extension() != ".log") continue;
    const auto log = fieldpose::load_log(entry.path());
    const auto path = fieldpose::join_paths(log);
    if (path.size() < 2) continue;
    // File names look like 20231105@123513!QUALIFICATION03_Auto.log
    std::string name = entry.path().stem().string();
    name = name.substr(name.find('!') + 1);
    fieldpose::write_waypoints_csv(fs::path(argv[2]) / (name + ".csv"), path);
    std::cout << name << ": " << path.size() << " waypoints, "
              << log.tag_fixes.size() << " tag fixes, "
              << log.robot_poses.size() << " robot poses\n";
  }
}
