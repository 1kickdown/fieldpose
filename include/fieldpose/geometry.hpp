#pragma once

#include <cmath>
#include <vector>

namespace fieldpose {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

// FTC field is 144 x 144 in nominal; the team's RobotParams uses 141.24 in
// between the tile-mat edges. Coordinates are inches with the origin at the
// field centre, angles in radians.
constexpr double kFieldInches = 141.24;

// Wrap an angle to (-pi, pi].
inline double wrap_angle(double a) {
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a <= 0.0) a += 2.0 * kPi;
  return a - kPi;
}

struct Pose2D {
  double x = 0.0;      // inches
  double y = 0.0;      // inches
  double theta = 0.0;  // radians
};

inline double distance(const Pose2D& a, const Pose2D& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

}  // namespace fieldpose
