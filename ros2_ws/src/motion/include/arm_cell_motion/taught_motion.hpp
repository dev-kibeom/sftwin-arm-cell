#pragma once

#include <map>
#include <array>
#include <string>
#include <vector>

namespace arm_cell_motion
{
struct CartesianTeachingWaypoint
{
  std::array<double, 7> pose;
  bool linear{false};
};

struct TaughtMotion
{
  std::vector<double> joint_positions;
  double velocity_scale;
  double acceleration_scale;
  std::vector<std::vector<double>> joint_path{};
  double duration_s{0.0};
  bool plan_to_start{false};
  std::vector<CartesianTeachingWaypoint> cartesian_waypoints{};
};

using TaughtMotions = std::map<std::string, TaughtMotion>;

TaughtMotions parse_taught_motions(
  const std::string & json, const std::vector<double> & home,
  const std::vector<double> & tray_return, std::size_t joint_count);
}  // namespace arm_cell_motion
