#pragma once

#include <map>
#include <array>
#include <string>
#include <vector>

namespace arm_cell_motion
{
struct CartesianTeachingWaypoint
{
  std::array<double, 7> pose{0., 0., 0., 0., 0., 0., 1.};
  std::string motion{"MoveJ"};
  std::vector<double> joint_positions{};
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
  std::string movement{"MoveJ"};
};

using TaughtMotions = std::map<std::string, TaughtMotion>;

TaughtMotions parse_taught_motions(
  const std::string & json, const std::vector<double> & home,
  const std::vector<double> & tray_return, std::size_t joint_count);
}  // namespace arm_cell_motion
