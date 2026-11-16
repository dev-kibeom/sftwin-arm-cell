#pragma once

#include <map>
#include <string>
#include <vector>

namespace arm_cell_motion
{
struct TaughtMotion
{
  std::vector<double> joint_positions;
  double velocity_scale;
  double acceleration_scale;
};

using TaughtMotions = std::map<std::string, TaughtMotion>;

TaughtMotions parse_taught_motions(
  const std::string & json, const std::vector<double> & home,
  const std::vector<double> & tray_return, std::size_t joint_count);
}  // namespace arm_cell_motion
