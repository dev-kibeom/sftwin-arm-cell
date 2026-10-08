#pragma once

#include <string>
#include <vector>

namespace arm_cell_motion_moveit2
{

bool should_log_settling_snapshot(
  double last_logged_simulation_time,
  double simulation_time,
  double period_seconds);

std::string format_trajectory_settling_snapshot(
  double simulation_time_since_trajectory_end,
  const std::vector<double> & commanded_positions,
  const std::vector<double> & actual_positions,
  const std::vector<std::string> & joint_names);

}  // namespace arm_cell_motion_moveit2
