#pragma once

#include <string>
#include <vector>

#include <moveit/robot_state/robot_state.h>

namespace arm_cell_motion
{

inline void apply_measured_joint_positions(
  moveit::core::RobotState & planning_start,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & measured_positions)
{
  planning_start.setVariablePositions(joint_names, measured_positions);
  planning_start.update();
}

}  // namespace arm_cell_motion
