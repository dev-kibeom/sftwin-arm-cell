#pragma once

#include <string>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

namespace arm_cell_motion_moveit2
{

struct PlaceApproachTrajectoryTimingResult
{
  bool success{false};
  double duration_s{0.0};
  std::string reason;
};

PlaceApproachTrajectoryTimingResult time_parameterize_place_approach_trajectory(
  const moveit::core::RobotModelConstPtr & robot_model,
  const moveit::core::RobotState & start_state,
  moveit_msgs::msg::RobotTrajectory & trajectory);

}  // namespace arm_cell_motion_moveit2
