#include "arm_cell_motion_moveit2/place_approach_trajectory.hpp"

#include <cmath>
#include <exception>
#include <limits>
#include <utility>
#include <vector>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include "arm_cell_motion_moveit2/executable_trajectory_finalize.hpp"

namespace arm_cell_motion_moveit2
{

PlaceApproachTrajectoryTimingResult time_parameterize_place_approach_trajectory(
  const moveit::core::RobotModelConstPtr & robot_model,
  const moveit::core::RobotState & start_state,
  moveit_msgs::msg::RobotTrajectory & trajectory)
{
  if (!robot_model || !robot_model->getJointModelGroup("arm") ||
    trajectory.joint_trajectory.joint_names.empty() ||
    trajectory.joint_trajectory.points.empty())
  {
    return {false, 0.0, "trajectory_or_arm_model_missing"};
  }

  std::vector<double> velocity_limits;
  velocity_limits.reserve(trajectory.joint_trajectory.joint_names.size());
  for (const auto & joint_name : trajectory.joint_trajectory.joint_names) {
    const auto * joint_model = robot_model->getJointModel(joint_name);
    if (!joint_model || joint_model->getVariableBounds().empty()) {
      return {false, 0.0, "trajectory_joint_model_or_velocity_limit_missing"};
    }
    const double velocity_limit = joint_model->getVariableBounds().front().max_velocity_;
    if (!std::isfinite(velocity_limit) || velocity_limit <= 0.0) {
      return {false, 0.0, "trajectory_joint_model_or_velocity_limit_missing"};
    }
    velocity_limits.push_back(velocity_limit);
  }
  const auto finalized = finalize_executable_trajectory(
    trajectory, trajectory.joint_trajectory.joint_names, velocity_limits,
    std::numeric_limits<double>::max(),
    [&](moveit_msgs::msg::RobotTrajectory & raw_trajectory) {
      robot_trajectory::RobotTrajectory timed_trajectory(robot_model, "arm");
      timed_trajectory.setRobotTrajectoryMsg(start_state, raw_trajectory);
      trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
      if (!time_parameterization.computeTimeStamps(timed_trajectory, 1.0, 1.0)) {
        return false;
      }
      timed_trajectory.getRobotTrajectoryMsg(raw_trajectory);
      return true;
    });
  return {finalized.success, finalized.duration_s, finalized.reason};
}

}  // namespace arm_cell_motion_moveit2
