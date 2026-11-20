#include "arm_cell_motion/prepared_teaching.hpp"

#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>
#include <algorithm>
#include <cmath>

namespace arm_cell_motion
{
namespace
{
using Code = arm_cell_interfaces::msg::MotionTaskResultCode;
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;

bool prepare_entry(
  const TaughtMotion & motion, const std::string & group_name,
  const moveit::core::RobotState & start,
  const std::vector<JointMotionLimit> & limits, std::mutex & solver_mutex,
  const std::function<std::uint8_t()> & interruption, PreparedTeachingEntry & entry)
{
  auto state = start;
  const auto * group = state.getJointModelGroup(group_name);
  const auto & names = group->getVariableNames();
  std::vector<JointTrajectory> segments;
  for (std::size_t i = entry.first_waypoint; i <= entry.last_waypoint; ++i) {
    if ((interruption && interruption() != Code::TASK_RESULT_SUCCESS) ||
      !state.satisfiesBounds(group))
    {return false;}
    const auto & point = motion.cartesian_waypoints[i];
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(point.pose[0], point.pose[1], point.pose[2]);
    pose.linear() = Eigen::Quaterniond(point.pose[6], point.pose[3], point.pose[4], point.pose[5])
      .toRotationMatrix();
    const Eigen::Isometry3d target = state.getGlobalLinkTransform("base_link") * pose;
    JointTrajectory segment;
    entry.target_joints = point.joint_positions;
    entry.linear_joint_change = {};
    if (point.motion == "MoveJ") {
      if (entry.target_joints.empty()) {
        std::lock_guard<std::mutex> lock(solver_mutex);
        if (resolve_pose_joint_target(
            state, group_name, "sf_grasp_tcp", target,
            entry.target_joints) !=
          Code::TASK_RESULT_SUCCESS)
        {return false;}
      }
      std::vector<double> from;
      state.copyJointGroupPositions(group, from);
      auto generated = make_taught_joint_trajectory(
        names, from, entry.target_joints, limits, motion.velocity_scale, motion.acceleration_scale);
      if (!generated.valid) {return false;}
      segment = std::move(generated.trajectory);
    } else {
      moveit_msgs::msg::RobotTrajectory generated;
      {
        std::lock_guard<std::mutex> lock(solver_mutex);
        if (generate_linear_joint_trajectory(
            state, group_name, "sf_grasp_tcp", target, generated, interruption,
            &entry.linear_joint_change) != Code::TASK_RESULT_SUCCESS)
        {return false;}
      }
      segment = std::move(generated.joint_trajectory);
    }
    state.setJointGroupPositions(group, segment.points.back().positions);
    state.update();
    segments.push_back(std::move(segment));
  }

  if (segments.size() == 1 && motion.cartesian_waypoints[entry.first_waypoint].motion == "MoveJ") {
    entry.trajectory = std::move(segments.front());
    return true;
  }
  moveit_msgs::msg::RobotTrajectory trajectory;
  if (segments.size() == 1) {
    trajectory.joint_trajectory = std::move(segments.front());
  } else if (!blend_taught_joint_paths(segments, trajectory)) {
    return false;
  }
  if (!retime_joint_trajectory(
      start.getRobotModel(), group_name, start, trajectory,
      motion.velocity_scale, motion.acceleration_scale, &limits))
  {return false;}
  entry.trajectory = std::move(trajectory.joint_trajectory);
  return true;
}
}  // namespace

const PreparedTeachingEntry * find_prepared_teaching(
  const std::vector<PreparedTeachingEntry> & entries, std::size_t first_waypoint,
  const std::vector<double> & start, double tolerance)
{
  for (const auto & entry : entries) {
    const auto & nominal = entry.trajectory.points.front().positions;
    if (entry.first_waypoint == first_waypoint &&
      std::equal(
        nominal.begin(), nominal.end(), start.begin(), start.end(),
        [tolerance](double a, double b) {return std::abs(a - b) <= tolerance;}))
    {
      return &entry;
    }
  }
  return nullptr;
}

PreparedTeaching prepare_teaching_cycle(
  const TaughtMotions & motions, const std::vector<std::string> & cycle,
  const moveit::core::RobotState & home, const std::string & group_name,
  const std::vector<JointMotionLimit> & limits, std::mutex & solver_mutex,
  const std::function<std::uint8_t()> & interruption)
{
  PreparedTeaching prepared;
  const auto * group = home.getJointModelGroup(group_name);
  if (!group || !home.getRobotModel()->hasLinkModel("base_link") ||
    limits.size() != group->getVariableCount() || !home.satisfiesBounds(group))
  {return prepared;}
  auto state = home;
  for (int pass = 0; pass < 2; ++pass) {
    for (const auto & name : cycle) {
      const auto found = motions.find(name);
      if (found == motions.end()) {return prepared;}
      const auto & motion = found->second;
      const auto & waypoints = motion.cartesian_waypoints;
      // Scene-dependent and legacy joint-path operations are planned at execution.
      if (waypoints.empty() || waypoints.back().blend ||
        std::any_of(
          waypoints.begin(), waypoints.end(), [](const auto & point) {
            return point.motion != "MoveJ" && point.motion != "MoveL";
          }))
      {return prepared;}
      auto & entries = prepared[name];
      for (std::size_t first = 0; first < waypoints.size(); ) {
        if (interruption && interruption() != Code::TASK_RESULT_SUCCESS) {return {};}
        std::vector<double> nominal;
        state.copyJointGroupPositions(group, nominal);
        auto entry = find_prepared_teaching(entries, first, nominal, 1e-9);
        if (!entry) {
          auto last = first;
          while (waypoints[last].blend) {++last;}
          PreparedTeachingEntry generated{first, last, {}, {}, {}};
          if (!prepare_entry(
              motion, group_name, state, limits, solver_mutex, interruption,
              generated))
          {
            return prepared;
          }
          entries.push_back(std::move(generated));
          entry = &entries.back();
        }
        state.setJointGroupPositions(group, entry->trajectory.points.back().positions);
        state.update();
        first = entry->last_waypoint + 1;
      }
    }
  }
  return prepared;
}
}  // namespace arm_cell_motion
