#pragma once

#include "arm_cell_motion/taught_motion.hpp"
#include "arm_cell_motion/trajectory_generation.hpp"

#include <mutex>

namespace arm_cell_motion
{
struct PreparedTeachingEntry
{
  std::size_t first_waypoint;
  std::size_t last_waypoint;
  trajectory_msgs::msg::JointTrajectory trajectory;
  std::vector<double> target_joints;
  LinearJointChangeDiagnostics linear_joint_change;
};

using PreparedTeaching = std::map<std::string, std::vector<PreparedTeachingEntry>>;

// A cycle is traversed from home, then from its own final (retreat) state.
// Entries end only at a stop point; adjacent blended segments are kept together.
PreparedTeaching prepare_teaching_cycle(
  const TaughtMotions & motions, const std::vector<std::string> & cycle,
  const moveit::core::RobotState & home, const std::string & group,
  const std::vector<JointMotionLimit> & limits, std::mutex & solver_mutex,
  const std::function<std::uint8_t()> & interruption = {});

const PreparedTeachingEntry * find_prepared_teaching(
  const std::vector<PreparedTeachingEntry> & entries, std::size_t first_waypoint,
  const std::vector<double> & start, double tolerance);
}  // namespace arm_cell_motion
