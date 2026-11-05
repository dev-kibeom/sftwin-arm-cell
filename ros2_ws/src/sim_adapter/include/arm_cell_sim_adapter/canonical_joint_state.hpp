#ifndef ARM_CELL_SIM_ADAPTER__CANONICAL_JOINT_STATE_HPP_
#define ARM_CELL_SIM_ADAPTER__CANONICAL_JOINT_STATE_HPP_

#include <optional>
#include <string>
#include <vector>
#include <sensor_msgs/msg/joint_state.hpp>

namespace arm_cell_sim_adapter
{
std::optional<sensor_msgs::msg::JointState> canonical_joint_state(
  const sensor_msgs::msg::JointState & raw,
  const std::vector<std::string> & independent_joints);
}  // namespace arm_cell_sim_adapter
#endif
