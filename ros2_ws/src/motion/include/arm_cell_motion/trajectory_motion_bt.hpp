#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <behaviortree_cpp_v3/bt_factory.h>

namespace arm_cell_motion
{
// Operations share one movement-owned trajectory. Nodes do not reconstruct it.
struct TrajectoryMotionOperations
{
  std::function<std::uint8_t()> resolve_joint_target;
  std::function<std::uint8_t()> generate_joint_trajectory;
  std::function<std::uint8_t()> generate_linear_trajectory;
  std::function<std::uint8_t()> plan_collision_free_path;
  std::function<std::uint8_t()> check_trajectory_collision;
  std::function<std::uint8_t()> execute_trajectory;
  std::function<std::uint8_t()> interruption;
};

struct TrajectoryMotionResult
{
  std::uint8_t code;
  std::string failure_stage;
};

// Registration allows callers to compose these same functional actions in
// their own trees. Operations and result must outlive the factory and tree.
void register_trajectory_motion_nodes(
  BT::BehaviorTreeFactory & factory, const TrajectoryMotionOperations & operations,
  TrajectoryMotionResult & result);
void register_trajectory_motion_trees(BT::BehaviorTreeFactory & factory);
TrajectoryMotionResult run_trajectory_motion(
  const std::string & motion, const TrajectoryMotionOperations & operations);
}  // namespace arm_cell_motion
