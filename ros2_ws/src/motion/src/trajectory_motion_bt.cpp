#include "arm_cell_motion/trajectory_motion_bt.hpp"
#include "trajectory_motion_tree_xml.hpp"
#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>

namespace arm_cell_motion
{
namespace
{
using Code = arm_cell_interfaces::msg::MotionTaskResultCode;
}

void register_trajectory_motion_nodes(
  BT::BehaviorTreeFactory & factory, const TrajectoryMotionOperations & operations,
  TrajectoryMotionResult & result)
{
  const auto add = [&](const char * name, const std::function<std::uint8_t()> & operation) {
      factory.registerSimpleAction(name, [&, name, operation](BT::TreeNode &) {
        result.failure_stage = name;
        result.code = operations.interruption ? operations.interruption() : Code::TASK_RESULT_SUCCESS;
        if (result.code == Code::TASK_RESULT_SUCCESS) {
          result.code = operation ? operation() : Code::TASK_RESULT_BACKEND_UNAVAILABLE;
        }
        return result.code == Code::TASK_RESULT_SUCCESS ? BT::NodeStatus::SUCCESS :
               BT::NodeStatus::FAILURE;
      });
    };
  add("ResolveJointTarget", operations.resolve_joint_target);
  add("GenerateJointTrajectory", operations.generate_joint_trajectory);
  add("GenerateLinearTrajectory", operations.generate_linear_trajectory);
  add("PlanCollisionFreePath", operations.plan_collision_free_path);
  add("CheckTrajectoryCollision", operations.check_trajectory_collision);
  add("ExecuteTrajectory", operations.execute_trajectory);
}

void register_trajectory_motion_trees(BT::BehaviorTreeFactory & factory)
{
  factory.registerBehaviorTreeFromText(kTrajectoryMotionTreeXml);
}

TrajectoryMotionResult run_trajectory_motion(
  const std::string & motion, const TrajectoryMotionOperations & operations)
{
  TrajectoryMotionResult result{Code::TASK_RESULT_BACKEND_ERROR, "CreateMotionTree"};
  BT::BehaviorTreeFactory factory;
  register_trajectory_motion_nodes(factory, operations, result);
  register_trajectory_motion_trees(factory);
  auto tree = factory.createTree(motion);
  if (tree.tickRoot() == BT::NodeStatus::SUCCESS) {result.failure_stage.clear();}
  return result;
}
}  // namespace arm_cell_motion
