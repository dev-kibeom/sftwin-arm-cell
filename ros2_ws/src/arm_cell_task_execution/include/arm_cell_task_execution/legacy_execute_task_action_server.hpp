#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "arm_cell_motion/gripper_port.hpp"
#include "arm_cell_motion/motion_geometry.hpp"
#include "arm_cell_motion/motion_port.hpp"
#include "arm_cell_task_execution/task_executor.hpp"

namespace arm_cell_task_execution
{

using namespace arm_cell_motion;

class LegacyExecuteTaskActionServer final
{
public:
  using Action = arm_cell_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  LegacyExecuteTaskActionServer(
    rclcpp::Node * node, std::shared_ptr<MotionPort> motion_port,
    std::shared_ptr<TaskExecutor> task_executor);

private:
  struct CancellationState
  {
    std::mutex mutex;
    std::unordered_set<std::string> before_execution;
  };

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Action::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  static void execute(
    const std::shared_ptr<GoalHandle> goal_handle,
    const std::shared_ptr<MotionPort> motion_port,
    const std::shared_ptr<TaskExecutor> task_executor,
    const std::shared_ptr<CancellationState> cancellation_state,
    const std::shared_ptr<std::mutex> execution_mutex);

  rclcpp::Node * node_;
  std::shared_ptr<MotionPort> motion_port_;
  std::shared_ptr<TaskExecutor> task_executor_;
  std::shared_ptr<std::mutex> execution_mutex_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  std::shared_ptr<CancellationState> cancellation_state_{std::make_shared<CancellationState>()};
};

}  // namespace arm_cell_task_execution
