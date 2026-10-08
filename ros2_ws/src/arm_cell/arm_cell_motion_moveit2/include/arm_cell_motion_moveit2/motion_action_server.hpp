#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "arm_cell_motion_moveit2/motion_core.hpp"

namespace arm_cell_motion_moveit2
{

class MotionActionServer
{
public:
  using Action = ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  MotionActionServer(
    rclcpp::Node * node,
    std::shared_ptr<MotionCore> core);

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Action::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  static void execute(
    const std::shared_ptr<GoalHandle> goal_handle,
    const std::shared_ptr<MotionCore> core);

  rclcpp::Node * node_;
  std::shared_ptr<MotionCore> core_;
  rclcpp_action::Server<Action>::SharedPtr server_;
};

inline UUID action_goal_uuid_to_message_uuid(const rclcpp_action::GoalUUID & action_uuid)
{
  UUID uuid;
  uuid.uuid = action_uuid;
  return uuid;
}

inline rclcpp_action::GoalResponse action_goal_response(
  MotionCore & core, const UUID & goal_uuid, const ExecuteTask::Goal & goal)
{
  return core.reserve_goal(goal_uuid, goal) == MotionTaskResultCode::TASK_RESULT_SUCCESS ?
         rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
         rclcpp_action::GoalResponse::REJECT;
}

inline uint8_t action_result_code(const MotionStatus & status, bool client_canceling)
{
  if (client_canceling) {
    return MotionTaskResultCode::TASK_RESULT_CANCELED;
  }
  return status.execution_state == MotionStatus::MOTION_STATE_STOPPED ?
         MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED :
         MotionTaskResultCode::TASK_RESULT_SUCCESS;
}

}  // namespace arm_cell_motion_moveit2
