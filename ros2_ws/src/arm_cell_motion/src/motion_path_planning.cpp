#include "arm_cell_motion/motion_path_planning.hpp"
#include "arm_cell_motion/pick_retract_action_wait.hpp"

#include <chrono>
#include <future>
#include <moveit_msgs/action/move_group.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace arm_cell_motion
{
moveit::core::MoveItErrorCode plan_with_request_scene_diff(
  moveit::planning_interface::MoveGroupInterface & move_group,
  moveit::planning_interface::MoveGroupInterface::Plan & plan,
  const moveit_msgs::msg::PlanningScene & planning_scene_diff,
  const std::function<bool()> & interrupted)
{
  constexpr auto cancel_result_grace = std::chrono::milliseconds(1000);
  auto & client = move_group.getMoveGroupClient();
  if (!client.action_server_is_ready()) {
    return moveit::core::MoveItErrorCode::FAILURE;
  }
  moveit_msgs::action::MoveGroup::Goal goal;
  move_group.constructMotionPlanRequest(goal.request);
  goal.planning_options.plan_only = true;
  goal.planning_options.look_around = false;
  goal.planning_options.replan = false;
  goal.planning_options.planning_scene_diff = planning_scene_diff;
  const auto action_started = std::chrono::steady_clock::now();
  auto goal_handle_future = client.async_send_goal(goal);
  const auto acceptance_deadline = action_started + std::chrono::duration_cast<
    std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(
      goal.request.allowed_planning_time));
  while (goal_handle_future.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready) {
    if ((interrupted && interrupted()) || std::chrono::steady_clock::now() >= acceptance_deadline) {
      // A late plan-only response cannot execute or mutate the installed scene.
      return moveit::core::MoveItErrorCode::COMMUNICATION_FAILURE;
    }
  }
  const auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {return moveit::core::MoveItErrorCode::FAILURE;}
  auto result_future = client.async_get_result(goal_handle);
  const auto request_budget = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(goal.request.allowed_planning_time));
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - action_started);
  const auto remaining_budget = request_budget > elapsed ? request_budget - elapsed :
    std::chrono::milliseconds::zero();
  const auto wait_outcome = wait_for_pick_retract_action_result(
    [&result_future](std::chrono::milliseconds timeout) {
      return result_future.wait_for(timeout);
    }, remaining_budget, interrupted,
    [&client, &goal_handle]() {(void)client.async_cancel_goal(goal_handle);}, cancel_result_grace);
  if (!wait_outcome.result_ready ||
    wait_outcome.termination != PickRetractActionWaitTermination::RESULT_READY)
  {
    return moveit::core::MoveItErrorCode::FAILURE;
  }
  const auto wrapped_result = result_future.get();
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped_result.result) {
    return wrapped_result.result ?
           moveit::core::MoveItErrorCode(wrapped_result.result->error_code.val) :
           moveit::core::MoveItErrorCode::FAILURE;
  }
  plan.start_state_ = wrapped_result.result->trajectory_start;
  plan.trajectory_ = wrapped_result.result->planned_trajectory;
  plan.planning_time_ = wrapped_result.result->planning_time;
  return wrapped_result.result->error_code;
}

moveit::core::MoveItErrorCode plan_pick_approach(
  moveit::planning_interface::MoveGroupInterface & move_group,
  moveit::planning_interface::MoveGroupInterface::Plan & plan,
  const moveit_msgs::msg::PlanningScene * planning_scene_diff,
  const std::function<bool()> & interrupted)
{
  return planning_scene_diff ?
         plan_with_request_scene_diff(move_group, plan, *planning_scene_diff, interrupted) :
         move_group.plan(plan);
}
}  // namespace arm_cell_motion
