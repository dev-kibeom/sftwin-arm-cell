#include "arm_cell_motion_moveit2/motion_action_server.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>
#include <thread>

namespace arm_cell_motion_moveit2
{

namespace
{
std::string uuid_string(const rclcpp_action::GoalUUID & uuid)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : uuid) {
    stream << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return stream.str();
}
}  // namespace

MotionActionServer::MotionActionServer(
  rclcpp::Node * node,
  std::shared_ptr<MotionCore> core)
: node_(node), core_(std::move(core))
{
  server_ = rclcpp_action::create_server<Action>(
    node_,
    "/motion/execute_task",
    std::bind(&MotionActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MotionActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&MotionActionServer::handle_accepted, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse MotionActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Action::Goal> goal)
{
  const auto validation_result = core_->reserve_goal(
    action_goal_uuid_to_message_uuid(uuid), *goal);
  RCLCPP_DEBUG(
    node_->get_logger(),
    "handle_goal uuid=%s task_type=%u validate_result=%u",
    uuid_string(uuid).c_str(), goal->task_type.value, validation_result);
  return validation_result == MotionTaskResultCode::TASK_RESULT_SUCCESS ?
         rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
         rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse MotionActionServer::handle_cancel(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  core_->request_cancel(action_goal_uuid_to_message_uuid(goal_handle->get_goal_id()));
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionActionServer::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  RCLCPP_DEBUG(
    node_->get_logger(), "handle_accepted uuid=%s entry=1",
    uuid_string(goal_handle->get_goal_id()).c_str());
  std::thread{&MotionActionServer::execute, goal_handle, core_}.detach();
}

void MotionActionServer::execute(
  const std::shared_ptr<GoalHandle> goal_handle,
  const std::shared_ptr<MotionCore> core)
{
  const auto uuid = action_goal_uuid_to_message_uuid(goal_handle->get_goal_id());
  RCLCPP_DEBUG(
    rclcpp::get_logger("motion_action_server"), "execute entry uuid=%s",
    uuid_string(goal_handle->get_goal_id()).c_str());
  RCLCPP_INFO(
    rclcpp::get_logger("motion_action_server"),
    "motion task started motion_execution_id=%s task_type=%u",
    uuid_string(goal_handle->get_goal_id()).c_str(),
    goal_handle->get_goal()->task_type.value);
  const auto & goal = *goal_handle->get_goal();
  if (goal.task_type.value == MotionTaskType::TASK_TYPE_PLACE) {
    const auto & pose = goal.target_pose;
    RCLCPP_DEBUG(
      rclcpp::get_logger("motion_action_server"),
      "PLACE recipe geometry received motion_execution_id=%s frame=%s object_position=(%.9f,%.9f,%.9f) "
      "object_orientation_xyzw=(%.9f,%.9f,%.9f,%.9f) position_tolerance_m=%.9f "
      "orientation_tolerance_rad=%.9f approach_direction_object=(%.9f,%.9f,%.9f) "
      "approach_distance_m=%.9f retract_distance_m=%.9f",
      uuid_string(goal_handle->get_goal_id()).c_str(), pose.header.frame_id.c_str(),
      pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
      pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
      pose.pose.orientation.w, goal.place_position_tolerance_m,
      goal.place_orientation_tolerance_rad, goal.place_approach_direction_object.x,
      goal.place_approach_direction_object.y, goal.place_approach_direction_object.z,
      goal.place_approach_distance_m, goal.place_retract_distance_m);
  }
  std::string diagnostic_detail;
  const auto result_code = core->accept_goal_with_diagnostic(
    uuid, *goal_handle->get_goal(), diagnostic_detail);
  RCLCPP_DEBUG(
    rclcpp::get_logger("motion_action_server"),
    "execute accept_goal result uuid=%s result_code=%u diagnostic=%s",
    uuid_string(goal_handle->get_goal_id()).c_str(), result_code, diagnostic_detail.c_str());
  auto result = std::make_shared<Action::Result>();
  result->result_code.value = result_code;

  if (result_code != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    result->diagnostic_detail = diagnostic_detail.empty() ?
      "motion goal rejected" : diagnostic_detail;
    if (result_code == MotionTaskResultCode::TASK_RESULT_CANCELED) {
      RCLCPP_INFO(
        rclcpp::get_logger("motion_action_server"),
        "motion execution canceled motion_execution_id=%s task_type=%u result_code=%u",
        uuid_string(goal_handle->get_goal_id()).c_str(),
        goal_handle->get_goal()->task_type.value, result_code);
      goal_handle->canceled(result);
    } else if (result_code == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED) {
      RCLCPP_WARN(
        rclcpp::get_logger("motion_action_server"),
        "motion execution interrupted motion_execution_id=%s task_type=%u result_code=%u diagnostic=%s",
        uuid_string(goal_handle->get_goal_id()).c_str(),
        goal_handle->get_goal()->task_type.value, result_code,
        result->diagnostic_detail.c_str());
      goal_handle->abort(result);
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("motion_action_server"),
        "motion execution failed motion_execution_id=%s task_type=%u result_code=%u diagnostic=%s",
        uuid_string(goal_handle->get_goal_id()).c_str(),
        goal_handle->get_goal()->task_type.value, result_code,
        result->diagnostic_detail.c_str());
      goal_handle->abort(result);
    }
    return;
  }

  auto feedback = std::make_shared<Action::Feedback>();
  feedback->phase.value = arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING;
  feedback->diagnostic_detail = "Motion backend execution active";
  goal_handle->publish_feedback(feedback);

  while (rclcpp::ok()) {
    core->refresh_stop_state();
    const auto status = core->status();
    if (!status.execution_active && status.backend_inactivity_confirmed) {
      result->result_code.value = action_result_code(status, goal_handle->is_canceling());
      if (goal_handle->is_canceling()) {
        RCLCPP_INFO(
          rclcpp::get_logger("motion_action_server"),
          "motion execution canceled motion_execution_id=%s task_type=%u result_code=%u",
          uuid_string(goal_handle->get_goal_id()).c_str(),
          goal_handle->get_goal()->task_type.value, result->result_code.value);
        goal_handle->canceled(result);
        return;
      }
      goal_handle->succeed(result);
      if (result->result_code.value == MotionTaskResultCode::TASK_RESULT_SUCCESS) {
        RCLCPP_INFO(
          rclcpp::get_logger("motion_action_server"),
          "motion task completed motion_execution_id=%s task_type=%u result_code=%u",
          uuid_string(goal_handle->get_goal_id()).c_str(),
          goal_handle->get_goal()->task_type.value, result->result_code.value);
      } else if (result->result_code.value ==
        MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED)
      {
        RCLCPP_WARN(
          rclcpp::get_logger("motion_action_server"),
          "motion task interrupted by Safety motion_execution_id=%s task_type=%u result_code=%u",
          uuid_string(goal_handle->get_goal_id()).c_str(),
          goal_handle->get_goal()->task_type.value, result->result_code.value);
      } else {
        RCLCPP_ERROR(
          rclcpp::get_logger("motion_action_server"),
          "motion task terminated unsuccessfully motion_execution_id=%s task_type=%u result_code=%u",
          uuid_string(goal_handle->get_goal_id()).c_str(),
          goal_handle->get_goal()->task_type.value, result->result_code.value);
      }
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

}  // namespace arm_cell_motion_moveit2
