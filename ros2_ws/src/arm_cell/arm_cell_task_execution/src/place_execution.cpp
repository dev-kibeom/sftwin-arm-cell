#include "arm_cell_task_execution/task_execution_modules.hpp"

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <arm_cell_interfaces/msg/motion_task_phase.hpp>

#include "arm_cell_task_execution/bt_callback_node.hpp"
#include "arm_cell_task_execution/motion_plan_execute_action.hpp"
#include "arm_cell_task_execution/task_execution_validation.hpp"

namespace arm_cell_task_execution
{
using namespace arm_cell_motion;
using namespace task_execution_validation;

namespace
{
constexpr std::uint8_t kPlaceReleaseMaxRetries = 3;
}  // namespace

namespace
{
BT::NodeStatus domain_status(std::uint8_t result)
{
  return result == MotionTaskResultCode::TASK_RESULT_SUCCESS ?
         BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
}  // namespace

uint8_t PlaceExecution::validate_goal()
{
  std::string place_strategy = "PLANNED";
  blackboard_->get("place_strategy", place_strategy);
  state_.waypoint_strategy = place_strategy == "WAYPOINT";
  if (state_.waypoint_strategy) {
    const auto & geometry = support_.geometry();
    if (!geometry.place_waypoint_configured ||
      !geometry.place_waypoint_clearance_toward_positive_y)
    {
      report_->failure_stage = "place_approach_planning";
      report_->result_code = MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
      return report_->result_code;
    }
    state_.goal.target_pose = geometry.place_waypoint_target_pose;
    state_.goal.place_approach_distance_m = geometry.place_waypoint_approach_distance_m;
    state_.goal.place_approach_direction_object =
      geometry.place_waypoint_approach_direction_object;
    state_.goal.place_orientation_constraint =
      ExecuteTask::Goal::PLACE_ORIENTATION_FIXED;
    state_.goal.has_place_tool_orientation_preference = false;
  }
  const auto & goal = state_.goal;
  if (!goal.has_target_pose || goal.target_pose.header.frame_id != "base_link" ||
    support_.object_state_tracker().state() != MotionObjectState::ATTACHED ||
    !support_.geometry().valid() ||
    !valid_pose(goal.target_pose) || !finite(goal.place_position_tolerance_m) ||
    goal.place_position_tolerance_m < 0.0 || !finite(goal.place_orientation_tolerance_rad) ||
    goal.place_orientation_tolerance_rad < 0.0 ||
    !valid_place_orientation_constraint(goal.place_orientation_constraint) ||
    (goal.place_orientation_constraint == ExecuteTask::Goal::PLACE_ORIENTATION_FREE &&
    goal.has_place_tool_orientation_preference) ||
    (goal.has_place_tool_orientation_preference &&
    !valid_unit_quaternion(goal.place_tool_orientation_preference)) ||
    !valid_unit_vector(goal.place_approach_direction_object) ||
    !finite(goal.place_approach_distance_m) || goal.place_approach_distance_m <= 0.0 ||
    !finite(goal.place_retract_distance_m) || goal.place_retract_distance_m <= 0.0)
  {
    report_->failure_stage = "validation:place_request";
    report_->result_code = MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
    return report_->result_code;
  }
  report_->result_code = MotionTaskResultCode::TASK_RESULT_SUCCESS;
  return report_->result_code;
}

std::uint8_t PlaceExecution::prepare_waypoint_clearance()
{
  if (!state_.waypoint_strategy) {return MotionTaskResultCode::TASK_RESULT_SUCCESS;}
  geometry_msgs::msg::PoseStamped measured;
  if (!support_.motion_port()->measuredTcpPose(measured) ||
    measured.header.frame_id != "base_link")
  {
    report_->failure_stage = "place_clearance_planning";
    return report_->result_code = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  }
  const auto boundary = support_.geometry().place_waypoint_clearance_y_boundary_m;
  if (!std::isfinite(measured.pose.position.y) || !std::isfinite(boundary)) {
    report_->failure_stage = "place_clearance_planning";
    return report_->result_code = MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
  }
  state_.waypoint_clearance_needed = measured.pose.position.y < boundary;
  state_.waypoint_clearance_target = measured;
  if (state_.waypoint_clearance_needed) {
    state_.waypoint_clearance_target.pose.position.y = boundary +
      support_.geometry().place_waypoint_clearance_y_margin_m;
  }
  return report_->result_code = MotionTaskResultCode::TASK_RESULT_SUCCESS;
}

std::uint8_t PlaceExecution::verify_waypoint_clearance()
{
  geometry_msgs::msg::PoseStamped measured;
  if (!support_.motion_port()->measuredTcpPose(measured) ||
    measured.header.frame_id != "base_link" ||
    !std::isfinite(measured.pose.position.y) ||
    measured.pose.position.y < support_.geometry().place_waypoint_clearance_y_boundary_m)
  {
    report_->failure_stage = "place_clearance_target_not_reached";
    return report_->result_code = MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED;
  }
  return report_->result_code = MotionTaskResultCode::TASK_RESULT_SUCCESS;
}

std::uint8_t PlaceExecution::execute_waypoint_home()
{
  if (!state_.waypoint_strategy) {return MotionTaskResultCode::TASK_RESULT_SUCCESS;}
  report_->failure_stage = "place_clearance_execution";
  const auto result = support_.execute_legacy_movement(
    arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_GO_HOME);
  report_->result_code = result;
  return result;
}

uint8_t PlaceExecution::execute_approach(std::uint8_t motion_result)
{
  report_->failure_stage = "place_approach_backend_request";
  report_->result_code = motion_result;
  if (report_->result_code != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    const auto stage = support_.motion_port()->lastPlaceFailureStage();
    if (!stage.empty()) {report_->failure_stage = stage;}
  }

  return report_->result_code;
}

uint8_t PlaceExecution::execute_insertion(std::uint8_t motion_result)
{
  report_->failure_stage = "place_global_planning_or_execution";
  report_->result_code = motion_result;
  if (report_->result_code != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    const auto stage = support_.motion_port()->lastPlaceFailureStage();
    if (!stage.empty()) {report_->failure_stage = stage;}
  }
  return report_->result_code;
}

bool PlaceExecution::advance_planning_candidate()
{
  if (report_->result_code == MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    return false;
  }
  if (support_.interruption_checker()) {
    const auto interrupted = support_.interruption_checker()();
    if (interrupted != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      report_->result_code = interrupted;
      report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
      return false;
    }
  }
  return support_.motion_port()->advanceToNextPlanningCandidate();
}

uint8_t PlaceExecution::execute_release_attempt()
{
  using Feedback = ExecuteTask::Feedback;
  if (state_.release_attempt >= kPlaceReleaseMaxRetries + 1) {
    return report_->result_code;
  }
  ++state_.release_attempt;
  const auto last_failure = state_.release_attempt == 1 ?
    Feedback::RELEASE_FAILURE_NONE : Feedback::RELEASE_FAILURE_CONFIRMATION_TIMEOUT;
  std::uint64_t baseline = 0;
  report_->result_code = support_.execute_gripper_open(
    baseline, state_.release_attempt, kPlaceReleaseMaxRetries,
    last_failure);
  if (report_->result_code != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    report_->failure_stage = "gripper:place_open_command";
    state_.holding_result = HoldingWaitResult::UNKNOWN;
    return report_->result_code;
  }
  report_->failure_stage = "holding:released_confirmation";
  state_.holding_result = support_.fresh_holding(
    HoldingState::RELEASED,
    baseline);
  return report_->result_code;
}

bool PlaceExecution::release_confirmed() const
{return state_.holding_result == HoldingWaitResult::CONFIRMED;}

bool PlaceExecution::continue_release_retries() const
{
  return state_.holding_result == HoldingWaitResult::TIMEOUT &&
         state_.release_attempt <= kPlaceReleaseMaxRetries;
}

uint8_t PlaceExecution::finalize_release()
{
  if (report_->result_code != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
    return report_->result_code;
  }
  if (state_.holding_result != HoldingWaitResult::CONFIRMED) {
    report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
    report_->result_code =
      state_.holding_result == HoldingWaitResult::CANCELED ?
      MotionTaskResultCode::TASK_RESULT_CANCELED :
      state_.holding_result == HoldingWaitResult::SAFETY_PREEMPTED ?
      MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED :
      state_.holding_result == HoldingWaitResult::BACKEND_UNAVAILABLE ?
      MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE :
      MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE;
    return report_->result_code;
  }
  if (support_.phase_callback()) {
    support_.phase_callback()(
      arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_DETACHING, 0, 0,
      0);
  }
  report_->failure_stage = "attach_detach:confirm_released_grasp_relation";
  support_.motion_port()->clearGraspRelationAfterRelease();
  support_.object_state_tracker().mark_detached();
  report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_PLACED;
  return MotionTaskResultCode::TASK_RESULT_SUCCESS;
}

uint8_t PlaceExecution::execute_retreat(std::uint8_t motion_result)
{
  report_->failure_stage = "place_retract_backend_request";
  report_->result_code = motion_result;
  return report_->result_code;
}

void PlaceExecution::begin_release_retries()
{
  state_.release_attempt = 0;
  state_.holding_result = HoldingWaitResult::TIMEOUT;
  report_->result_code = MotionTaskResultCode::TASK_RESULT_SUCCESS;
}

void PlaceExecution::register_nodes(
  BT::BehaviorTreeFactory & factory, BT::Blackboard::Ptr blackboard,
  TaskExecutionReport & report)
{
  blackboard_ = std::move(blackboard);
  report_ = &report;
  const auto set_result = [this](std::uint8_t result) {
      report_->result_code = result; return result;
    };
  register_callback_action(
    factory, "ValidatePlaceGoal", [this, set_result]() {
      state_.goal = blackboard_->get<ExecuteTask::Goal>("goal");
      return domain_status(set_result(validate_goal()));
    });
  register_callback_action(
    factory, "IsPlaceWaypointStrategy", [this]() {
      return state_.waypoint_strategy ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  register_callback_action(
    factory, "IsPlacePlannedStrategy", [this]() {
      return state_.waypoint_strategy ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    });
  register_callback_action(
    factory, "PreparePlaceWaypointClearance", [this, set_result]() {
      return domain_status(set_result(prepare_waypoint_clearance()));
    });
  register_callback_action(
    factory, "PlaceWaypointClearanceNotNeeded", [this]() {
      return !state_.waypoint_clearance_needed ? BT::NodeStatus::SUCCESS :
             BT::NodeStatus::FAILURE;
    });
  register_plan_motion_action(
    factory, "PlanPlaceWaypointClearance", support_,
    [this]() {
      report_->failure_stage = "place_clearance_planning";
      auto request = support_.make_motion_request(
        state_.goal, arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING,
        report_->motion_execution_id, "place_waypoint_clearance");
      request.segment = MotionSegmentKind::PLACE_CLEARANCE;
      request.target_pose = state_.waypoint_clearance_target;
      request.target_is_tcp_pose = true;
      request.place_waypoint_clearance_y_margin_m =
        support_.geometry().place_waypoint_clearance_y_margin_m;
      return request;
    }, arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_PLANNING,
    [this, set_result](const MotionPlanningResult & result) {
      if (result.result_code != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
        report_->failure_stage = "place_clearance_planning";
        return domain_status(set_result(result.result_code));
      }
      state_.pending_motion_plan = result.plan;
      return BT::NodeStatus::SUCCESS;
    });
  register_execute_planned_motion_action(
    factory, "ExecutePlaceWaypointClearance", support_,
    [this]() {return state_.pending_motion_plan;}, true,
    [this, set_result](const MotionPortResult & result) {
      if (result.result_code != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
        report_->failure_stage = "place_clearance_execution";
        return domain_status(set_result(result.result_code));
      }
      return domain_status(set_result(verify_waypoint_clearance()));
    });
  register_callback_action(
    factory, "ExecutePlaceWaypointHome", [this, set_result]() {
      return domain_status(set_result(execute_waypoint_home()));
    });
  register_plan_motion_action(
    factory, "PlanPlaceApproach", support_,
    [this]() {
      report_->failure_stage = "place_approach_backend_request";
      auto request = support_.make_motion_request(
        state_.goal, arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING,
        report_->motion_execution_id);
      if (state_.waypoint_strategy) {
        request.target_pose = support_.geometry().place_waypoint_target_pose;
        request.constraints.approach_distance_m =
          support_.geometry().place_waypoint_approach_distance_m;
        request.place_waypoint_approach = true;
      }
      return request;
    }, arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING,
    [this, set_result](const MotionPlanningResult & result) {
      if (result.result_code != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
        return domain_status(set_result(execute_approach(result.result_code)));
      }
      state_.pending_motion_plan = result.plan;
      return BT::NodeStatus::SUCCESS;
    });
  register_execute_planned_motion_action(
    factory, "ExecutePlaceApproach", support_,
    [this]() {return state_.pending_motion_plan;}, true,
    [this, set_result](const MotionPortResult & result) {
      return domain_status(set_result(execute_approach(result.result_code)));
    });
  register_plan_motion_action(
    factory, "PlanPlaceInsertion", support_,
    [this]() {
      report_->failure_stage = "place_global_planning_or_execution";
      return support_.make_motion_request(
        state_.goal, arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING,
        report_->motion_execution_id);
    }, arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING,
    [this, set_result](const MotionPlanningResult & result) {
      if (result.result_code != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
        return domain_status(set_result(execute_insertion(result.result_code)));
      }
      state_.pending_motion_plan = result.plan;
      return BT::NodeStatus::SUCCESS;
    });
  register_execute_planned_motion_action(
    factory, "ExecutePlaceInsertion", support_,
    [this]() {return state_.pending_motion_plan;}, true,
    [this, set_result](const MotionPortResult & result) {
      return domain_status(set_result(execute_insertion(result.result_code)));
    });
  register_callback_action(
    factory, "AdvancePlacePlanningCandidate", [this]() {
      if (advance_planning_candidate()) {return BT::NodeStatus::SUCCESS;}
      return BT::NodeStatus::FAILURE;
    });
  register_callback_action(
    factory, "BeginPlaceReleaseRetries", [this]() {
      begin_release_retries();
      return BT::NodeStatus::SUCCESS;
    });
  register_callback_action(
    factory, "ExecutePlaceReleaseAttempt", [this, set_result]() {
      set_result(execute_release_attempt());
      return BT::NodeStatus::SUCCESS;
    });
  register_callback_action(
    factory, "PlaceReleaseConfirmed", [this]() {
      return release_confirmed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  register_callback_action(
    factory, "ContinuePlaceReleaseRetries", [this]() {
      const bool retry = continue_release_retries();
      if (retry) {
        RCLCPP_WARN(
          rclcpp::get_logger("place_execution"),
          "PLACE release confirmation timed out; retrying OPEN attempt=%u/4",
          static_cast<unsigned>(state_.release_attempt + 1));
      }
      return retry ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  register_callback_action(
    factory, "ReleasePlacedObject", [this, set_result]() {
      return domain_status(set_result(finalize_release()));
    });
  register_callback_action(
    factory, "FinalizePlaceRelease", [this, set_result]() {
      return domain_status(set_result(finalize_release()));
    });
  register_plan_motion_action(
    factory, "PlanPlaceRetreat", support_,
    [this]() {
      report_->failure_stage = "place_retract_backend_request";
      return support_.make_motion_request(
        state_.goal, arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING,
        report_->motion_execution_id);
    }, arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING,
    [this, set_result](const MotionPlanningResult & result) {
      if (result.result_code != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
        return domain_status(set_result(execute_retreat(result.result_code)));
      }
      state_.pending_motion_plan = result.plan;
      return BT::NodeStatus::SUCCESS;
    });
  register_execute_planned_motion_action(
    factory, "RetractPlacedObject", support_,
    [this]() {return state_.pending_motion_plan;}, true,
    [this, set_result](const MotionPortResult & result) {
      return domain_status(set_result(execute_retreat(result.result_code)));
    });
}

}  // namespace arm_cell_task_execution
