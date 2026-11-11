#include "arm_cell_task_execution/task_execution_modules.hpp"

#include <rclcpp/rclcpp.hpp>

#include "arm_cell_task_execution/bt_callback_node.hpp"
#include "arm_cell_task_execution/payload_home_action.hpp"
#include "arm_cell_task_execution/motion_plan_execute_action.hpp"

namespace arm_cell_task_execution
{
using namespace arm_cell_motion;
namespace
{
BT::NodeStatus domain_status(std::uint8_t result)
{
  return result == MotionTaskResultCode::TASK_RESULT_SUCCESS ?
         BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
}  // namespace

bool PlaceRecovery::approach_recovery_eligible() const
{
  const bool place_approach_failure = report_->failure_stage == "place_approach_planning";
  const bool place_clearance_failure =
    report_->failure_stage == "place_clearance_planning" ||
    report_->failure_stage == "place_clearance_execution" ||
    report_->failure_stage == "place_clearance_target_not_reached";
  const bool place_insertion_planning_failure = report_->failure_stage ==
    "place_insertion_planning";
  return (place_approach_failure || place_clearance_failure || place_insertion_planning_failure) &&
         (report_->result_code ==
         MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE ||
         report_->result_code ==
         MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED ||
         report_->result_code ==
         MotionTaskResultCode::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED);
}

void PlaceRecovery::prepare_held_failure_recovery()
{
  state_.return_needed = false;
  if (report_->result_code == MotionTaskResultCode::TASK_RESULT_SUCCESS ||
    (report_->failure_stage != "place_insertion_planning" &&
    report_->failure_stage != "place_insertion_execution"))
  {
    return;
  }
  if (support_.interruption_checker()) {
    const auto interrupted = support_.interruption_checker()();
    if (interrupted != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      report_->result_code = interrupted;
      report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
      return;
    }
  }
  state_.original_result = report_->result_code;
  const auto observed = support_.gripper()->holding();
  if (observed.fresh() && observed.state == HoldingState::HELD) {
    state_.return_needed = true;
  } else {
    report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
  }
}

std::uint8_t PlaceRecovery::return_held_home()
{
  if (support_.interruption_checker()) {
    const auto interrupted = support_.interruption_checker()();
    if (interrupted != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      report_->result_code = interrupted;
      report_->failure_stage = "place_return_interrupted_before_home";
      report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
      return report_->result_code;
    }
  }
  report_->failure_stage = "place_return_home";
  report_->result_code = support_.execute_legacy_movement(MotionTaskType::TASK_TYPE_GO_HOME);
  if (report_->result_code != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
  }
  return report_->result_code;
}

std::uint8_t PlaceRecovery::finalize_held_return()
{
  if (report_->object_disposition != ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED) {
    return report_->result_code;
  }
  report_->result_code = state_.original_result;
  report_->failure_stage = "place_return_completed";
  return report_->result_code;
}

bool PlaceRecovery::advance_retreat_recovery()
{
  if (!state_.retreat_result_captured) {
    state_.original_retreat_result = report_->result_code;
    state_.retreat_result_captured = true;
  }
  if (report_->result_code == MotionTaskResultCode::TASK_RESULT_SUCCESS ||
    report_->result_code == MotionTaskResultCode::TASK_RESULT_CANCELED ||
    report_->result_code == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED ||
    report_->result_code == MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR ||
    report_->result_code == MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE)
  {
    return false;
  }
  if (support_.interruption_checker()) {
    const auto interrupted = support_.interruption_checker()();
    if (interrupted != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      report_->result_code = interrupted;
      return false;
    }
  }
  if (!support_.motion_port()->retryPlaceRetreat()) {return false;}
  state_.retreat_replanned = true;
  report_->failure_stage = "place_retract_replanning";
  return true;
}

std::uint8_t PlaceRecovery::execute_retreat_retry(std::uint8_t motion_result)
{
  report_->result_code = motion_result;
  return report_->result_code;
}

std::uint8_t PlaceRecovery::finalize_retreat()
{
  if (state_.retreat_replanned &&
    report_->result_code != MotionTaskResultCode::TASK_RESULT_SUCCESS &&
    report_->result_code != MotionTaskResultCode::TASK_RESULT_CANCELED &&
    report_->result_code != MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED &&
    report_->result_code != MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR &&
    report_->result_code != MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE)
  {
    report_->result_code = state_.original_retreat_result;
  }
  return report_->result_code;
}

void PlaceRecovery::register_nodes(
  BT::BehaviorTreeFactory & factory, const ExecuteTask::Goal & goal,
  TaskExecutionReport & report)
{
  state_.goal = goal;
  report_ = &report;
  register_callback_action(
    factory, "PreparePlaceHomeRetry", [this]() {
      if (!approach_recovery_eligible() ||
      state_.home_retries >= support_.geometry().place_home_retry_count)
      {
        return BT::NodeStatus::FAILURE;
      }
      if (support_.interruption_checker() && support_.interruption_checker()() !=
      MotionTaskResultCode::TASK_RESULT_SUCCESS) {return BT::NodeStatus::FAILURE;}
      if (!support_.gripper()) {
        report_->result_code = MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
        report_->failure_stage = "payload_home:gripper_unavailable";
        return BT::NodeStatus::FAILURE;
      }
      const auto held = support_.gripper()->holding();
      if (!held.fresh() || held.state != HoldingState::HELD) {
        report_->failure_stage = "place_home_retry:holding_unconfirmed";
        return BT::NodeStatus::FAILURE;
      }
      ++state_.home_retries;
      state_.original_result = report_->result_code;
      return BT::NodeStatus::SUCCESS;
    });
  payload_home_operation_ = [this]() {
      if (support_.interruption_checker()) {
        const auto interrupted = support_.interruption_checker()();
        if (interrupted != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
          report_->result_code = interrupted;
          report_->failure_stage = "payload_home:interrupted";
          return BT::NodeStatus::FAILURE;
        }
      }
      if (!support_.gripper()) {
        report_->result_code = MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
        report_->failure_stage = "payload_home:gripper_unavailable";
        return BT::NodeStatus::FAILURE;
      }
      const auto held = support_.gripper()->holding();
      if (!held.fresh() || held.state != HoldingState::HELD) {
        report_->failure_stage = "payload_home:holding_unconfirmed";
        report_->result_code = MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE;
        return BT::NodeStatus::FAILURE;
      }
      auto request = support_.make_motion_request(
        state_.goal,
        arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING,
        report_->motion_execution_id);
      report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_HELD;
      request.segment = MotionSegmentKind::PAYLOAD_HOME;
      // The backend owns the home joint target. Supply a valid pose envelope
      // for MotionPort admission; it is not used as the home planning target.
      request.constraints.orientation_symmetry_order = MotionOrientationSymmetry::UNIQUE;
      request.has_unobserved_yaw_fallback = false;
      request.unobserved_yaw_fallback_rad = 0.0;
      request.target_pose = geometry_msgs::msg::PoseStamped{};
      request.target_pose.header.frame_id = "base_link";
      request.target_pose.pose.orientation.w = 1.0;
      report_->failure_stage = "payload_home:planning";
      const auto planned = support_.plan_motion_request(request);
      report_->result_code = planned.result_code;
      if (planned.result_code != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
        return BT::NodeStatus::FAILURE;
      }
      const auto before_execution = support_.gripper()->holding();
      if (!before_execution.fresh() || before_execution.state != HoldingState::HELD) {
        report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
        report_->result_code = MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE;
        (void)support_.cancel_active_motion(request.execution_id);
        report_->failure_stage = "payload_home:holding_lost_before_execution";
        return BT::NodeStatus::FAILURE;
      }
      report_->failure_stage = "payload_home:execution";
      const auto executed = support_.execute_planned_motion(planned.plan);
      report_->result_code = executed.result_code;
      if (executed.result_code == MotionTaskResultCode::TASK_RESULT_SUCCESS) {
        const auto after = support_.gripper()->holding();
        if (!after.fresh() || after.state != HoldingState::HELD) {
          report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
          report_->result_code = MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE;
          report_->failure_stage = "payload_home:holding_lost";
        }
      }
      return domain_status(report_->result_code);
    };
  if (support_.geometry().payload_home_bt_plugin.empty()) {
    factory.registerNodeType<PayloadHomeAction>("ReturnHomeWithPayload");
  } else {
    factory.registerFromPlugin(support_.geometry().payload_home_bt_plugin);
  }
  register_callback_action(
    factory, "PreparePlaceApproachReturn", [this]() {
      if (!approach_recovery_eligible()) {return BT::NodeStatus::FAILURE;}
      if (!support_.gripper()) {return BT::NodeStatus::FAILURE;}
      const auto held = support_.gripper()->holding();
      if (!held.fresh() || held.state != HoldingState::HELD) {
        report_->result_code = MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE;
        report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
        report_->failure_stage = "place_return:holding_unconfirmed";
        return BT::NodeStatus::FAILURE;
      }
      state_.original_result = report_->result_code;
      state_.return_needed = true;
      return BT::NodeStatus::SUCCESS;
    });
  register_callback_action(
    factory, "PlaceApproachRecoveryEligible", [this]() {
      return approach_recovery_eligible() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  register_callback_action(
    factory, "PrepareHeldPlaceFailureRecovery", [this]() {
      prepare_held_failure_recovery();
      return BT::NodeStatus::SUCCESS;
    });
  register_callback_action(
    factory, "PlaceReturnRecoveryNeeded", [this]() {
      return state_.return_needed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  register_callback_action(
    factory, "PlaceReturnNotNeeded", [this]() {
      return !state_.return_needed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  register_callback_action(
    factory, "ReturnHeldPlaceHome", [this]() {
      return domain_status(return_held_home());
    });
  register_callback_action(
    factory, "FinalizeHeldPlaceReturn", [this]() {
      return domain_status(finalize_held_return());
    });
  register_callback_action(
    factory, "PreparePlaceRetreatRetry", [this]() {
      report_->failure_stage = "place_retract_replanning";
      state_.pending_motion_request = support_.make_motion_request(
        state_.goal, arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING,
        report_->motion_execution_id);
      return BT::NodeStatus::SUCCESS;
    });
  register_plan_motion_action(
    factory, "PlanPlaceRetreatRetry", support_,
    [this]() {return state_.pending_motion_request;},
    arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING,
    [this](const MotionPlanningResult & result) {
      if (result.result_code != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
        return domain_status(execute_retreat_retry(result.result_code));
      }
      state_.pending_motion_plan = result.plan;
      return BT::NodeStatus::SUCCESS;
    });
  register_execute_planned_motion_action(
    factory, "ExecutePlaceRetreatRetry", support_,
    [this]() {return state_.pending_motion_plan;}, true,
    [this](const MotionPortResult & result) {
      return domain_status(execute_retreat_retry(result.result_code));
    });
  register_callback_action(
    factory, "AdvancePlaceRetreatRecovery", [this]() {
      return advance_retreat_recovery() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  register_callback_action(
    factory, "FinalizePlaceRetreat", [this]() {
      return domain_status(finalize_retreat());
    });
}

}  // namespace arm_cell_task_execution
