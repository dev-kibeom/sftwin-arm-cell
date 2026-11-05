#include "arm_cell_task_execution/task_execution_modules.hpp"

#include <rclcpp/rclcpp.hpp>

#include "arm_cell_task_execution/bt_callback_node.hpp"
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
  return (place_approach_failure || place_clearance_failure) &&
         (report_->result_code ==
         MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE ||
         report_->result_code ==
         MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED ||
         report_->result_code ==
         MotionTaskResultCode::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED);
}

bool PlaceRecovery::observe_approach_recovery_holding()
{
  state_.approach_completed = false;
  state_.approach_return_open_needed = false;
  const auto observed = support_.gripper()->holding();
  if (observed.fresh() && observed.state == HoldingState::RELEASED) {
    support_.motion_port()->clearGraspRelationAfterRelease();
    support_.object_state_tracker().mark_detached();
    report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED;
    state_.approach_completed = true;
    return true;
  }
  if (observed.fresh() && observed.state == HoldingState::HELD) {
    state_.approach_return_open_needed = true;
    return true;
  }
  report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
  return false;
}

std::uint8_t PlaceRecovery::open_approach_return()
{
  std::uint64_t baseline = 0;
  report_->failure_stage = "place_approach_return_open";
  const auto opened = support_.execute_gripper_open(baseline);
  if (opened != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
    report_->result_code = opened;
    return opened;
  }
  state_.holding_baseline = baseline;
  return opened;
}

std::uint8_t PlaceRecovery::confirm_approach_return()
{
  const auto released = support_.fresh_holding(
    HoldingState::RELEASED, state_.holding_baseline);
  if (released != HoldingWaitResult::CONFIRMED) {
    report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
    report_->failure_stage = "place_approach_return_release_confirmation";
    report_->result_code = released == HoldingWaitResult::CANCELED ?
      MotionTaskResultCode::TASK_RESULT_CANCELED :
      released == HoldingWaitResult::SAFETY_PREEMPTED ?
      MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED :
      released == HoldingWaitResult::BACKEND_UNAVAILABLE ?
      MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE :
      MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE;
    return report_->result_code;
  }
  support_.motion_port()->clearGraspRelationAfterRelease();
  support_.object_state_tracker().mark_detached();
  report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED;
  state_.approach_completed = true;
  return report_->result_code;
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
    factory, "PlaceApproachRecoveryEligible", [this]() {
      return approach_recovery_eligible() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  register_callback_action(
    factory, "ObservePlaceApproachRecoveryHolding", [this]() {
      return observe_approach_recovery_holding() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  register_callback_action(
    factory, "PlaceApproachRecoveryComplete", [this]() {
      return state_.approach_completed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  register_callback_action(
    factory, "PlaceApproachReturnOpenRequired", [this]() {
      return state_.approach_return_open_needed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  register_callback_action(
    factory, "OpenPlaceApproachReturn", [this]() {
      return open_approach_return() == MotionTaskResultCode::TASK_RESULT_SUCCESS ?
      BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  register_callback_action(
    factory, "ConfirmPlaceApproachReturn", [this]() {
      (void)confirm_approach_return();
      return state_.approach_completed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
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
