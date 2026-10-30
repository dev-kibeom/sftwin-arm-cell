#include "arm_cell_task_execution/task_execution_modules.hpp"

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <arm_cell_interfaces/msg/motion_task_phase.hpp>

#include "arm_cell_task_execution/bt_callback_node.hpp"
#include "arm_cell_task_execution/motion_plan_execute_action.hpp"
#include "arm_cell_motion/candidate_occupancy.hpp"
#include "arm_cell_motion/motion_result_code_name.hpp"
#include "arm_cell_task_execution/pick_execution_record.hpp"
#include "arm_cell_motion/pick_rectangle_geometry.hpp"
#include "arm_cell_task_execution/task_execution_validation.hpp"

namespace arm_cell_task_execution
{
using namespace arm_cell_motion;
using namespace task_execution_validation;

namespace
{
BT::NodeStatus domain_status(std::uint8_t result)
{
  return result == MotionTaskResultCode::TASK_RESULT_SUCCESS ?
         BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
}  // namespace

std::uint8_t PickExecution::record_stage(
  std::uint8_t result, const std::string & stage, bool held, bool retract_succeeded)
{
  report_->failure_stage = result == MotionTaskResultCode::TASK_RESULT_SUCCESS ?
    std::string{} : stage + ":" + motion_result_code_name(result);
  queue_pick_execution_record(
    state_.goal, report_->motion_execution_id, stage, result, held, retract_succeeded,
    state_.approach_submitted ? support_.motion_port()->lastPickCandidateEvaluation() : "");
  report_->result_code = result;
  return result;
}

uint8_t PickExecution::prepare_candidate(bool candidate_inputs_validated)
{
  const auto & goal = state_.goal;
  state_.approach_submitted = false;
  state_.candidate_inputs_validated = candidate_inputs_validated;
  const bool valid_pick_approach =
    goal.pick_approach_translation_direction ==
    ExecuteTask::Goal::PICK_APPROACH_TCP_INSERTION_AXIS ||
    goal.pick_approach_translation_direction ==
    ExecuteTask::Goal::PICK_APPROACH_PLANNING_FRAME_VERTICAL;
  const bool valid_pick_grasp_envelope =
    finite(goal.pick_approach_distance_m) && goal.pick_approach_distance_m > 0.0 &&
    finite(goal.pick_grasp_position_tolerance_m) &&
    goal.pick_grasp_position_tolerance_m > 0.0 &&
    finite(goal.pick_grasp_orientation_tolerance_rad) &&
    goal.pick_grasp_orientation_tolerance_rad > 0.0 &&
    goal.pick_grasp_orientation_tolerance_rad <= 3.14159265358979323846;
  const bool rectangle_geometry_required = goal.has_observed_rectangle ||
    goal.rectangle_long_side_m != 0.0F || goal.rectangle_short_side_m != 0.0F ||
    goal.orientation_symmetry_order == ExecuteTask::Goal::ORIENTATION_TWOFOLD ||
    goal.orientation_symmetry_order == ExecuteTask::Goal::ORIENTATION_FOURFOLD;
  const auto rectangle_geometry_status = classify_rectangle_geometry(
    rectangle_geometry_required, goal.has_estimated_object_height,
    goal.rectangle_long_side_m, goal.rectangle_short_side_m, goal.estimated_object_height_m);
  const bool rectangle_geometry_available =
    rectangle_geometry_status == RectangleGeometryStatus::AVAILABLE;
  const bool valid_grasp_width = !rectangle_geometry_available ||
    (finite(goal.grasp_width_min_mm) && finite(goal.grasp_width_max_mm) &&
    goal.grasp_width_min_mm > 0.0F && goal.grasp_width_max_mm >= goal.grasp_width_min_mm &&
    goal.grasp_width_max_mm <= 85.0F);
  if (!valid_pick_approach || !valid_pick_grasp_envelope || !valid_grasp_width ||
    goal.vision_acquisition_id.empty() || !goal.has_target_pose ||
    support_.object_state_tracker().state() != MotionObjectState::NO_OBJECT ||
    !support_.geometry().valid())
  {
    return record_stage(
      MotionTaskResultCode::TASK_RESULT_INVALID_GOAL, "validation", false, false);
  }
  if (!state_.candidate_inputs_validated) {
    const bool supported_order = goal.orientation_symmetry_order ==
      ExecuteTask::Goal::ORIENTATION_UNOBSERVABLE || goal.orientation_symmetry_order ==
      ExecuteTask::Goal::ORIENTATION_UNIQUE || goal.orientation_symmetry_order ==
      ExecuteTask::Goal::ORIENTATION_TWOFOLD || goal.orientation_symmetry_order ==
      ExecuteTask::Goal::ORIENTATION_FOURFOLD;
    const bool fallback_consistent = goal.orientation_symmetry_order ==
      ExecuteTask::Goal::ORIENTATION_UNOBSERVABLE ?
      (goal.has_unobserved_yaw_fallback && finite(goal.unobserved_yaw_fallback_rad)) :
      (!goal.has_unobserved_yaw_fallback && goal.unobserved_yaw_fallback_rad == 0.0);
    const bool valid_observation = goal.orientation_symmetry_order ==
      ExecuteTask::Goal::ORIENTATION_UNOBSERVABLE ?
      valid_observation_position(goal.target_pose) : valid_pose(goal.target_pose);
    const bool valid_rectangle = rectangle_geometry_status != RectangleGeometryStatus::INVALID &&
      (goal.orientation_symmetry_order != ExecuteTask::Goal::ORIENTATION_TWOFOLD ||
      goal.rectangle_long_side_m > goal.rectangle_short_side_m);
    if (!supported_order || !fallback_consistent || !valid_observation || !valid_rectangle)
    {
      return record_stage(
        MotionTaskResultCode::TASK_RESULT_INVALID_GOAL, "validation", false, false);
    }
    if (!valid_candidate_occupancy(
        goal.has_non_target_occupancy, goal.non_target_occupancy, goal.target_pose))
    {
      return record_stage(
        MotionTaskResultCode::TASK_RESULT_INVALID_GOAL, "occupancy_validation", false, false);
    }
  }
  report_->result_code = MotionTaskResultCode::TASK_RESULT_SUCCESS;
  return report_->result_code;
}

uint8_t PickExecution::open_and_confirm_release()
{
  if (state_.released_confirmed) {
    return MotionTaskResultCode::TASK_RESULT_SUCCESS;
  }
  auto result = support_.execute_gripper_open(state_.holding_baseline);
  if (result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    support_.motion_port()->discardPendingGraspRelation();
    return record_stage(result, "gripper_open", false, false);
  }
  const auto release = support_.fresh_holding(
    HoldingState::RELEASED,
    state_.holding_baseline);
  if (release != HoldingWaitResult::CONFIRMED) {
    result = release == HoldingWaitResult::CANCELED ?
      MotionTaskResultCode::TASK_RESULT_CANCELED :
      release == HoldingWaitResult::SAFETY_PREEMPTED ?
      MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED :
      release == HoldingWaitResult::BACKEND_UNAVAILABLE ?
      MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE :
      MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE;
    support_.motion_port()->discardPendingGraspRelation();
    return record_stage(result, "released_confirmation", false, false);
  }
  state_.released_confirmed = true;
  return MotionTaskResultCode::TASK_RESULT_SUCCESS;
}

uint8_t PickExecution::execute_approach(std::uint8_t result)
{
  state_.approach_submitted = true;
  report_->failure_stage = "approach";
  if (result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    support_.motion_port()->discardPendingGraspRelation();
    return record_stage(result, "approach", false, false);
  }
  return result;
}

uint8_t PickExecution::execute_grasp_insertion(std::uint8_t result)
{
  report_->failure_stage = "grasp_motion";
  if (result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    support_.motion_port()->discardPendingGraspRelation();
    return record_stage(result, "grasp_motion", false, false);
  }
  report_->result_code = result;
  return result;
}

uint8_t PickExecution::close_and_confirm_grasp()
{
  auto result = support_.execute_gripper_close(state_.holding_baseline);
  if (result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    support_.motion_port()->discardPendingGraspRelation();
    return record_stage(result, "gripper_close", false, false);
  }
  const auto holding = support_.fresh_holding(
    HoldingState::HELD,
    state_.holding_baseline);
  if (holding != HoldingWaitResult::CONFIRMED) {
    if (holding == HoldingWaitResult::CANCELED) {
      result = MotionTaskResultCode::TASK_RESULT_CANCELED;
    } else if (holding == HoldingWaitResult::SAFETY_PREEMPTED) {
      result = MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED;
    } else if (holding == HoldingWaitResult::BACKEND_UNAVAILABLE) {
      result = MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
    } else {
      const auto observed = support_.gripper()->holding();
      constexpr double kFullyClosedWidthMm = 0.5;
      if (observed.fresh() && observed.width_mm && std::isfinite(*observed.width_mm) &&
        *observed.width_mm >= 0.0 && *observed.width_mm <= kFullyClosedWidthMm)
      {
        std::uint64_t release_baseline = observed.sequence;
        if (support_.execute_gripper_open(release_baseline) ==
          MotionTaskResultCode::TASK_RESULT_SUCCESS)
        {
          (void)support_.fresh_holding(HoldingState::RELEASED, release_baseline);
        }
      }
      result = MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE;
    }
    support_.motion_port()->discardPendingGraspRelation();
    return record_stage(result, "held_confirmation", false, false);
  }
  report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_HELD;
  state_.released_confirmed = false;
  if (support_.phase_callback()) {
    support_.phase_callback()(
      arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_ATTACHING, 0, 0,
      0);
  }
  support_.object_state_tracker().mark_attached();
  if (!support_.motion_port()->commitPendingGraspRelation()) {
    support_.motion_port()->discardPendingGraspRelation();
    return record_stage(
      MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE,
      "grasp_relation_confirmation", true, false);
  }
  return MotionTaskResultCode::TASK_RESULT_SUCCESS;
}

uint8_t PickExecution::execute_retract(std::uint8_t result)
{
  if (result == MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    const auto terminal_holding = support_.fresh_holding(
      HoldingState::HELD,
      state_.holding_baseline);
    if (terminal_holding != HoldingWaitResult::CONFIRMED) {
      report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
      state_.holding_confirmation_failed = true;
      report_->failure_stage = "retract_held_confirmation";
      if (terminal_holding == HoldingWaitResult::CANCELED) {
        result = MotionTaskResultCode::TASK_RESULT_CANCELED;
      } else if (terminal_holding == HoldingWaitResult::SAFETY_PREEMPTED) {
        result = MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED;
      } else if (terminal_holding == HoldingWaitResult::BACKEND_UNAVAILABLE) {
        result = MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
      } else {
        result = MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE;
      }
      report_->result_code = result;
      return result;
    }
    report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_HELD;
    report_->result_code = result;
    return result;
  }
  if (result != MotionTaskResultCode::TASK_RESULT_SUCCESS &&
    result != MotionTaskResultCode::TASK_RESULT_CANCELED &&
    result != MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED &&
    result != MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR &&
    result != MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE)
  {
    if (result != MotionTaskResultCode::TASK_RESULT_SUCCESS &&
      report_->object_disposition == ExecuteTask::Result::OBJECT_DISPOSITION_HELD &&
      support_.interruption_checker() &&
      support_.interruption_checker()() != MotionTaskResultCode::TASK_RESULT_SUCCESS)
    {
      report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
    }
  }
  report_->result_code = result;
  return result;
}

std::uint8_t PickExecution::finalize_retract()
{
  const auto result = report_->result_code;
  return record_stage(
    result, state_.holding_confirmation_failed ? "retract_held_confirmation" : "retract",
    true, result == MotionTaskResultCode::TASK_RESULT_SUCCESS);
}

void PickExecution::register_nodes(
  BT::BehaviorTreeFactory & factory, BT::Blackboard::Ptr blackboard,
  TaskExecutionReport & report)
{
  blackboard_ = std::move(blackboard);
  report_ = &report;
  const auto set_result = [this](std::uint8_t result) {
      report_->result_code = result; return result;
    };
  register_callback_action(
    factory, "PreparePickCandidate", [this, set_result]() {
      state_.goal = blackboard_->get<ExecuteTask::Goal>("pick_candidate_goal");
      const bool validated = blackboard_->get<bool>("pick_candidate_inputs_validated");
      return domain_status(set_result(prepare_candidate(validated)));
    });
  register_callback_action(
    factory, "OpenAndConfirmPickRelease", [this, set_result]() {
      return domain_status(set_result(open_and_confirm_release()));
    });
  register_plan_motion_action(
    factory, "PlanPickApproach", support_,
    [this]() {
      state_.approach_submitted = true;
      report_->failure_stage = "approach";
      return support_.make_motion_request(
        state_.goal, arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING,
        report_->motion_execution_id);
    }, arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING,
    [this, set_result](const MotionPlanningResult & result) {
      if (result.result_code != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
        return domain_status(set_result(execute_approach(result.result_code)));
      }
      state_.pending_motion_plan = result.plan;
      return BT::NodeStatus::SUCCESS;
    });
  register_execute_planned_motion_action(
    factory, "ExecutePickApproach", support_,
    [this]() {return state_.pending_motion_plan;}, true,
    [this, set_result](const MotionPortResult & result) {
      return domain_status(set_result(execute_approach(result.result_code)));
    });
  register_plan_motion_action(
    factory, "PlanPickGraspInsertion", support_,
    [this]() {
      report_->failure_stage = "grasp_motion";
      return support_.make_motion_request(
        state_.goal, arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING,
        report_->motion_execution_id);
    }, arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING,
    [this, set_result](const MotionPlanningResult & result) {
      if (result.result_code != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
        return domain_status(set_result(execute_grasp_insertion(result.result_code)));
      }
      state_.pending_motion_plan = result.plan;
      return BT::NodeStatus::SUCCESS;
    });
  register_execute_planned_motion_action(
    factory, "ExecutePickGraspInsertion", support_,
    [this]() {return state_.pending_motion_plan;}, true,
    [this, set_result](const MotionPortResult & result) {
      return domain_status(set_result(execute_grasp_insertion(result.result_code)));
    });
  register_callback_action(
    factory, "AdvancePickPlanningCandidate", [this]() {
      const auto & interrupted = support_.interruption_checker();
      if (interrupted) {
        const auto result = interrupted();
        if (result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
          report_->result_code = result;
          return BT::NodeStatus::FAILURE;
        }
      }
      if (!support_.motion_port()->advanceToNextPlanningCandidate()) {
        return BT::NodeStatus::FAILURE;
      }
      support_.motion_port()->beginPickGraspRelation();
      return BT::NodeStatus::SUCCESS;
    });
  register_callback_action(
    factory, "CloseAndConfirmPickGrasp", [this, set_result]() {
      return domain_status(set_result(close_and_confirm_grasp()));
    });
  register_plan_motion_action(
    factory, "PlanPickRetract", support_,
    [this]() {
      return support_.make_motion_request(
        state_.goal, arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING,
        report_->motion_execution_id);
    }, arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING,
    [this, set_result](const MotionPlanningResult & result) {
      if (result.result_code != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
        return domain_status(set_result(execute_retract(result.result_code)));
      }
      state_.pending_motion_plan = result.plan;
      return BT::NodeStatus::SUCCESS;
    });
  register_execute_planned_motion_action(
    factory, "ExecutePickRetract", support_,
    [this]() {return state_.pending_motion_plan;}, true,
    [this, set_result](const MotionPortResult & result) {
      return domain_status(set_result(execute_retract(result.result_code)));
    });
  register_callback_action(
    factory, "FinalizePickRetract", [this, set_result]() {
      return domain_status(set_result(finalize_retract()));
    });
}

}  // namespace arm_cell_task_execution
