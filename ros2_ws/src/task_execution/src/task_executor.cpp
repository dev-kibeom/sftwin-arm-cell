#include "arm_cell_task_execution/task_executor.hpp"

#include <chrono>
#include <exception>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <arm_cell_interfaces/msg/motion_task_phase.hpp>

#include "arm_cell_task_execution/motion_bt_executor.hpp"
#include "arm_cell_motion/motion_result_code_name.hpp"

namespace arm_cell_task_execution
{
using namespace arm_cell_motion;

namespace
{
bool is_planning_failure(std::uint8_t result)
{
  return result == MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE ||
         result == MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED ||
         result == MotionTaskResultCode::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED;
}

bool is_resolved_unplaced_disposition(std::uint8_t disposition)
{
  return disposition == ExecuteTask::Result::OBJECT_DISPOSITION_UNCHANGED ||
         disposition == ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED;
}

}  // namespace

TaskExecutor::TaskExecutor(
  std::shared_ptr<MotionPort> motion_port,
  std::shared_ptr<GripperPort> gripper_port,
  MotionGeometry geometry,
  InterruptionChecker interruption_checker,
  std::chrono::milliseconds holding_confirmation_timeout,
  PickPlaceStrategies strategies)
: TaskExecutor(std::make_shared<TaskExecutionSupport>(
    motion_port, std::move(gripper_port), std::move(geometry),
    holding_confirmation_timeout), strategies)
{
  support_->set_interruption_checker(std::move(interruption_checker));
}

TaskExecutor::TaskExecutor(
  std::shared_ptr<TaskExecutionSupport> support, PickPlaceStrategies strategies)
: support_(std::move(support)), strategies_(strategies),
  bt_executor_(std::make_unique<MotionBtExecutor>(*support_))
{
}

TaskExecutor::~TaskExecutor() = default;

void TaskExecutor::set_interruption_checker(InterruptionChecker interruption_checker)
{
  support_->set_interruption_checker(std::move(interruption_checker));
}

void TaskExecutor::set_phase_callback(PhaseCallback phase_callback)
{
  support_->set_phase_callback(std::move(phase_callback));
}

void TaskExecutor::set_runtime_geometry(
  MotionGeometry geometry, std::chrono::milliseconds holding_confirmation_timeout)
{
  support_->set_runtime_geometry(std::move(geometry), holding_confirmation_timeout);
}

void TaskExecutor::set_object_state(MotionObjectState object_state)
{
  support_->object_state_tracker().set_state(object_state);
}

MotionObjectState TaskExecutor::object_state() const
{
  return support_->object_state_tracker().state();
}

std::uint8_t TaskExecutor::execute(const ExecuteTask::Goal & goal)
{
  return execute(goal, object_state());
}

MotionDisturbResult TaskExecutor::execute_disturb(const MotionDisturbRequest & request)
{
  return bt_executor_->execute_disturb(request);
}

std::uint8_t TaskExecutor::execute(
  const ExecuteTask::Goal & goal, MotionObjectState object_state,
  const std::string & motion_execution_id)
{
  const auto execution_started = std::chrono::steady_clock::now();
  const auto motion = support_->motion_port();
  motion->resetPickGraspInsertionStarted();
  support_->set_execution_identity(motion_execution_id);
  set_object_state(object_state);
  attempted_material_candidate_ids_.clear();
  outcome_material_candidate_id_.clear();
  last_report_ = TaskExecutionReport{};
  last_report_.motion_execution_id = motion_execution_id;
  last_report_.object_disposition =
    goal.task_type.value == MotionTaskType::TASK_TYPE_GO_HOME ||
    goal.task_type.value == MotionTaskType::TASK_TYPE_RETRACT ?
    ExecuteTask::Result::OBJECT_DISPOSITION_UNCHANGED :
    ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
  if (support_->phase_callback()) {
    support_->phase_callback()(
      arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_VALIDATING, 0,
      0, 0);
  }

  try {
    auto execution = bt_executor_->execute(goal, motion_execution_id, last_report_, strategies_);
    attempted_material_candidate_ids_ = std::move(execution.attempted_material_candidate_ids);
    outcome_material_candidate_id_ = std::move(execution.outcome_material_candidate_id);
  } catch (const std::exception & error) {
    last_report_.failure_stage = "execution_worker:exception";
    try {
      motion->finalizeMotionPlanningDiagnostic(
        MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR,
        std::chrono::duration<double>(
          std::chrono::steady_clock::now() - execution_started).count(),
        last_report_.failure_stage, error.what());
    } catch (...) {
      // Diagnostic capture is best-effort and cannot replace the task exception.
    }
    throw;
  } catch (...) {
    last_report_.failure_stage = "execution_worker:exception";
    try {
      motion->finalizeMotionPlanningDiagnostic(
        MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR,
        std::chrono::duration<double>(
          std::chrono::steady_clock::now() - execution_started).count(),
        last_report_.failure_stage, "");
    } catch (...) {
      // Diagnostic capture is best-effort and cannot replace the task exception.
    }
    throw;
  }

  if (support_->phase_callback()) {
    support_->phase_callback()(
      arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_COMPLETING, 0,
      0, 0);
  }
  if (last_report_.result_code == MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    last_report_.failure_stage.clear();
  }
  if (goal.task_type.value == MotionTaskType::TASK_TYPE_PICK ||
    goal.task_type.value == MotionTaskType::TASK_TYPE_PLACE)
  {
    try {
      motion->finalizeMotionPlanningDiagnostic(
        last_report_.result_code, std::chrono::duration<double>(
          std::chrono::steady_clock::now() - execution_started).count(),
        last_report_.failure_stage, "");
    } catch (...) {
      // Diagnostic capture is best-effort and cannot replace the PnP result.
    }
  }

  return last_report_.result_code;
}

TaskExecutor::ExecutePnp::Result TaskExecutor::execute_pnp(
  const ExecutePnp::Goal & goal, const std::string & motion_execution_id,
  PhaseCallback phase_callback)
{
  ExecutePnp::Result result;
  result.result_code.value = MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
  result.object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNCHANGED;
  result.pnp_outcome = ExecutePnp::Result::PNP_OUTCOME_FAILED;
  if (goal.vision_acquisition_id.empty() || goal.material_candidates.empty() ||
    !goal.has_place_target_pose)
  {
    result.diagnostic_detail = "PnP goal requires an acquisition, candidates, and PLACE target";
    return result;
  }

  ExecuteTask::Goal pick;
  pick.task_type.value = MotionTaskType::TASK_TYPE_PICK;
  pick.material_candidates = goal.material_candidates;
  pick.vision_acquisition_id = goal.vision_acquisition_id;
  pick.pick_approach_translation_direction = goal.pick_approach_translation_direction;
  pick.pick_approach_distance_m = goal.pick_approach_distance_m;
  pick.pick_grasp_position_tolerance_m = goal.pick_grasp_position_tolerance_m;
  pick.pick_grasp_orientation_tolerance_rad = goal.pick_grasp_orientation_tolerance_rad;
  pick.grasp_width_min_mm = goal.grasp_width_min_mm;
  pick.grasp_width_max_mm = goal.grasp_width_max_mm;
  pick.has_unobserved_yaw_fallback = goal.has_unobserved_yaw_fallback;
  pick.unobserved_yaw_fallback_rad = goal.unobserved_yaw_fallback_rad;

  set_phase_callback(phase_callback);
  const auto pick_code = execute(pick, MotionObjectState::NO_OBJECT, motion_execution_id);
  result.result_code.value = pick_code;
  result.object_disposition = last_report_.object_disposition;
  result.diagnostic_detail = last_report_.failure_stage;
  result.attempted_candidate_ids = attempted_material_candidate_ids_;
  result.selected_candidate_id = outcome_material_candidate_id_;
  if (pick_code != MotionTaskResultCode::TASK_RESULT_SUCCESS ||
    result.object_disposition != ExecuteTask::Result::OBJECT_DISPOSITION_HELD)
  {
    if (pick_code == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED) {
      result.pnp_outcome = ExecutePnp::Result::PNP_OUTCOME_SAFETY_PREEMPTED;
    } else if (is_resolved_unplaced_disposition(result.object_disposition) &&
      (is_planning_failure(pick_code) ||
      pick_code == MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE ||
      pick_code == MotionTaskResultCode::TASK_RESULT_JOINT_PROGRESS_STALLED))
    {
      result.pnp_outcome = pick_code ==
        MotionTaskResultCode::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED ?
        ExecutePnp::Result::PNP_OUTCOME_CANDIDATES_EXHAUSTED :
        ExecutePnp::Result::PNP_OUTCOME_RETRYABLE;
    }
    return result;
  }

  ExecuteTask::Goal place;
  place.task_type.value = MotionTaskType::TASK_TYPE_PLACE;
  place.target_pose = goal.place_target_pose;
  place.has_target_pose = goal.has_place_target_pose;
  place.place_position_tolerance_m = goal.place_position_tolerance_m;
  place.place_orientation_tolerance_rad = goal.place_orientation_tolerance_rad;
  place.place_orientation_constraint = goal.place_orientation_constraint;
  place.place_approach_direction_object = goal.place_approach_direction_object;
  place.place_approach_distance_m = goal.place_approach_distance_m;
  place.place_retract_distance_m = goal.place_retract_distance_m;
  place.has_place_tool_orientation_preference = goal.has_place_tool_orientation_preference;
  place.place_tool_orientation_preference = goal.place_tool_orientation_preference;

  set_phase_callback(phase_callback);
  const auto place_code = execute(place, MotionObjectState::ATTACHED, motion_execution_id);
  result.result_code.value = place_code;
  result.object_disposition = last_report_.object_disposition;
  result.material_placed =
    result.object_disposition == ExecuteTask::Result::OBJECT_DISPOSITION_PLACED;
  result.diagnostic_detail = last_report_.failure_stage;
  if (place_code == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED) {
    result.pnp_outcome = ExecutePnp::Result::PNP_OUTCOME_SAFETY_PREEMPTED;
  } else if (place_code == MotionTaskResultCode::TASK_RESULT_SUCCESS && result.material_placed) {
    ExecuteTask::Goal home;
    home.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
    const auto home_code = execute(home, MotionObjectState::NO_OBJECT, motion_execution_id);
    result.result_code.value = home_code;
    result.diagnostic_detail = last_report_.failure_stage;
    if (home_code == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED) {
      result.pnp_outcome = ExecutePnp::Result::PNP_OUTCOME_SAFETY_PREEMPTED;
    } else if (home_code == MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      result.pnp_outcome = ExecutePnp::Result::PNP_OUTCOME_COMPLETED;
    }
  } else if (!result.material_placed &&
    is_resolved_unplaced_disposition(result.object_disposition) &&
    (is_planning_failure(place_code) ||
    place_code == MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE ||
    place_code == MotionTaskResultCode::TASK_RESULT_JOINT_PROGRESS_STALLED))
  {
    result.pnp_outcome = ExecutePnp::Result::PNP_OUTCOME_RETRYABLE;
  }
  return result;
}

bool TaskExecutor::pick_grasp_insertion_started() const
{
  const auto motion = support_->motion_port();
  return motion && motion->pickGraspInsertionStarted();
}

MotionCancelResult TaskExecutor::cancel_active_motion(
  const unique_identifier_msgs::msg::UUID & execution_id)
{return support_->cancel_active_motion(execution_id);}

void TaskExecutor::reset_candidate_search_result()
{
  attempted_material_candidate_ids_.clear();
  outcome_material_candidate_id_.clear();
}

}  // namespace arm_cell_task_execution
