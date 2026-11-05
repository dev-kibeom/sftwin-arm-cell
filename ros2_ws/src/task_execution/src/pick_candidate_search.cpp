#include "arm_cell_task_execution/task_execution_modules.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <unordered_set>

#include "arm_cell_task_execution/bt_callback_node.hpp"
#include "arm_cell_motion/pick_rectangle_geometry.hpp"
#include "arm_cell_task_execution/task_execution_validation.hpp"

namespace arm_cell_task_execution
{
using namespace arm_cell_motion;
using namespace task_execution_validation;

std::uint8_t PickCandidateSearch::prepare(
  const ExecuteTask::Goal & goal, TaskExecutionReport & report)
{
  report.object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNCHANGED;
  state_.attempted_candidate_ids.clear();
  state_.outcome_material_candidate_id.clear();
  state_.ordered_candidates.clear();
  state_.attempts.clear();
  state_.candidate_cursor.reset({});
  state_.recording_enabled = !goal.material_candidates.empty();
  state_.direct_pick_pending = goal.material_candidates.empty();
  report.result_code =
    MotionTaskResultCode::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED;
  support_.motion_port()->beginPickSearch(support_.execution_identity());
  if (state_.direct_pick_pending) {
    state_.outcome_material_candidate_id = goal.vision_candidate_id;
    return MotionTaskResultCode::TASK_RESULT_SUCCESS;
  }

  auto candidates = goal.material_candidates;
  const auto acquisition_id = candidates.front().acquisition_id;
  std::unordered_set<std::string> candidate_ids;
  for (const auto & candidate : candidates) {
    if (acquisition_id.empty() || candidate.acquisition_id != acquisition_id ||
      candidate.candidate_id.empty() || !candidate_ids.insert(candidate.candidate_id).second)
    {
      report.failure_stage = "validation:invalid_material_candidate_list";
      state_.recording_enabled = false;
      report.result_code = MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
      return report.result_code;
    }
  }

  for (const auto & candidate : candidates) {
    std::string invalid_reason;
    const bool valid_orientation =
      candidate.orientation_symmetry_order == ExecuteTask::Goal::ORIENTATION_UNOBSERVABLE ||
      candidate.orientation_symmetry_order == ExecuteTask::Goal::ORIENTATION_UNIQUE ||
      candidate.orientation_symmetry_order == ExecuteTask::Goal::ORIENTATION_TWOFOLD ||
      candidate.orientation_symmetry_order == ExecuteTask::Goal::ORIENTATION_FOURFOLD;
    const bool valid_pose_value = candidate.orientation_symmetry_order ==
      ExecuteTask::Goal::ORIENTATION_UNOBSERVABLE ?
      valid_observation_position(candidate.target_pose) : valid_pose(candidate.target_pose);
    const bool valid_frame = candidate.target_pose.header.frame_id == "base_link";
    const bool fallback_consistent = candidate.orientation_symmetry_order ==
      ExecuteTask::Goal::ORIENTATION_UNOBSERVABLE ?
      (goal.has_unobserved_yaw_fallback && finite(goal.unobserved_yaw_fallback_rad)) :
      (!goal.has_unobserved_yaw_fallback && goal.unobserved_yaw_fallback_rad == 0.0);
    const bool valid_height = !candidate.has_estimated_height ||
      (std::isfinite(candidate.estimated_height_m) && candidate.estimated_height_m > 0.0F);
    const bool rectangle_geometry_required = candidate.has_observed_rectangle ||
      candidate.rectangle_long_side_m != 0.0F || candidate.rectangle_short_side_m != 0.0F ||
      candidate.orientation_symmetry_order == ExecuteTask::Goal::ORIENTATION_TWOFOLD ||
      candidate.orientation_symmetry_order == ExecuteTask::Goal::ORIENTATION_FOURFOLD;
    const auto rectangle_geometry_status = classify_rectangle_geometry(
      rectangle_geometry_required, candidate.has_estimated_height,
      candidate.rectangle_long_side_m, candidate.rectangle_short_side_m,
      candidate.estimated_height_m);
    const bool valid_rectangle =
      rectangle_geometry_status != RectangleGeometryStatus::INVALID &&
      (candidate.orientation_symmetry_order != ExecuteTask::Goal::ORIENTATION_TWOFOLD ||
      candidate.rectangle_long_side_m > candidate.rectangle_short_side_m);
    if (!valid_orientation) {
      invalid_reason = "unsupported_orientation_symmetry_order";
    } else if (!valid_frame) {
      invalid_reason = "invalid_target_frame";
    } else if (!valid_pose_value) {
      invalid_reason = "invalid_target_pose";
    } else if (!fallback_consistent) {
      invalid_reason = "inconsistent_orientation_fallback";
    } else if (!valid_height) {
      invalid_reason = "invalid_estimated_height";
    } else if (!valid_rectangle) {
      invalid_reason = "invalid_rectangle_geometry";
    }
    if (invalid_reason.empty()) {
      state_.ordered_candidates.push_back(candidate);
    } else {
      state_.attempts.push_back(
        MaterialCandidateSearchAttempt{
          candidate.candidate_id, MotionTaskResultCode::TASK_RESULT_INVALID_GOAL,
          "validation:" + invalid_reason, false, false});
    }
  }
  if (state_.ordered_candidates.empty()) {
    state_.outcome_material_candidate_id =
      state_.attempts.empty() ? std::string{} :
    state_.attempts.back().candidate_id;
    report.failure_stage = "validation:no_valid_material_candidate";
    report.result_code = MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
    return report.result_code;
  }

  // Tray +Y points toward the robot in the installed base_link convention.
  // The shared policy keeps selection separate from Motion feasibility checks.
  std::vector<CandidatePriority> priorities;
  priorities.reserve(state_.ordered_candidates.size());
  for (std::size_t index = 0; index < state_.ordered_candidates.size(); ++index) {
    const auto & candidate = state_.ordered_candidates[index];
    priorities.push_back({
      index, candidate.target_pose.pose.position.y,
      candidate.target_pose.pose.position.z, candidate.candidate_id});
  }
  state_.candidate_cursor.reset(order_candidate_indices(priorities, priority_policy_));
  return MotionTaskResultCode::TASK_RESULT_SUCCESS;
}

bool PickCandidateSearch::select_next(
  const ExecuteTask::Goal & goal, BT::Blackboard & blackboard)
{
  bool validated = false;
  if (state_.direct_pick_pending) {
    state_.direct_pick_pending = false;
    state_.selected_goal = goal;
    state_.outcome_material_candidate_id = goal.vision_candidate_id;
  } else {
    std::size_t candidate_index = 0;
    if (!state_.candidate_cursor.select_next(candidate_index)) {return false;}
    const auto & candidate = state_.ordered_candidates[candidate_index];
    state_.selected_goal = goal;
    // Preserve the complete acquisition so Motion can derive the other
    // candidates' BOX occupancy for this selected target.
    state_.selected_goal.vision_acquisition_id = candidate.acquisition_id;
    state_.selected_goal.vision_candidate_id = candidate.candidate_id;
    state_.selected_goal.target_pose = candidate.target_pose;
    state_.selected_goal.has_target_pose = true;
    state_.selected_goal.orientation_symmetry_order = candidate.orientation_symmetry_order;
    state_.selected_goal.estimated_object_height_m = candidate.estimated_height_m;
    state_.selected_goal.has_estimated_object_height = candidate.has_estimated_height;
    state_.selected_goal.has_observed_rectangle = candidate.has_observed_rectangle;
    state_.selected_goal.rectangle_long_side_m = candidate.rectangle_long_side_m;
    state_.selected_goal.rectangle_short_side_m = candidate.rectangle_short_side_m;
    state_.attempted_candidate_ids.push_back(candidate.candidate_id);
    state_.outcome_material_candidate_id = candidate.candidate_id;
    validated = true;
  }
  state_.candidate_active = true;
  support_.motion_port()->beginPickCandidateSearch();
  blackboard.set("pick_candidate_goal", state_.selected_goal);
  blackboard.set("pick_candidate_inputs_validated", validated);
  return true;
}

void PickCandidateSearch::record_attempt(
  std::uint8_t result, const std::string & failure_stage, bool insertion_started)
{
  if (!state_.candidate_active) {return;}
  state_.attempts.push_back(
    MaterialCandidateSearchAttempt{
      state_.selected_goal.vision_candidate_id, result, failure_stage, insertion_started});
  state_.last_result = result;
  state_.candidate_active = false;
}

bool PickCandidateSearch::transition_allowed() const
{
  return state_.last_result != MotionTaskResultCode::TASK_RESULT_SUCCESS &&
         !support_.motion_port()->pickGraspInsertionStarted() &&
         (state_.last_result == MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE ||
         state_.last_result == MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED ||
         state_.last_result == MotionTaskResultCode::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED);
}

void PickCandidateSearch::record_search()
{
  if (state_.recording_enabled) {
    support_.motion_port()->recordMaterialCandidateSearch(
      state_.attempts, state_.outcome_material_candidate_id);
  }
}

const std::vector<std::string> & PickCandidateSearch::attempted_candidate_ids() const
{return state_.attempted_candidate_ids;}

const std::string & PickCandidateSearch::selected_candidate_id() const
{return state_.outcome_material_candidate_id;}

void PickCandidateSearch::register_nodes(
  BT::BehaviorTreeFactory & factory, BT::Blackboard::Ptr blackboard,
  TaskExecutionReport & report)
{
  blackboard_ = std::move(blackboard);
  report_ = &report;
  register_callback_action(
    factory, "PreparePickSearch", [this]() {
      report_->result_code = prepare(blackboard_->get<ExecuteTask::Goal>("goal"), *report_);
      state_.last_result = report_->result_code;
      return BT::NodeStatus::SUCCESS;
    });
  register_callback_action(
    factory, "SelectNextPickCandidate", [this]() {
      if (select_next(blackboard_->get<ExecuteTask::Goal>("goal"), *blackboard_)) {
        return BT::NodeStatus::SUCCESS;
      }
      report_->result_code = state_.last_result;
      return BT::NodeStatus::FAILURE;
    });
  register_callback_action(
    factory, "RecordPickCandidateSuccess", [this]() {
      record_attempt(
        report_->result_code, report_->failure_stage,
        support_.motion_port()->pickGraspInsertionStarted());
      return BT::NodeStatus::SUCCESS;
    });
  register_callback_action(
    factory, "CanTransitionPickCandidate", [this]() {
      record_attempt(
        report_->result_code, report_->failure_stage,
        support_.motion_port()->pickGraspInsertionStarted());
      const auto & interrupted = support_.interruption_checker();
      if (interrupted) {
        const auto result = interrupted();
        if (result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
          report_->result_code = result;
          return BT::NodeStatus::FAILURE;
        }
      }
      const bool allowed = transition_allowed();
      if (!allowed) {report_->result_code = state_.last_result;}
      return allowed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
}

}  // namespace arm_cell_task_execution
