#include "arm_cell_motion/pick_planning_diagnostics.hpp"
#include <nlohmann/json.hpp>
#include <cmath>
#include <limits>
namespace arm_cell_motion
{
namespace
{
double stage_elapsed(const PickPlanningAttempt & attempt, const std::string & stage)
{
  const auto found = attempt.stage_elapsed_s.find(stage);
  return found == attempt.stage_elapsed_s.end() ? 0.0 : found->second;
}
nlohmann::json attempt_record(
  const PickPlanningAttempt & attempt,
  const PickPlanningDiagnosticMetadata & metadata)
{
  const auto pose = attempt.current_pose_index;
  const auto seed = attempt.seed_index;
  const auto evaluation = attempt.attempt_evaluation;
  const auto & stage = attempt.failure_stage;
  const auto & reason = attempt.failure_reason;
  nlohmann::json record = {
    {"attempt", metadata.attempt_number}, {"seed", attempt.current_seed},
    {"grasp_pose_candidate", pose + 1U}, {"ik_seed_attempt", seed},
    {"stage", stage}, {"reason", reason.empty() ? "NONE" : reason},
    {"search_evaluation", stage == "INTERRUPTED" ? "INTERRUPTED" : stage == "SEARCH_BUDGET" ?
      "SEARCH_BUDGET_EXHAUSTED" : pick_search_evaluation_name(evaluation)},
    {"planning_bt_steps", metadata.bt_steps},
    {"executed_stages", attempt.executed_stages},
    {"planning_elapsed_s", attempt.approach_planning_elapsed_s},
    {"global_planning_requests", nlohmann::json::array()}};
  for (const auto & entry : attempt.planning_request_evidence) {
    record["global_planning_requests"].push_back(nlohmann::json::parse(entry));
  }
  const auto elapsed = [&](const std::string & name) {
      const auto entry = attempt.stage_elapsed_s.find(name);
      return entry == attempt.stage_elapsed_s.end() ? 0.0 : entry->second;
    };
  const bool grasp_called = attempt.stage_elapsed_s.count("SolveGraspIK") != 0U;
  const bool approach_called = attempt.stage_elapsed_s.count("SolveApproachIK") != 0U;
  const double grasp_elapsed = elapsed("SolveGraspIK");
  const double validation_elapsed = elapsed("ValidateGraspKinematics") + elapsed(
    "ValidateGraspContact") +
    elapsed("ValidateGraspCollision") + elapsed("ValidateApproachState");
  record["ik_elapsed_s"] = grasp_elapsed;
  record["validation_elapsed_s"] = validation_elapsed;
  double seed_distance_squared = 0.0;
  for (std::size_t joint = 0; joint < attempt.grasp_joint_positions.size(); ++joint) {
    const double delta = std::remainder(
      attempt.grasp_joint_positions[joint] - attempt.current_seed[joint],
      6.28318530717958647692);
    seed_distance_squared += delta * delta;
  }
  record["seed_distance_rad"] = std::sqrt(seed_distance_squared);
  record["grasp_validation"] = attempt.stage_elapsed_s.count("ValidateApproachState") ? "VALID" :
    attempt.stage_elapsed_s.count("ValidateGraspKinematics") ? "REJECTED" : "NOT_RUN";
  record["planning"] = attempt.planning_status;
  const auto & occupancy = attempt.grasp_occupancy_result;
  auto & validation = record["grasp_collision_validation"];
  validation = {{"mode", grasp_occupancy_mode_name(occupancy.mode)},
    {"evaluated", occupancy.evaluated}, {"valid", occupancy.valid},
    {"common_collision", occupancy.evaluated ? nlohmann::json(occupancy.common.collision) :
      nlohmann::json(nullptr)},
    {"occupancy_evaluated", occupancy.occupancy_evaluated},
    {"occupancy_collision", occupancy.occupancy_evaluated ?
      nlohmann::json(occupancy.occupancy.collision) : nlohmann::json(nullptr)},
    {"allowed", occupancy.evaluated ? nlohmann::json(occupancy.allowed) : nlohmann::json(nullptr)},
    {"reason", occupancy.reason}, {"frame_id", attempt.current_grasp_goal.header.frame_id},
    {"finger_volumes", nlohmann::json::array()}, {"intrusions", nlohmann::json::array()},
    {"common_contacts", occupancy.common.contacts},
    {"occupancy_contacts", occupancy.occupancy.contacts}};
  if (occupancy.finger_volumes) {
    for (std::size_t side = 0; side < occupancy.finger_volumes->size(); ++side) {
      const auto & volume = (*occupancy.finger_volumes)[side];
      const auto & p = volume.frame.translation();
      const Eigen::Quaterniond q(volume.frame.linear());
      validation["finger_volumes"].push_back({{"finger_index", side},
        {"shape", "OUTWARD_HALF_CAPSULE_EXTRUSION"}, {"position_m", {p.x(), p.y(), p.z()}},
        {"orientation_xyzw", {q.x(), q.y(), q.z(), q.w()}},
        {"radius_m", volume.dimensions.radius_m},
        {"contact_span_m", volume.dimensions.contact_span_m},
        {"insertion_interval_m", {-volume.dimensions.depth_m, 0.0}}});
    }
  }
  for (const auto & intrusion : occupancy.intrusions) {
    validation["intrusions"].push_back({{"finger_index", intrusion.finger_index},
      {"occupancy_id", intrusion.occupancy_id}});
  }
  record["grasp_ik"] = {{"called", grasp_called},
    {"timeout_limit_s", attempt.grasp_timeout_s},
    {"elapsed_s", grasp_elapsed}, {"solver_returned", attempt.grasp_ik_result.solver_returned},
    {"moveit_error_code", attempt.grasp_ik_error.val},
    {"reason", reason.empty() ? "NONE" : reason}};
  record["approach_ik"] = {{"calls", approach_called ? 1 : 0},
    {"elapsed_s", attempt.approach_ik_elapsed_s},
    {"timeout_limit_s", attempt.approach_timeout_s},
    {"result",
      !approach_called ? "NOT_RUN" : attempt.approach_ik_result.check ==
      PickApproachIkCheck::VALID ?
      "SUCCESS" : "IK_FAILED"},
    {"reason", !approach_called ? "NOT_RUN" : attempt.approach_ik_result.reason},
    {"timeout_status", "not_exposed_by_robot_state_api"}};
  record["collision_seed_diversification_used"] = attempt.collision_seed_diversification_used;
  record["collision_feedback_source_contacts"] = nlohmann::json::array();
  for (const auto & contact : attempt.collision_feedback) {
    record["collision_feedback_source_contacts"].push_back(
      {{"body_1", contact.body_1},
        {"body_2", contact.body_2}, {"stage", contact.stage}, {"pose_candidate",
          contact.pose_candidate},
        {"ik_seed_attempt", contact.seed_attempt}, {"sample_index", contact.sample_index_available ?
          nlohmann::json(contact.sample_index) : nlohmann::json(nullptr)}});
  }
  record["selection_path"] = (attempt.selected_joint_goal.empty() ? "none" : "direct_grasp_6d");
  record["grasp_ik_solution_joints"] = attempt.grasp_joint_positions;
  const auto pose_record = [](const geometry_msgs::msg::PoseStamped & goal) {
      return nlohmann::json{{"position",
      {goal.pose.position.x, goal.pose.position.y, goal.pose.position.z}},
      {"orientation",
        {goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z,
          goal.pose.orientation.w}}};
    };
  record["grasp_ik_goal"] = pose_record(attempt.current_grasp_goal);
  record["approach_ik_goal"] = pose_record(attempt.current_approach_goal);
  return record;
}
}  // namespace
void PickPlanningDiagnostics::resize(std::size_t count, bool has_recipe_candidates)
{
  summary_.candidate_joint_distance.assign(count, std::numeric_limits<double>::infinity());
  summary_.tcp_orientation_candidates.assign(count, Eigen::Quaterniond::Identity());
  summary_.expected_width_candidates.assign(count, 0.0);
  summary_.candidate_evaluated.assign(count, false);
  summary_.candidate_results.assign(count, "NOT_EVALUATED");
  if (!has_recipe_candidates && count > 0U) {
    summary_.candidate_results[0] = "NO_RECIPE_VALID_GRASP_POSE";
  }
  summary_.candidate_transition_reasons.assign(count, "NOT_TRANSITIONED");
  summary_.candidate_plan_status.assign(count, "NOT_ATTEMPTED");
  summary_.candidate_plan_elapsed_s.assign(count, 0.0);
  summary_.candidate_moveit_error_code.assign(count, 0);
  summary_.candidate_trajectory_points.assign(count, 0U);
  summary_.candidate_finalize_reason.assign(count, "not_attempted");
}
void PickPlanningDiagnostics::finish(const PickPlanningResult & result)
{
  summary_.pick_search_budget_exhausted = result.search_status ==
    PickSearchStatus::SEARCH_BUDGET_EXHAUSTED;
  summary_.pick_search_termination_reason = result.search_status ==
    PickSearchStatus::INTERRUPTED ? "INTERRUPTED" :
    summary_.pick_search_budget_exhausted ? "SEARCH_BUDGET_EXHAUSTED" :
    result.selected ? "EXECUTABLE_APPROACH_PLAN_SELECTED" :
    result.search_status ==
    PickSearchStatus::BACKEND_ERROR ? "BACKEND_ERROR" : "ALL_CANDIDATES_EXHAUSTED";
  summary_.ik_termination_reason = summary_.pick_search_termination_reason;
  if (!result.selected) {
    for (std::size_t pose = 0; pose < summary_.candidate_results.size(); ++pose) {
      if (summary_.candidate_results[pose] == "SEED_DEFERRED" ||
        (!summary_.candidate_evaluated[pose] &&
        summary_.candidate_results[pose] == "NOT_EVALUATED"))
      {
        summary_.candidate_transition_reasons[pose] = summary_.pick_search_budget_exhausted ?
          "SEARCH_BUDGET_EXHAUSTED" : "SEARCH_TERMINATED_BEFORE_EVALUATION";
      }
    }
  }
}

void PickPlanningDiagnostics::collect(
  const PickPlanningAttempt & attempt,
  const PickPlanningResult & result, const PickPlanningDiagnosticMetadata & metadata)
{
  const auto pose = attempt.current_pose_index;
  const auto seed = attempt.seed_index;
  const auto evaluation = attempt.attempt_evaluation;
  const auto & stage = attempt.failure_stage;
  const auto & reason = attempt.failure_reason;
  summary_.candidate_target = attempt.current_grasp_goal;
  summary_.candidate_approach = attempt.current_approach_goal;
  if (!summary_.candidate_evaluated[pose]) {
    summary_.candidate_evaluated[pose] = true;
    ++summary_.evaluated_candidate_count;
  }
  summary_.final_pick_failure_stage = stage;
  summary_.final_pick_failure_reason = reason;
  summary_.diagnostic_grasp_ik_solution = attempt.grasp_joint_positions;
  summary_.diagnostic_selection_path =
    attempt.selected_joint_goal.empty() ? "none" : "direct_grasp_6d";
  const bool grasp_called = attempt.stage_elapsed_s.count("SolveGraspIK") != 0U;
  const bool approach_called = attempt.stage_elapsed_s.count("SolveApproachIK") != 0U;
  const double grasp_elapsed = stage_elapsed(attempt, "SolveGraspIK");
  const double validation_elapsed = stage_elapsed(attempt, "ValidateGraspKinematics") +
    stage_elapsed(
    attempt,
    "ValidateGraspContact") + stage_elapsed(attempt, "ValidateGraspCollision") +
    stage_elapsed(attempt, "ValidateApproachState");
  summary_.ik_solver_calls += grasp_called ? 1U : 0U;
  summary_.ik_attempts += grasp_called ? 1U : 0U;
  summary_.ik_api_elapsed_s += grasp_elapsed;
  summary_.ik_validation_elapsed_s += validation_elapsed;
  summary_.task_validation_attempts += attempt.stage_elapsed_s.count("ValidateGraspKinematics");
  summary_.approach_ik_calls += approach_called ? 1U : 0U;
  summary_.approach_ik_elapsed_s += attempt.approach_ik_elapsed_s;
  summary_.pick_global_planning_calls += attempt.planning_request_evidence.size();
  summary_.pick_global_planning_elapsed_s += attempt.approach_planning_elapsed_s;
  summary_.planning_attempted += attempt.stage_elapsed_s.count("PlanApproachPath");
  if (evaluation == PickSearchEvaluation::GLOBAL_PLANNING_FAILED) {
    if (attempt.stage_elapsed_s.count("FinalizeTrajectory")) {
      ++summary_.trajectory_finalize_failed;
    } else {++summary_.planning_failed;}
  }
  if (result.selected && evaluation == PickSearchEvaluation::EXECUTABLE_APPROACH_SELECTED) {
    ++summary_.planned_candidate_count;
  }
  if (attempt.stage_elapsed_s.count("RegisterValidCandidate")) {++summary_.valid_ik_solutions;}
  if (evaluation == PickSearchEvaluation::GRASP_CONTACT_MISMATCH) {++summary_.geometry_rejected;}
  if (attempt.grasp_validation_result && attempt.grasp_validation_result->check ==
    PickGraspKinematicCheck::JOINT_BOUNDS) {++summary_.bounds_rejected;}
  if (attempt.grasp_validation_result && attempt.grasp_validation_result->check ==
    PickGraspKinematicCheck::FK_TARGET_MISMATCH) {++summary_.fk_rejected;}
  if (attempt.grasp_collision_result.collision) {++summary_.collision_rejected;}
  for (const auto & contact : metadata.collision_contacts) {
    summary_.all_pick_collision_contacts.push_back(contact);
  }
  if (attempt.candidate_registered) {
    summary_.candidate_joint_distance[pose] = attempt.joint_distance;
    const auto & q = attempt.current_grasp_goal.pose.orientation;
    summary_.tcp_orientation_candidates[pose] = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
    summary_.expected_width_candidates[pose] = attempt.grasp_contact_result.contact ?
      attempt.grasp_contact_result.contact->width_mm : 0.0;
  }
  summary_.candidate_plan_elapsed_s[pose] = attempt.approach_planning_elapsed_s;
  summary_.candidate_moveit_error_code[pose] = attempt.planning_error_code;
  summary_.candidate_trajectory_points[pose] = attempt.trajectory_points;
  summary_.candidate_finalize_reason[pose] = attempt.finalize_reason;
  summary_.candidate_plan_status[pose] = attempt.planning_status == "SUCCESS" ?
    (evaluation == PickSearchEvaluation::EXECUTABLE_APPROACH_SELECTED ? "PLANNED_FEASIBLE" :
    "TRAJECTORY_FINALIZATION_FAILED") : attempt.planning_status == "FAILED" ?
    "PLANNING_FAILED" : "NOT_ATTEMPTED";
  summary_.pick_attempt_records.push_back(attempt_record(attempt, metadata).dump());
  summary_.candidate_transition_reasons[pose] = reason;
  summary_.candidate_results[pose] = evaluation ==
    PickSearchEvaluation::EXECUTABLE_APPROACH_SELECTED ? "PLANNED_SELECTED" :
    evaluation == PickSearchEvaluation::BACKEND_ERROR ? "BACKEND_ERROR" :
    evaluation == PickSearchEvaluation::GLOBAL_PLANNING_FAILED ? "APPROACH_PLANNING_EXHAUSTED" :
    evaluation == PickSearchEvaluation::GRASP_IK_NO_SOLUTION &&
    seed < metadata.seed_limit ? "SEED_DEFERRED" :
    evaluation == PickSearchEvaluation::GRASP_CONTACT_MISMATCH ? "GRASP_CONTACT_MISMATCH" :
    seed >= metadata.seed_limit ? "IK_SEED_ATTEMPTS_EXHAUSTED" : "REJECTED";
  if (evaluation == PickSearchEvaluation::BACKEND_ERROR) {
    summary_.pick_search_termination_reason = "BACKEND_ERROR";
    summary_.ik_termination_reason = "BACKEND_ERROR";
  }
  if (stage == "INTERRUPTED") {
    summary_.pick_search_termination_reason = "INTERRUPTED";
    summary_.ik_termination_reason = "INTERRUPTED";
    summary_.candidate_results[pose] = "DISCARDED_INTERRUPTED";
  } else if (stage == "SEARCH_BUDGET") {
    summary_.pick_search_budget_exhausted = true;
    summary_.pick_search_termination_reason = "SEARCH_BUDGET_EXHAUSTED";
    summary_.ik_termination_reason = "SEARCH_BUDGET_EXHAUSTED";
    summary_.candidate_results[pose] = "SEARCH_BUDGET_EXHAUSTED";
  }

}
}  // namespace arm_cell_motion
