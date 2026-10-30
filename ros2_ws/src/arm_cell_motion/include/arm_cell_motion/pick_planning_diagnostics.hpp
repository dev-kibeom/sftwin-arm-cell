#pragma once
#include "arm_cell_motion/pick_approach_planning_bt.hpp"
namespace arm_cell_motion
{
struct PickPlanningDiagnosticMetadata
{
  std::size_t attempt_number{0U};
  std::size_t seed_limit{0U};
  std::vector<std::string> bt_steps;
  std::vector<PickCollisionContact> collision_contacts;
};
// Derived reporting values only. No controller, budget, solver or plan ownership.
struct PickPlanningDiagnosticSummary
{
  geometry_msgs::msg::PoseStamped candidate_target;
  geometry_msgs::msg::PoseStamped candidate_approach;
  std::vector<double> candidate_joint_distance;
  std::vector<Eigen::Quaterniond> tcp_orientation_candidates;
  std::vector<double> expected_width_candidates;
  std::vector<bool> candidate_evaluated;
  std::size_t evaluated_candidate_count{};
  std::string final_pick_failure_stage;
  std::string final_pick_failure_reason;
  std::vector<double> diagnostic_grasp_ik_solution;
  std::string diagnostic_selection_path;
  std::size_t ik_solver_calls{};
  std::size_t ik_attempts{};
  double ik_api_elapsed_s{};
  double ik_validation_elapsed_s{};
  double ik_search_elapsed_s{};
  double ik_callback_elapsed_s{};
  std::size_t task_validation_attempts{};
  std::size_t approach_ik_calls{};
  double approach_ik_elapsed_s{};
  std::size_t pick_global_planning_calls{};
  double pick_global_planning_elapsed_s{};
  std::size_t planning_attempted{};
  std::size_t trajectory_finalize_failed{};
  std::size_t planning_failed{};
  std::size_t planned_candidate_count{};
  std::size_t valid_ik_solutions{};
  std::size_t geometry_rejected{};
  std::size_t bounds_rejected{};
  std::size_t fk_rejected{};
  std::size_t collision_rejected{};
  std::vector<PickCollisionContact> all_pick_collision_contacts;
  std::vector<double> candidate_plan_elapsed_s;
  std::vector<int> candidate_moveit_error_code;
  std::vector<std::size_t> candidate_trajectory_points;
  std::vector<std::string> candidate_finalize_reason;
  std::vector<std::string> candidate_plan_status;
  std::vector<std::string> pick_attempt_records;
  std::vector<std::string> candidate_transition_reasons;
  std::vector<std::string> candidate_results;
  std::string pick_search_termination_reason;
  std::string ik_termination_reason;
  bool pick_search_budget_exhausted{};
};
class PickPlanningDiagnostics
{
public:
  void resize(std::size_t candidates, bool has_recipe_candidates = true);
  void finish(const PickPlanningResult & result);
  void collect(
    const PickPlanningAttempt & attempt, const PickPlanningResult & result,
    const PickPlanningDiagnosticMetadata & metadata);
  const PickPlanningDiagnosticSummary & summary() const {return summary_;}
  // Existing shared serializer uses these reporting values for the legacy PLACE path too.
  PickPlanningDiagnosticSummary & legacy_output() {return summary_;}

private:
  PickPlanningDiagnosticSummary summary_;
};
}  // namespace arm_cell_motion
