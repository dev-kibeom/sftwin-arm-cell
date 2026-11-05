#pragma once

#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include "arm_cell_motion/place_orientation_constraint.hpp"
#include "arm_cell_motion/trajectory_continuity.hpp"

namespace arm_cell_motion
{

bool place_planning_failure_is_retryable(
  int result_code, double planning_duration_s, double planning_budget_s);
std::string moveit_error_code_name(int result_code);

struct PlaceObjectPoseCandidate
{
  geometry_msgs::msg::PoseStamped object_pose;
  double nominal_deviation{0.0};
};

struct BoundedPlacePoseCandidateGroups
{
  std::vector<std::size_t> primary_orientation_indices;
  std::vector<std::size_t> position_fallback_indices;
};

std::vector<PlaceObjectPoseCandidate> generate_place_object_pose_candidates(
  const geometry_msgs::msg::PoseStamped & desired_object_pose,
  double position_tolerance_m, double orientation_tolerance_rad,
  PlaceOrientationConstraint orientation_constraint = PlaceOrientationConstraint::BOUNDED);

BoundedPlacePoseCandidateGroups group_bounded_place_pose_candidates(
  const std::vector<PlaceObjectPoseCandidate> & candidates,
  const geometry_msgs::msg::PoseStamped & nominal_object_pose);

struct PlaceApproachIkPolicy
{
  double ik_seed_quantum_rad{0.001};
  double seed_variation_rad{0.25};
  std::size_t max_candidates{5};
  double ik_timeout_s{0.2};
  double per_candidate_planning_budget_s{1.0};
  double total_planning_budget_s{5.0};
  double duplicate_tolerance_rad{1e-6};
  std::size_t planning_requests_per_candidate{1U};
};

struct PlaceApproachIkSeed
{
  std::size_t index{0};
  std::string source;
  std::vector<double> joints;
};

std::vector<double> canonicalize_ik_seed(
  const std::vector<double> & measured_joints, double quantum_rad);

std::vector<PlaceApproachIkSeed> generate_place_approach_ik_seeds(
  const std::vector<double> & measured_joints,
  const std::vector<std::string> & joint_names,
  const std::vector<JointContinuityLimit> & joint_limits,
  const PlaceApproachIkPolicy & policy = {});

struct PlaceApproachIkCandidate
{
  std::size_t object_pose_candidate_index{0};
  double nominal_pose_deviation{0.0};
  std::optional<double> tool_orientation_preference_error_rad;
  std::size_t seed_index{0};
  std::string seed_source;
  std::vector<double> seed_positions;
  bool ik_attempted{false};
  bool ik_success{false};
  double ik_elapsed_s{0.0};
  std::vector<double> raw_solution;
  bool goal_valid{false};
  std::vector<double> normalized_goal;
  std::vector<double> raw_delta;
  std::vector<double> wrap_aware_delta;
  bool endpoint_state_valid{false};
  std::vector<std::string> contact_bodies;
  std::vector<std::pair<std::string, std::string>> contact_pairs;
  bool duplicate_rejected{false};
  bool valid{false};
  double ranking_cost{0.0};
  std::string rejection_reason;
};

std::optional<double> orientation_angular_distance_rad(
  const geometry_msgs::msg::Quaternion & candidate,
  const geometry_msgs::msg::Quaternion & preferred);

bool prepare_place_approach_candidate(
  PlaceApproachIkCandidate & candidate,
  const std::vector<double> & measured_joints,
  const std::vector<JointContinuityLimit> & joint_limits);

std::vector<std::size_t> rank_place_approach_candidates(
  std::vector<PlaceApproachIkCandidate> & candidates,
  const std::vector<double> & measured_joints,
  const std::vector<JointContinuityLimit> & joint_limits,
  const std::vector<double> & cost_weights = {},
  double duplicate_tolerance_rad = 1e-6);

std::vector<std::size_t> order_place_approach_candidate_specs(
  const std::vector<PlaceApproachIkCandidate> & candidates);

struct PlaceApproachPlanObservation
{
  PlaceApproachPlanObservation() = default;
  PlaceApproachPlanObservation(
    bool success_in, int result_code_in, std::size_t trajectory_points_in,
    double planning_duration_s_in, double trajectory_duration_s_in,
    bool time_parameterized_in, std::string time_parameterization_reason_in)
  : success(success_in),
    result_code(result_code_in),
    trajectory_points(trajectory_points_in),
    planning_duration_s(planning_duration_s_in),
    trajectory_duration_s(trajectory_duration_s_in),
    time_parameterized(time_parameterized_in),
    time_parameterization_reason(std::move(time_parameterization_reason_in))
  {}

  bool success{false};
  int result_code{0};
  std::size_t trajectory_points{0};
  double planning_duration_s{0.0};
  double trajectory_duration_s{0.0};
  bool time_parameterized{false};
  std::string time_parameterization_reason;
  bool target_setter_attempted{false};
  bool target_setter_accepted{false};
  bool planner_invoked{false};
  bool ik_attempted{false};
  bool ik_succeeded{false};
  double ik_elapsed_s{0.0};
  std::string validation_stage;
  bool terminal{false};
  bool waypoint_retryable{false};
  bool infrastructure_failure{false};
  struct TargetVariableDiagnostic
  {
    std::string name;
    double value{0.0};
    bool position_bounded{false};
    double lower_bound{0.0};
    double upper_bound{0.0};
    bool within_bounds{false};
  };
  struct TargetStateDiagnostic
  {
    std::string group_name;
    std::vector<std::string> group_variable_names;
    bool group_contract_matches{false};
    bool goal_size_matches_group{false};
    bool group_bounds_satisfied{false};
    std::vector<TargetVariableDiagnostic> variables;
  } target_state_diagnostic;
};

struct PlaceApproachPlanAttempt
{
  std::size_t candidate_index{0};
  std::size_t request_index{0};
  std::size_t seed_index{0};
  double planning_budget_s{0.0};
  double planning_duration_s{0.0};
  double trajectory_duration_s{0.0};
  int result_code{0};
  std::size_t trajectory_points{0};
  bool success{false};
  bool time_parameterized{false};
  std::string time_parameterization_reason;
  bool target_setter_attempted{false};
  bool target_setter_accepted{false};
  bool planner_invoked{false};
  bool ik_attempted{false};
  bool ik_succeeded{false};
  double ik_elapsed_s{0.0};
  std::string validation_stage;
  bool terminal{false};
  bool waypoint_retryable{false};
  bool infrastructure_failure{false};
  PlaceApproachPlanObservation::TargetStateDiagnostic target_state_diagnostic;
};

struct PlaceApproachPlanResult
{
  bool success{false};
  bool terminal{false};
  std::size_t selected_candidate_index{0};
  double elapsed_planning_s{0.0};
  std::vector<PlaceApproachPlanAttempt> attempts;
};

using PlaceApproachPlanFunction = std::function<PlaceApproachPlanObservation(
      const PlaceApproachIkCandidate &, double)>;

PlaceApproachPlanResult plan_ranked_place_approach_candidates(
  const std::vector<PlaceApproachIkCandidate> & candidates,
  const std::vector<std::size_t> & ranked_candidate_indices,
  const PlaceApproachIkPolicy & policy,
  const PlaceApproachPlanFunction & plan_candidate,
  std::size_t max_plan_candidates = 0U);

PlaceApproachPlanResult plan_place_candidates_with_fallback(
  const std::vector<PlaceApproachIkCandidate> & candidates,
  const std::vector<std::size_t> & primary_candidate_indices,
  const std::vector<std::size_t> & fallback_candidate_indices,
  const PlaceApproachIkPolicy & policy,
  const PlaceApproachPlanFunction & plan_candidate,
  std::size_t max_primary_candidates,
  std::size_t max_fallback_candidates);

std::vector<std::size_t> interleave_place_candidates_by_pose(
  const std::vector<PlaceApproachIkCandidate> & candidates,
  const std::vector<std::size_t> & ranked_candidate_indices,
  std::size_t pose_candidate_count);

}  // namespace arm_cell_motion
