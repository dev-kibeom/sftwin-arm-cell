#pragma once

#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include "arm_cell_motion_moveit2/place_orientation_constraint.hpp"
#include "arm_cell_motion_moveit2/trajectory_continuity.hpp"

namespace arm_cell_motion_moveit2
{

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
  bool ik_success{false};
  std::vector<double> raw_solution;
  bool goal_valid{false};
  std::vector<double> normalized_goal;
  std::vector<double> raw_delta;
  std::vector<double> wrap_aware_delta;
  bool endpoint_state_valid{false};
  std::vector<std::string> contact_bodies;
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

struct PlaceApproachPlanObservation
{
  bool success{false};
  int result_code{0};
  std::size_t trajectory_points{0};
  double planning_duration_s{0.0};
  double trajectory_duration_s{0.0};
  bool time_parameterized{false};
  std::string time_parameterization_reason;
};

struct PlaceApproachPlanAttempt
{
  std::size_t candidate_index{0};
  std::size_t seed_index{0};
  double planning_budget_s{0.0};
  double planning_duration_s{0.0};
  double trajectory_duration_s{0.0};
  int result_code{0};
  std::size_t trajectory_points{0};
  bool success{false};
  bool time_parameterized{false};
  std::string time_parameterization_reason;
};

struct PlaceApproachPlanResult
{
  bool success{false};
  std::size_t selected_candidate_index{0};
  double elapsed_planning_s{0.0};
  std::vector<PlaceApproachPlanAttempt> attempts;
};

enum class PlacePlanSelectionPolicy
{
  EXISTING_RANKING,
  SHORTEST_PLANNING_DURATION
};

using PlaceApproachPlanFunction = std::function<PlaceApproachPlanObservation(
      const PlaceApproachIkCandidate &, double)>;

PlaceApproachPlanResult plan_ranked_place_approach_candidates(
  const std::vector<PlaceApproachIkCandidate> & candidates,
  const std::vector<std::size_t> & ranked_candidate_indices,
  const PlaceApproachIkPolicy & policy,
  const PlaceApproachPlanFunction & plan_candidate,
  PlacePlanSelectionPolicy selection_policy =
  PlacePlanSelectionPolicy::EXISTING_RANKING,
  std::size_t max_plan_candidates = 0U);

PlaceApproachPlanResult plan_place_candidates_with_fallback(
  const std::vector<PlaceApproachIkCandidate> & candidates,
  const std::vector<std::size_t> & primary_candidate_indices,
  const std::vector<std::size_t> & fallback_candidate_indices,
  const PlaceApproachIkPolicy & policy,
  const PlaceApproachPlanFunction & plan_candidate,
  PlacePlanSelectionPolicy selection_policy,
  std::size_t max_primary_candidates,
  std::size_t max_fallback_candidates);

std::vector<std::size_t> interleave_place_candidates_by_pose(
  const std::vector<PlaceApproachIkCandidate> & candidates,
  const std::vector<std::size_t> & ranked_candidate_indices,
  std::size_t pose_candidate_count);

}  // namespace arm_cell_motion_moveit2
