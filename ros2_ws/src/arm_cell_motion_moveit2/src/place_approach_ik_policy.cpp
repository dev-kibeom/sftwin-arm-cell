#include "arm_cell_motion_moveit2/place_approach_ik_policy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <utility>

namespace arm_cell_motion_moveit2
{

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;
constexpr double kBoundTolerance = 1e-9;

bool finite_vector(const std::vector<double> & values)
{
  return std::all_of(
    values.begin(), values.end(), [](double value) {
      return std::isfinite(value);
    });
}

bool inside_bounds(double value, const JointContinuityLimit & limit)
{
  return std::isfinite(value) && value >= limit.lower - kBoundTolerance &&
         value <= limit.upper + kBoundTolerance;
}

bool wrap_equivalent(
  const std::vector<double> & first,
  const std::vector<double> & second,
  const std::vector<JointContinuityLimit> & limits,
  double tolerance)
{
  if (first.size() != second.size() || first.size() != limits.size()) {
    return false;
  }
  for (std::size_t index = 0; index < first.size(); ++index) {
    const double delta = first[index] - second[index];
    const double equivalent_delta = limits[index].wrap_equivalent ?
      std::remainder(delta, kTwoPi) : delta;
    if (std::abs(equivalent_delta) > tolerance) {
      return false;
    }
  }
  return true;
}
}  // namespace

std::optional<double> orientation_angular_distance_rad(
  const geometry_msgs::msg::Quaternion & candidate,
  const geometry_msgs::msg::Quaternion & preferred)
{
  const double candidate_norm = std::sqrt(
    candidate.x * candidate.x + candidate.y * candidate.y +
    candidate.z * candidate.z + candidate.w * candidate.w);
  const double preferred_norm = std::sqrt(
    preferred.x * preferred.x + preferred.y * preferred.y +
    preferred.z * preferred.z + preferred.w * preferred.w);
  if (!std::isfinite(candidate_norm) || candidate_norm <= 1e-12 ||
    !std::isfinite(preferred_norm) || preferred_norm <= 1e-12)
  {
    return std::nullopt;
  }
  const double dot = std::abs(
    (candidate.x * preferred.x + candidate.y * preferred.y +
    candidate.z * preferred.z + candidate.w * preferred.w) /
    (candidate_norm * preferred_norm));
  if (!std::isfinite(dot)) {
    return std::nullopt;
  }
  return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
}

std::vector<double> canonicalize_ik_seed(
  const std::vector<double> & measured_joints, double quantum_rad)
{
  if (measured_joints.empty() || !finite_vector(measured_joints) ||
    !std::isfinite(quantum_rad) || quantum_rad <= 0.0)
  {
    return {};
  }
  std::vector<double> canonical(measured_joints.size());
  std::transform(
    measured_joints.begin(), measured_joints.end(), canonical.begin(),
    [quantum_rad](double value) {return std::round(value / quantum_rad) * quantum_rad;});
  return canonical;
}

std::vector<PlaceApproachIkSeed> generate_place_approach_ik_seeds(
  const std::vector<double> & measured_joints,
  const std::vector<std::string> & joint_names,
  const std::vector<JointContinuityLimit> & joint_limits,
  const PlaceApproachIkPolicy & policy)
{
  std::vector<PlaceApproachIkSeed> seeds;
  if (measured_joints.empty() || measured_joints.size() != joint_names.size() ||
    measured_joints.size() != joint_limits.size() || policy.max_candidates == 0U ||
    !std::isfinite(policy.seed_variation_rad) || policy.seed_variation_rad <= 0.0)
  {
    return seeds;
  }
  const auto canonical = canonicalize_ik_seed(measured_joints, policy.ik_seed_quantum_rad);
  if (canonical.size() != measured_joints.size()) {
    return seeds;
  }
  for (std::size_t index = 0; index < canonical.size(); ++index) {
    if (!inside_bounds(canonical[index], joint_limits[index])) {
      return seeds;
    }
  }

  seeds.push_back({0U, "canonical_current", canonical});
  const auto append_variations = [&](const std::string & joint_name,
      const std::string & source_prefix, std::size_t first_index) {
      const auto found = std::find(joint_names.begin(), joint_names.end(), joint_name);
      if (found == joint_names.end()) {
        return;
      }
      const auto joint_index = static_cast<std::size_t>(found - joint_names.begin());
      std::size_t variation_index = 0U;
      for (const auto & variation : std::vector<std::pair<double, std::string>>{
        {policy.seed_variation_rad, "_plus"},
        {-policy.seed_variation_rad, "_minus"}})
      {
        const auto seed_index = first_index + variation_index++;
        if (seed_index >= policy.max_candidates) {
          return;
        }
        auto seed_joints = canonical;
        seed_joints[joint_index] += variation.first;
        if (!inside_bounds(seed_joints[joint_index], joint_limits[joint_index])) {
          continue;
        }
        seeds.push_back(
          {seed_index, source_prefix + variation.second, std::move(seed_joints)});
      }
    };
  append_variations("joint_1", "joint_1", 1U);
  append_variations("joint_3", "joint_3", 3U);
  return seeds;
}

bool prepare_place_approach_candidate(
  PlaceApproachIkCandidate & candidate,
  const std::vector<double> & measured_joints,
  const std::vector<JointContinuityLimit> & joint_limits)
{
  candidate.goal_valid = false;
  candidate.normalized_goal.clear();
  candidate.raw_delta.clear();
  candidate.wrap_aware_delta.clear();
  candidate.valid = false;
  if (!candidate.ik_success) {
    if (candidate.rejection_reason.empty()) {
      candidate.rejection_reason = "IK_FAILED";
    }
    return false;
  }
  const auto selection = select_nearest_joint_goal(
    candidate.raw_solution, measured_joints, joint_limits);
  candidate.raw_delta = selection.raw_endpoint_delta;
  candidate.wrap_aware_delta = selection.wrap_aware_endpoint_delta;
  candidate.normalized_goal = selection.selected_goal;
  candidate.goal_valid = selection.valid;
  if (!selection.valid) {
    candidate.rejection_reason = selection.reason;
    return false;
  }
  candidate.rejection_reason.clear();
  return true;
}

std::vector<std::size_t> rank_place_approach_candidates(
  std::vector<PlaceApproachIkCandidate> & candidates,
  const std::vector<double> & measured_joints,
  const std::vector<JointContinuityLimit> & joint_limits,
  const std::vector<double> & cost_weights,
  double duplicate_tolerance_rad)
{
  std::vector<std::size_t> ranked;
  std::vector<std::size_t> unique_candidates;
  if (!std::isfinite(duplicate_tolerance_rad) || duplicate_tolerance_rad < 0.0) {
    return ranked;
  }
  for (std::size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index) {
    auto & candidate = candidates[candidate_index];
    candidate.valid = false;
    candidate.duplicate_rejected = false;
    candidate.ranking_cost = 0.0;
    if (!candidate.ik_success || !candidate.goal_valid) {
      if (candidate.rejection_reason.empty()) {
        candidate.rejection_reason = candidate.ik_success ? "IK_GOAL_INVALID" : "IK_FAILED";
      }
      continue;
    }
    if (!candidate.endpoint_state_valid) {
      if (candidate.rejection_reason.empty()) {
        candidate.rejection_reason = "ENDPOINT_STATE_INVALID";
      }
      continue;
    }
    if (candidate.normalized_goal.size() != measured_joints.size() ||
      joint_limits.size() != measured_joints.size() ||
      (!cost_weights.empty() && cost_weights.size() != measured_joints.size()))
    {
      candidate.rejection_reason = "RANKING_INPUT_DIMENSION_MISMATCH";
      continue;
    }

    candidate.raw_delta.resize(measured_joints.size());
    candidate.wrap_aware_delta.resize(measured_joints.size());
    bool cost_valid = true;
    for (std::size_t joint = 0; joint < measured_joints.size(); ++joint) {
      const double weight = cost_weights.empty() ? 1.0 : cost_weights[joint];
      if (!std::isfinite(weight) || weight < 0.0) {
        cost_valid = false;
        break;
      }
      candidate.raw_delta[joint] = candidate.normalized_goal[joint] - measured_joints[joint];
      candidate.wrap_aware_delta[joint] = joint_limits[joint].wrap_equivalent ?
        std::remainder(candidate.raw_delta[joint], kTwoPi) : candidate.raw_delta[joint];
      candidate.ranking_cost += weight * std::abs(candidate.wrap_aware_delta[joint]);
    }
    if (!cost_valid || !std::isfinite(candidate.ranking_cost)) {
      candidate.ranking_cost = std::numeric_limits<double>::infinity();
      candidate.rejection_reason = "RANKING_COST_INVALID";
      continue;
    }

    const bool duplicate = std::any_of(
      unique_candidates.begin(), unique_candidates.end(), [&](std::size_t earlier_index) {
      return wrap_equivalent(
          candidate.normalized_goal, candidates[earlier_index].normalized_goal,
          joint_limits, duplicate_tolerance_rad) &&
             candidate.object_pose_candidate_index ==
             candidates[earlier_index].object_pose_candidate_index;
      });
    if (duplicate) {
      candidate.duplicate_rejected = true;
      candidate.rejection_reason = "WRAP_EQUIVALENT_DUPLICATE";
      continue;
    }
    candidate.valid = true;
    candidate.rejection_reason.clear();
    unique_candidates.push_back(candidate_index);
  }

  ranked = unique_candidates;
  std::stable_sort(
    ranked.begin(), ranked.end(), [&](std::size_t left, std::size_t right) {
      const auto left_orientation_error =
        candidates[left].tool_orientation_preference_error_rad;
      const auto right_orientation_error =
        candidates[right].tool_orientation_preference_error_rad;
      if (left_orientation_error && right_orientation_error &&
        *left_orientation_error != *right_orientation_error)
      {
        return *left_orientation_error < *right_orientation_error;
      }
      const auto left_cost = candidates[left].ranking_cost;
      const auto right_cost = candidates[right].ranking_cost;
      const auto left_deviation = candidates[left].nominal_pose_deviation;
      const auto right_deviation = candidates[right].nominal_pose_deviation;
      if (left_deviation != right_deviation) {
        return left_deviation < right_deviation;
      }
      if (left_cost != right_cost) {
        return left_cost < right_cost;
      }
      if (candidates[left].seed_index != candidates[right].seed_index) {
        return candidates[left].seed_index < candidates[right].seed_index;
      }
      return left < right;
    });
  return ranked;
}

PlaceApproachPlanResult plan_ranked_place_approach_candidates(
  const std::vector<PlaceApproachIkCandidate> & candidates,
  const std::vector<std::size_t> & ranked_candidate_indices,
  const PlaceApproachIkPolicy & policy,
  const PlaceApproachPlanFunction & plan_candidate,
  PlacePlanSelectionPolicy selection_policy,
  std::size_t max_plan_candidates)
{
  PlaceApproachPlanResult result;
  double selected_pose_deviation = std::numeric_limits<double>::infinity();
  double selected_joint_cost = std::numeric_limits<double>::infinity();
  double selected_trajectory_duration = std::numeric_limits<double>::infinity();
  double selected_planning_duration = std::numeric_limits<double>::infinity();
  double selected_orientation_error = std::numeric_limits<double>::infinity();
  const auto candidate_limit = max_plan_candidates == 0U ?
    policy.max_candidates : max_plan_candidates;
  if (!plan_candidate || policy.max_candidates == 0U ||
    candidate_limit == 0U ||
    !std::isfinite(policy.per_candidate_planning_budget_s) ||
    policy.per_candidate_planning_budget_s <= 0.0 ||
    !std::isfinite(policy.total_planning_budget_s) || policy.total_planning_budget_s <= 0.0)
  {
    return result;
  }

  for (const auto candidate_index : ranked_candidate_indices) {
    if (result.attempts.size() >= candidate_limit ||
      candidate_index >= candidates.size() || !candidates[candidate_index].valid)
    {
      continue;
    }
    const double remaining_s = policy.total_planning_budget_s - result.elapsed_planning_s;
    if (remaining_s <= 0.0) {
      break;
    }
    const double budget_s = std::min(policy.per_candidate_planning_budget_s, remaining_s);
    const auto observation = plan_candidate(candidates[candidate_index], budget_s);
    const double duration_s = std::isfinite(observation.planning_duration_s) ?
      std::max(0.0, observation.planning_duration_s) : budget_s;
    PlaceApproachPlanAttempt attempt;
    attempt.candidate_index = candidate_index;
    attempt.seed_index = candidates[candidate_index].seed_index;
    attempt.planning_budget_s = budget_s;
    attempt.planning_duration_s = duration_s;
    attempt.trajectory_duration_s = std::isfinite(observation.trajectory_duration_s) ?
      std::max(0.0, observation.trajectory_duration_s) :
      std::numeric_limits<double>::infinity();
    attempt.result_code = observation.result_code;
    attempt.trajectory_points = observation.trajectory_points;
    attempt.time_parameterized = observation.time_parameterized;
    attempt.time_parameterization_reason = observation.time_parameterization_reason;
    attempt.success = observation.success && observation.time_parameterized &&
      observation.trajectory_points > 0U &&
      std::isfinite(observation.trajectory_duration_s) &&
      observation.trajectory_duration_s > 0.0;
    result.attempts.push_back(attempt);
    result.elapsed_planning_s += duration_s;
    if (attempt.success) {
      const auto & candidate = candidates[candidate_index];
      const auto pose_deviation = candidate.nominal_pose_deviation;
      const auto joint_cost = candidate.ranking_cost;
      const auto trajectory_duration = attempt.trajectory_duration_s;
      const auto orientation_error = candidate.tool_orientation_preference_error_rad;
      const bool better = selection_policy ==
        PlacePlanSelectionPolicy::SHORTEST_PLANNING_DURATION ?
        (!result.success || duration_s < selected_planning_duration - 1e-9) :
        ([&]() {
          const bool orientation_is_better = orientation_error &&
            (*orientation_error < selected_orientation_error - 1e-9);
          const bool orientation_is_tied = !orientation_error ||
            std::abs(*orientation_error - selected_orientation_error) <= 1e-9;
          return !result.success || orientation_is_better ||
            (orientation_is_tied && (pose_deviation < selected_pose_deviation - 1e-9 ||
            (std::abs(pose_deviation - selected_pose_deviation) <= 1e-9 &&
            (joint_cost < selected_joint_cost - 1e-9 ||
            (std::abs(joint_cost - selected_joint_cost) <= 1e-9 &&
            trajectory_duration < selected_trajectory_duration - 1e-9)))));
        })();
      if (better) {
        result.success = true;
        result.selected_candidate_index = candidate_index;
        selected_planning_duration = duration_s;
        selected_pose_deviation = pose_deviation;
        selected_joint_cost = joint_cost;
        selected_trajectory_duration = trajectory_duration;
        selected_orientation_error = orientation_error.value_or(
          std::numeric_limits<double>::infinity());
      }
    }
  }
  return result;
}

PlaceApproachPlanResult plan_place_candidates_with_fallback(
  const std::vector<PlaceApproachIkCandidate> & candidates,
  const std::vector<std::size_t> & primary_candidate_indices,
  const std::vector<std::size_t> & fallback_candidate_indices,
  const PlaceApproachIkPolicy & policy,
  const PlaceApproachPlanFunction & plan_candidate,
  PlacePlanSelectionPolicy selection_policy,
  std::size_t max_primary_candidates,
  std::size_t max_fallback_candidates)
{
  auto result = plan_ranked_place_approach_candidates(
    candidates, primary_candidate_indices, policy, plan_candidate,
    selection_policy, max_primary_candidates);
  if (result.success || fallback_candidate_indices.empty() ||
    result.elapsed_planning_s >= policy.total_planning_budget_s)
  {
    return result;
  }

  auto fallback_policy = policy;
  fallback_policy.total_planning_budget_s = std::max(
    0.0, policy.total_planning_budget_s - result.elapsed_planning_s);
  const auto fallback_result = plan_ranked_place_approach_candidates(
    candidates, fallback_candidate_indices, fallback_policy, plan_candidate,
    selection_policy, max_fallback_candidates);
  result.elapsed_planning_s += fallback_result.elapsed_planning_s;
  result.attempts.insert(
    result.attempts.end(), fallback_result.attempts.begin(), fallback_result.attempts.end());
  if (fallback_result.success) {
    result.success = true;
    result.selected_candidate_index = fallback_result.selected_candidate_index;
  }
  return result;
}

std::vector<std::size_t> interleave_place_candidates_by_pose(
  const std::vector<PlaceApproachIkCandidate> & candidates,
  const std::vector<std::size_t> & ranked_candidate_indices,
  std::size_t pose_candidate_count)
{
  std::map<std::size_t, std::vector<std::size_t>> grouped;
  for (const auto index : ranked_candidate_indices) {
    if (index < candidates.size() && candidates[index].valid &&
      candidates[index].object_pose_candidate_index < pose_candidate_count)
    {
      grouped[candidates[index].object_pose_candidate_index].push_back(index);
    }
  }

  std::vector<std::size_t> ordered;
  ordered.reserve(ranked_candidate_indices.size());
  for (std::size_t rank = 0U;; ++rank) {
    bool appended = false;
    for (const auto & [pose_index, indices] : grouped) {
      (void)pose_index;
      if (rank < indices.size()) {
        ordered.push_back(indices[rank]);
        appended = true;
      }
    }
    if (!appended) {
      break;
    }
  }
  return ordered;
}

}  // namespace arm_cell_motion_moveit2
