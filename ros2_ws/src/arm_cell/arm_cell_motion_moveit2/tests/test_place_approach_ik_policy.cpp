#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "arm_cell_motion_moveit2/place_approach_ik_policy.hpp"

namespace arm_cell_motion_moveit2
{
namespace
{
std::vector<JointContinuityLimit> wide_limits(std::size_t count)
{
  return std::vector<JointContinuityLimit>(count, {-6.2832, 6.2832, true});
}

PlaceApproachIkCandidate candidate(
  std::size_t seed_index, const std::vector<double> & goal)
{
  PlaceApproachIkCandidate result;
  result.seed_index = seed_index;
  result.seed_source = "test";
  result.seed_positions = {0.0};
  result.ik_success = true;
  result.raw_solution = goal;
  result.endpoint_state_valid = true;
  EXPECT_TRUE(prepare_place_approach_candidate(result, {0.0}, wide_limits(goal.size())));
  std::vector<PlaceApproachIkCandidate> ranked_candidate{result};
  EXPECT_EQ(
    rank_place_approach_candidates(
      ranked_candidate, {0.0}, wide_limits(goal.size())).size(), 1U);
  result = ranked_candidate.front();
  return result;
}

TEST(PlaceApproachIkPolicy, PlaceObjectPoseCandidatesStayWithinRecipeEnvelope)
{
  geometry_msgs::msg::PoseStamped desired;
  desired.header.frame_id = "base_link";
  desired.pose.position.x = 0.12;
  desired.pose.position.y = -0.34;
  desired.pose.position.z = 0.56;
  desired.pose.orientation.w = 1.0;
  constexpr double position_tolerance = 0.01;
  constexpr double orientation_tolerance = 0.034906585;

  const auto candidates = generate_place_object_pose_candidates(
    desired, position_tolerance, orientation_tolerance,
    PlaceOrientationConstraint::BOUNDED);
  const auto groups = group_bounded_place_pose_candidates(candidates, desired);

  EXPECT_EQ(groups.primary_orientation_indices.size(), 7U);
  EXPECT_EQ(groups.position_fallback_indices.size(), 6U);
  EXPECT_DOUBLE_EQ(candidates.front().nominal_deviation, 0.0);
  EXPECT_DOUBLE_EQ(candidates.front().object_pose.pose.position.x, 0.12);
  EXPECT_DOUBLE_EQ(candidates.front().object_pose.pose.position.y, -0.34);
  EXPECT_DOUBLE_EQ(candidates.front().object_pose.pose.position.z, 0.56);
  double previous_deviation = -1.0;
  for (const auto & candidate_pose : candidates) {
    const auto & pose = candidate_pose.object_pose.pose;
    const double dx = pose.position.x - desired.pose.position.x;
    const double dy = pose.position.y - desired.pose.position.y;
    const double dz = pose.position.z - desired.pose.position.z;
    const double position_deviation = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double dot = std::abs(
      pose.orientation.x * desired.pose.orientation.x +
      pose.orientation.y * desired.pose.orientation.y +
      pose.orientation.z * desired.pose.orientation.z +
      pose.orientation.w * desired.pose.orientation.w);
    const double angle = 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
    EXPECT_LE(position_deviation, position_tolerance + 1e-9);
    EXPECT_LE(angle, orientation_tolerance + 1e-9);
    EXPECT_GE(candidate_pose.nominal_deviation, previous_deviation);
    previous_deviation = candidate_pose.nominal_deviation;
  }
}

TEST(PlaceApproachIkPolicy, FixedUsesNominalOrientationAndFreeIncludesSideOnCandidates)
{
  geometry_msgs::msg::PoseStamped desired;
  desired.header.frame_id = "base_link";
  desired.pose.orientation.w = 1.0;
  const auto fixed = generate_place_object_pose_candidates(
    desired, 0.01, 0.7, PlaceOrientationConstraint::FIXED);
  ASSERT_EQ(fixed.size(), 7U);
  for (const auto & candidate : fixed) {
    EXPECT_DOUBLE_EQ(candidate.object_pose.pose.orientation.w, 1.0);
    EXPECT_DOUBLE_EQ(candidate.object_pose.pose.orientation.x, 0.0);
    EXPECT_DOUBLE_EQ(candidate.object_pose.pose.orientation.y, 0.0);
    EXPECT_DOUBLE_EQ(candidate.object_pose.pose.orientation.z, 0.0);
  }

  const auto free = generate_place_object_pose_candidates(
    desired, 0.01, 0.0, PlaceOrientationConstraint::FREE);
  ASSERT_EQ(free.size(), 13U);
  EXPECT_TRUE(std::any_of(free.begin(), free.end(), [](const auto & candidate) {
    return std::abs(candidate.object_pose.pose.orientation.x) > 0.7 ||
           std::abs(candidate.object_pose.pose.orientation.y) > 0.7;
  }));
}

TEST(PlaceApproachIkPolicy, BoundedHasSevenTenDegreeCandidatesUnlikeFree)
{
  geometry_msgs::msg::PoseStamped desired;
  desired.header.frame_id = "base_link";
  desired.pose.orientation.x = 0.7071067811865476;
  desired.pose.orientation.w = 0.7071067811865476;
  constexpr double tolerance = 0.17453292519943295;
  const auto bounded = generate_place_object_pose_candidates(
    desired, 0.01, tolerance, PlaceOrientationConstraint::BOUNDED);
  const auto groups = group_bounded_place_pose_candidates(bounded, desired);
  geometry_msgs::msg::PoseStamped free_desired;
  free_desired.pose.orientation.w = 1.0;
  const auto free = generate_place_object_pose_candidates(
    free_desired, 0.01, tolerance, PlaceOrientationConstraint::FREE);

  EXPECT_EQ(groups.primary_orientation_indices.size(), 7U);
  EXPECT_EQ(groups.position_fallback_indices.size(), 6U);
  for (const auto index : groups.primary_orientation_indices) {
    const auto & position = bounded[index].object_pose.pose.position;
    EXPECT_NEAR(position.x, desired.pose.position.x, 1e-12);
    EXPECT_NEAR(position.y, desired.pose.position.y, 1e-12);
    EXPECT_NEAR(position.z, desired.pose.position.z, 1e-12);
  }
  for (const auto index : groups.position_fallback_indices) {
    const auto & position = bounded[index].object_pose.pose.position;
    const auto & orientation = bounded[index].object_pose.pose.orientation;
    EXPECT_NEAR(orientation.x, desired.pose.orientation.x, 1e-12);
    EXPECT_NEAR(orientation.y, desired.pose.orientation.y, 1e-12);
    EXPECT_NEAR(orientation.z, desired.pose.orientation.z, 1e-12);
    EXPECT_NEAR(orientation.w, desired.pose.orientation.w, 1e-12);
    EXPECT_NEAR(std::abs(position.x - desired.pose.position.x) +
      std::abs(position.y - desired.pose.position.y) +
      std::abs(position.z - desired.pose.position.z), 0.01, 1e-12);
  }
  EXPECT_TRUE(std::any_of(bounded.begin(), bounded.end(), [](const auto & candidate) {
    const auto & orientation = candidate.object_pose.pose.orientation;
    return std::abs(orientation.x - 0.7071067811865476) < 1e-12 &&
           std::abs(orientation.w - 0.7071067811865476) < 1e-12;
  }));
  std::vector<geometry_msgs::msg::Quaternion> unique_orientations;
  std::size_t translated_position_count = 0U;
  for (const auto & candidate : bounded) {
    const auto & orientation = candidate.object_pose.pose.orientation;
    const double dot = std::abs(
      orientation.x * desired.pose.orientation.x +
      orientation.y * desired.pose.orientation.y +
      orientation.z * desired.pose.orientation.z +
      orientation.w * desired.pose.orientation.w);
    const double angle = 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
    EXPECT_LE(angle, tolerance + 1e-9);
    const auto & position = candidate.object_pose.pose.position;
    const double dx = position.x - desired.pose.position.x;
    const double dy = position.y - desired.pose.position.y;
    const double dz = position.z - desired.pose.position.z;
    EXPECT_LE(std::sqrt(dx * dx + dy * dy + dz * dz), 0.01 + 1e-9);
    if (std::sqrt(dx * dx + dy * dy + dz * dz) > 1e-9) {
      EXPECT_NEAR(angle, 0.0, 1e-9);
      ++translated_position_count;
    }
    if (angle > 1e-9) {
      EXPECT_NEAR(angle, tolerance, 1e-9);
    }
    const bool already_seen = std::any_of(
      unique_orientations.begin(), unique_orientations.end(), [&](const auto & previous) {
        const double qdot = std::abs(
          orientation.x * previous.x + orientation.y * previous.y +
          orientation.z * previous.z + orientation.w * previous.w);
        return 2.0 * std::acos(std::clamp(qdot, 0.0, 1.0)) < 1e-9;
      });
    if (!already_seen) {
      unique_orientations.push_back(orientation);
    }
  }
  EXPECT_EQ(unique_orientations.size(), 7U);
  EXPECT_EQ(translated_position_count, 6U);
  EXPECT_EQ(free.size(), 13U);
  const bool free_has_pose_outside_bounded_envelope = std::any_of(
    free.begin(), free.end(), [&](const auto & candidate) {
      const auto & orientation = candidate.object_pose.pose.orientation;
      const double dot = std::abs(
        orientation.x * desired.pose.orientation.x +
        orientation.y * desired.pose.orientation.y +
        orientation.z * desired.pose.orientation.z +
        orientation.w * desired.pose.orientation.w);
      return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0)) > tolerance + 1e-9;
    });
  EXPECT_TRUE(free_has_pose_outside_bounded_envelope);
}

TEST(PlaceApproachIkPolicy, QuaternionAngularErrorUsesShortestEquivalentRotation)
{
  geometry_msgs::msg::Quaternion candidate_orientation;
  candidate_orientation.x = 1.0;
  candidate_orientation.w = 0.0;
  geometry_msgs::msg::Quaternion preferred_orientation;
  preferred_orientation.x = -1.0;
  preferred_orientation.w = 0.0;

  const auto error = orientation_angular_distance_rad(
    candidate_orientation, preferred_orientation);

  ASSERT_TRUE(error);
  EXPECT_NEAR(*error, 0.0, 1e-12);
}

TEST(PlaceApproachIkPolicy, CanonicalizesOnlyTheReturnedSeedCopy)
{
  const std::vector<double> measured{0.664287, -0.234499, 1.0006};
  const auto original = measured;

  const auto seed = canonicalize_ik_seed(measured, 0.001);

  ASSERT_EQ(seed.size(), 3U);
  EXPECT_DOUBLE_EQ(seed[0], 0.664);
  EXPECT_DOUBLE_EQ(seed[1], -0.234);
  EXPECT_NEAR(seed[2], 1.001, 1e-12);
  EXPECT_EQ(measured, original);
}

TEST(PlaceApproachIkPolicy, NearbyMeasuredStatesShareCanonicalSeed)
{
  EXPECT_EQ(
    canonicalize_ik_seed({0.664287, -0.812499}, 0.001),
    canonicalize_ik_seed({0.664412, -0.812401}, 0.001));
}

TEST(PlaceApproachIkPolicy, GeneratesStableBoundedM0609SeedOrder)
{
  const PlaceApproachIkPolicy policy;
  const std::vector<std::string> names{"joint_1", "joint_2", "joint_3"};
  const std::vector<JointContinuityLimit> limits{
    {-6.2832, 6.2832, true}, {-6.2832, 6.2832, true}, {-2.618, 2.618, false}};

  const auto first = generate_place_approach_ik_seeds(
    {0.664287, -0.2, 0.5}, names, limits, policy);
  const auto second = generate_place_approach_ik_seeds(
    {0.664287, -0.2, 0.5}, names, limits, policy);

  ASSERT_EQ(first.size(), 5U);
  ASSERT_EQ(first.size(), second.size());
  EXPECT_EQ(first[0].source, "canonical_current");
  EXPECT_EQ(first[1].source, "joint_1_plus");
  EXPECT_EQ(first[2].source, "joint_1_minus");
  EXPECT_EQ(first[3].source, "joint_3_plus");
  EXPECT_EQ(first[4].source, "joint_3_minus");
  EXPECT_DOUBLE_EQ(first[0].joints[0], 0.664);
  EXPECT_DOUBLE_EQ(first[1].joints[0], 0.914);
  EXPECT_DOUBLE_EQ(first[2].joints[0], 0.414);
  EXPECT_DOUBLE_EQ(first[3].joints[2], 0.75);
  EXPECT_DOUBLE_EQ(first[4].joints[2], 0.25);
  for (std::size_t index = 0; index < first.size(); ++index) {
    EXPECT_EQ(first[index].index, second[index].index);
    EXPECT_EQ(first[index].source, second[index].source);
    EXPECT_EQ(first[index].joints, second[index].joints);
  }
}

TEST(PlaceApproachIkPolicy, SkipsSeedVariationsOutsideJointBounds)
{
  const PlaceApproachIkPolicy policy;
  const auto seeds = generate_place_approach_ik_seeds(
    {6.20, 0.0, 2.50}, {"joint_1", "joint_2", "joint_3"},
    {{-6.2832, 6.2832, true}, {-6.2832, 6.2832, true}, {-2.618, 2.618, false}}, policy);

  ASSERT_EQ(seeds.size(), 3U);
  EXPECT_EQ(seeds[0].source, "canonical_current");
  EXPECT_EQ(seeds[1].source, "joint_1_minus");
  EXPECT_EQ(seeds[2].source, "joint_3_minus");
}

TEST(PlaceApproachIkPolicy, RejectsFailedIkAndInvalidEndpointCandidates)
{
  std::vector<PlaceApproachIkCandidate> candidates(3);
  candidates[0].seed_index = 0;
  candidates[0].seed_source = "ik_failed";
  candidates[0].ik_success = false;
  candidates[0].rejection_reason = "IK_FAILED";

  candidates[1].seed_index = 1;
  candidates[1].ik_success = true;
  candidates[1].raw_solution = {0.2};
  ASSERT_TRUE(
    prepare_place_approach_candidate(
      candidates[1], {0.0}, {{-1.0, 1.0, false}}));
  candidates[1].endpoint_state_valid = false;
  candidates[1].rejection_reason = "endpoint_collision";

  candidates[2] = candidate(2, {0.1});
  const auto ranked = rank_place_approach_candidates(
    candidates, {0.0}, {{-1.0, 1.0, false}});

  ASSERT_EQ(ranked.size(), 1U);
  EXPECT_EQ(ranked.front(), 2U);
  EXPECT_EQ(candidates[0].rejection_reason, "IK_FAILED");
  EXPECT_EQ(candidates[1].rejection_reason, "endpoint_collision");
}

TEST(PlaceApproachIkPolicy, RemovesWrapEquivalentDuplicateSolutions)
{
  auto direct = candidate(0, {0.5});
  auto wrapped = candidate(1, {-5.783185307179586});
  std::vector<PlaceApproachIkCandidate> candidates{direct, wrapped};

  const auto ranked = rank_place_approach_candidates(
    candidates, {0.0}, {{-6.2832, 6.2832, true}});

  ASSERT_EQ(ranked.size(), 1U);
  EXPECT_EQ(ranked.front(), 0U);
  EXPECT_FALSE(candidates[0].duplicate_rejected);
  EXPECT_TRUE(candidates[1].duplicate_rejected);
  EXPECT_EQ(candidates[1].rejection_reason, "WRAP_EQUIVALENT_DUPLICATE");
}

TEST(PlaceApproachIkPolicy, RanksByMeasuredWrapAwareDistanceAndStableSeedTieBreak)
{
  auto farther = candidate(0, {0.4});
  auto tie_later_seed = candidate(4, {-0.3});
  auto nearest = candidate(3, {0.1});
  auto tie_earlier_seed = candidate(2, {0.3});
  std::vector<PlaceApproachIkCandidate> candidates{
    farther, tie_later_seed, nearest, tie_earlier_seed};

  const auto ranked = rank_place_approach_candidates(
    candidates, {0.0}, {{-1.0, 1.0, false}});

  ASSERT_EQ(ranked.size(), 4U);
  EXPECT_EQ(ranked[0], 2U);
  EXPECT_EQ(ranked[1], 3U);
  EXPECT_EQ(ranked[2], 1U);
  EXPECT_EQ(ranked[3], 0U);
  EXPECT_DOUBLE_EQ(candidates[1].ranking_cost, candidates[3].ranking_cost);
}

TEST(PlaceApproachIkPolicy, PreferredToolOrientationRanksFeasibleCandidatesFirst)
{
  auto nominal = candidate(0, {0.1});
  nominal.nominal_pose_deviation = 0.0;
  nominal.tool_orientation_preference_error_rad = 0.2;
  auto preferred = candidate(1, {0.8});
  preferred.nominal_pose_deviation = 0.1;
  preferred.tool_orientation_preference_error_rad = 0.01;
  std::vector<PlaceApproachIkCandidate> candidates{nominal, preferred};

  const auto ranked = rank_place_approach_candidates(
    candidates, {0.0}, {{-1.0, 1.0, false}});

  ASSERT_EQ(ranked.size(), 2U);
  EXPECT_EQ(ranked[0], 1U);
  EXPECT_EQ(ranked[1], 0U);
}

TEST(PlaceApproachIkPolicy, PreferenceCannotRescueInvalidIkOrCollisionCandidates)
{
  auto collision = candidate(0, {0.1});
  collision.endpoint_state_valid = false;
  collision.tool_orientation_preference_error_rad = 0.0;
  auto ik_failed = candidate(1, {0.2});
  ik_failed.ik_success = false;
  ik_failed.tool_orientation_preference_error_rad = 0.0;
  auto feasible = candidate(2, {0.3});
  feasible.tool_orientation_preference_error_rad = 0.3;
  std::vector<PlaceApproachIkCandidate> candidates{collision, ik_failed, feasible};

  const auto ranked = rank_place_approach_candidates(
    candidates, {0.0}, {{-1.0, 1.0, false}});

  ASSERT_EQ(ranked.size(), 1U);
  EXPECT_EQ(ranked.front(), 2U);
}

TEST(PlaceApproachIkPolicy, EqualPreferenceErrorsRetainExistingDeterministicCriteria)
{
  auto farther = candidate(0, {0.4});
  farther.tool_orientation_preference_error_rad = 0.05;
  auto nearer = candidate(1, {0.1});
  nearer.tool_orientation_preference_error_rad = 0.05;
  std::vector<PlaceApproachIkCandidate> candidates{farther, nearer};

  const auto ranked = rank_place_approach_candidates(
    candidates, {0.0}, {{-1.0, 1.0, false}});

  ASSERT_EQ(ranked.size(), 2U);
  EXPECT_EQ(ranked[0], 1U);
  EXPECT_EQ(ranked[1], 0U);
}

TEST(PlaceApproachIkPolicy, TriesNextCandidateAfterPlanningFailure)
{
  const std::vector<PlaceApproachIkCandidate> candidates{
    candidate(0, {0.1}), candidate(1, {0.2})};
  const std::vector<std::size_t> ranked{0U, 1U};
  std::vector<std::size_t> attempted;
  PlaceApproachIkPolicy policy;
  policy.per_candidate_planning_budget_s = 1.0;
  policy.total_planning_budget_s = 2.0;

  const auto result = plan_ranked_place_approach_candidates(
    candidates, ranked, policy,
    [&attempted](const PlaceApproachIkCandidate & value, double budget_s) {
      attempted.push_back(value.seed_index);
      EXPECT_DOUBLE_EQ(budget_s, 1.0);
      const bool ready = value.seed_index == 1U;
      return PlaceApproachPlanObservation{
        ready, ready ? 1 : -6, ready ? 17U : 0U, 0.4, ready ? 0.5 : 0.0,
        ready, ready ? "valid" : "planning_failed"};
    });

  EXPECT_EQ(attempted, (std::vector<std::size_t>{0U, 1U}));
  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.selected_candidate_index, 1U);
  ASSERT_EQ(result.attempts.size(), 2U);
  EXPECT_DOUBLE_EQ(result.attempts[0].planning_budget_s, 1.0);
  EXPECT_EQ(result.attempts[0].trajectory_points, 0U);
  EXPECT_EQ(result.attempts[1].trajectory_points, 17U);
}

TEST(PlaceApproachIkPolicy, SelectedPlanPrefersOrientationBeforePoseDistanceAndDuration)
{
  auto nominal = candidate(0, {0.1});
  nominal.nominal_pose_deviation = 0.0;
  nominal.tool_orientation_preference_error_rad = 0.2;
  auto preferred = candidate(1, {0.8});
  preferred.nominal_pose_deviation = 0.1;
  preferred.tool_orientation_preference_error_rad = 0.01;
  const std::vector<PlaceApproachIkCandidate> candidates{nominal, preferred};
  const std::vector<std::size_t> ranked{1U, 0U};
  PlaceApproachIkPolicy policy;

  const auto result = plan_ranked_place_approach_candidates(
    candidates, ranked, policy,
    [](const PlaceApproachIkCandidate & value, double budget_s) {
      const double duration = value.seed_index == 1U ? 2.0 : 0.5;
      return PlaceApproachPlanObservation{
        true, 1, 1U, budget_s, duration, true, "valid"};
    });

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.selected_candidate_index, 1U);
}

TEST(PlaceApproachIkPolicy, BoundedSelectionUsesShortestPlanningDuration)
{
  auto nominal = candidate(0U, {0.1});
  nominal.nominal_pose_deviation = 0.0;
  nominal.ranking_cost = 0.1;
  auto rotated = candidate(1U, {0.8});
  rotated.nominal_pose_deviation = 0.0;
  rotated.ranking_cost = 0.2;
  const std::vector<PlaceApproachIkCandidate> candidates{nominal, rotated};
  const std::vector<std::size_t> ranked{0U, 1U};
  PlaceApproachIkPolicy policy;
  const auto observe = [](const PlaceApproachIkCandidate & value, double) {
      const bool is_nominal = value.seed_index == 0U;
      return PlaceApproachPlanObservation{
        true, 1, 1U, is_nominal ? 0.8 : 0.2, is_nominal ? 0.5 : 2.0,
        true, "valid"};
    };

  const auto bounded = plan_ranked_place_approach_candidates(
    candidates, ranked, policy, observe,
    PlacePlanSelectionPolicy::SHORTEST_PLANNING_DURATION);
  const auto free = plan_ranked_place_approach_candidates(
    candidates, ranked, policy, observe,
    PlacePlanSelectionPolicy::EXISTING_RANKING);

  ASSERT_TRUE(bounded.success);
  EXPECT_EQ(bounded.selected_candidate_index, 1U);
  ASSERT_TRUE(free.success);
  EXPECT_EQ(free.selected_candidate_index, 0U);
}

TEST(PlaceApproachIkPolicy, BoundedPlanOrderInterleavesOrientationCandidates)
{
  std::vector<PlaceApproachIkCandidate> candidates(4U);
  candidates[0].valid = true;
  candidates[0].object_pose_candidate_index = 0U;
  candidates[1].valid = true;
  candidates[1].object_pose_candidate_index = 0U;
  candidates[2].valid = true;
  candidates[2].object_pose_candidate_index = 1U;
  candidates[3].valid = true;
  candidates[3].object_pose_candidate_index = 2U;

  const auto ordered = interleave_place_candidates_by_pose(
    candidates, {0U, 1U, 2U, 3U}, 3U);

  EXPECT_EQ(ordered, (std::vector<std::size_t>{0U, 2U, 3U, 1U}));
}

TEST(PlaceApproachIkPolicy, BoundedSuccessSkipsPositionFallbackPlanning)
{
  const std::vector<PlaceApproachIkCandidate> candidates{
    candidate(0U, {0.1}), candidate(1U, {0.2}), candidate(2U, {0.3})};
  PlaceApproachIkPolicy policy;
  std::vector<std::size_t> planned;

  const auto result = plan_place_candidates_with_fallback(
    candidates, {0U, 1U}, {2U}, policy,
    [&](const PlaceApproachIkCandidate & value, double budget_s) {
      planned.push_back(value.seed_index);
      return PlaceApproachPlanObservation{
        true, 1, 1U, value.seed_index == 0U ? 0.2 : 0.1, budget_s, true, "valid"};
    }, PlacePlanSelectionPolicy::SHORTEST_PLANNING_DURATION, 2U, 1U);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.selected_candidate_index, 1U);
  EXPECT_EQ(planned, (std::vector<std::size_t>{0U, 1U}));
  EXPECT_EQ(result.attempts.size(), 2U);
}

TEST(PlaceApproachIkPolicy, BoundedFailurePlansPositionFallbackAfterPrimary)
{
  const std::vector<PlaceApproachIkCandidate> candidates{
    candidate(0U, {0.1}), candidate(1U, {0.2}), candidate(2U, {0.3})};
  PlaceApproachIkPolicy policy;
  policy.total_planning_budget_s = 5.0;
  std::vector<std::size_t> planned;
  std::vector<double> budgets;

  const auto result = plan_place_candidates_with_fallback(
    candidates, {0U, 1U}, {2U}, policy,
    [&](const PlaceApproachIkCandidate & value, double budget_s) {
      planned.push_back(value.seed_index);
      budgets.push_back(budget_s);
      const bool fallback = value.seed_index == 2U;
      return PlaceApproachPlanObservation{
        fallback, fallback ? 1 : -1, fallback ? 1U : 0U,
        fallback ? 0.3 : 1.0, fallback ? 0.4 : 0.0, fallback,
        fallback ? "valid" : "planning_failed"};
    }, PlacePlanSelectionPolicy::SHORTEST_PLANNING_DURATION, 2U, 1U);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.selected_candidate_index, 2U);
  EXPECT_EQ(planned, (std::vector<std::size_t>{0U, 1U, 2U}));
  EXPECT_EQ(result.attempts.size(), 3U);
  EXPECT_DOUBLE_EQ(result.attempts.back().planning_budget_s, budgets.back());
  EXPECT_LE(result.elapsed_planning_s, policy.total_planning_budget_s);
}

TEST(PlaceApproachIkPolicy, AppliesConfiguredPlacePlanningBudgets)
{
  const std::vector<PlaceApproachIkCandidate> candidates{
    candidate(0, {0.1}), candidate(1, {0.2})};
  PlaceApproachIkPolicy policy;
  policy.per_candidate_planning_budget_s = 15.0;
  policy.total_planning_budget_s = 60.0;
  std::vector<double> budgets;

  const auto result = plan_ranked_place_approach_candidates(
    candidates, {0U, 1U}, policy,
    [&budgets](const PlaceApproachIkCandidate &, double budget_s) {
      budgets.push_back(budget_s);
      return PlaceApproachPlanObservation{false, -6, 0U, budget_s, 0.0, false, "planning_failed"};
    });

  EXPECT_FALSE(result.success);
  EXPECT_EQ(budgets, (std::vector<double>{15.0, 15.0}));
  ASSERT_EQ(result.attempts.size(), 2U);
  EXPECT_DOUBLE_EQ(result.attempts[0].planning_budget_s, 15.0);
  EXPECT_DOUBLE_EQ(result.attempts[1].planning_budget_s, 15.0);
  EXPECT_DOUBLE_EQ(result.elapsed_planning_s, 30.0);
}

TEST(PlaceApproachIkPolicy, PlannedPlaceCandidatePriorityUsesPoseThenJointsThenDuration)
{
  std::vector<PlaceApproachIkCandidate> candidates(4U);
  candidates[0].valid = true;
  candidates[0].nominal_pose_deviation = 0.0;
  candidates[0].ranking_cost = 0.2;
  candidates[0].seed_index = 0U;
  candidates[1].valid = true;
  candidates[1].nominal_pose_deviation = 0.0;
  candidates[1].ranking_cost = 0.1;
  candidates[1].seed_index = 1U;
  candidates[2].valid = true;
  candidates[2].nominal_pose_deviation = 0.0;
  candidates[2].ranking_cost = 0.1;
  candidates[2].seed_index = 2U;
  candidates[3].valid = true;
  candidates[3].nominal_pose_deviation = 0.5;
  candidates[3].ranking_cost = 0.0;
  candidates[3].seed_index = 3U;

  PlaceApproachIkPolicy policy;
  policy.max_candidates = 4U;
  policy.per_candidate_planning_budget_s = 1.0;
  policy.total_planning_budget_s = 4.0;
  const auto result = plan_ranked_place_approach_candidates(
    candidates, {0U, 1U, 2U, 3U}, policy,
    [](const PlaceApproachIkCandidate & value, double) {
      const double duration = value.seed_index == 1U ? 4.0 :
        (value.seed_index == 2U ? 2.0 : 0.1);
      return PlaceApproachPlanObservation{true, 1, 2U, 0.1, duration, true, "valid"};
    });

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.selected_candidate_index, 2U);
  EXPECT_EQ(result.attempts.size(), 4U);
}

TEST(PlaceApproachIkPolicy, RejectsUnexecutableCandidatesAndContinuesToExecutableCandidate)
{
  const std::vector<PlaceApproachIkCandidate> candidates{
    candidate(0, {0.1}), candidate(1, {0.2}), candidate(2, {0.3}),
    candidate(3, {0.4})};
  PlaceApproachIkPolicy policy;
  policy.max_candidates = 4U;
  policy.per_candidate_planning_budget_s = 1.0;
  policy.total_planning_budget_s = 4.0;

  const auto result = plan_ranked_place_approach_candidates(
    candidates, {0U, 1U, 2U, 3U}, policy,
    [](const PlaceApproachIkCandidate & value, double) {
      const double duration = value.seed_index == 0U ? 0.25 :
        (value.seed_index == 1U ? 0.0 :
        (value.seed_index == 2U ? std::numeric_limits<double>::infinity() : 0.5));
      const bool time_parameterized = value.seed_index != 0U;
      return PlaceApproachPlanObservation{
        true, 1, 18U, 0.1, duration, time_parameterized,
        value.seed_index == 0U ? "time_parameterization_failed" :
        (value.seed_index == 3U ? "valid" : "invalid_duration")};
    });

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.selected_candidate_index, 3U);
  ASSERT_EQ(result.attempts.size(), 4U);
  EXPECT_FALSE(result.attempts[0].success);
  EXPECT_FALSE(result.attempts[0].time_parameterized);
  EXPECT_DOUBLE_EQ(result.attempts[0].trajectory_duration_s, 0.25);
  EXPECT_FALSE(result.attempts[1].success);
  EXPECT_TRUE(result.attempts[1].time_parameterized);
  EXPECT_DOUBLE_EQ(result.attempts[1].trajectory_duration_s, 0.0);
  EXPECT_FALSE(result.attempts[2].success);
  EXPECT_TRUE(result.attempts[2].time_parameterized);
  EXPECT_TRUE(std::isinf(result.attempts[2].trajectory_duration_s));
  EXPECT_TRUE(result.attempts[3].success);
  EXPECT_DOUBLE_EQ(result.attempts[3].trajectory_duration_s, 0.5);
}

TEST(PlaceApproachIkPolicy, StopsAfterBoundedAttemptsAndFailsClosed)
{
  std::vector<PlaceApproachIkCandidate> candidates;
  for (std::size_t index = 0; index < 7U; ++index) {
    candidates.push_back(candidate(index, {0.1 + 0.1 * static_cast<double>(index)}));
  }
  std::vector<std::size_t> ranked{0U, 1U, 2U, 3U, 4U, 5U, 6U};
  std::vector<std::size_t> attempted;
  PlaceApproachIkPolicy policy;
  policy.max_candidates = 5U;
  policy.per_candidate_planning_budget_s = 1.0;
  policy.total_planning_budget_s = 2.5;

  const auto result = plan_ranked_place_approach_candidates(
    candidates, ranked, policy,
    [&attempted](const PlaceApproachIkCandidate & value, double budget_s) {
      attempted.push_back(value.seed_index);
      EXPECT_LE(budget_s, 1.0);
      return PlaceApproachPlanObservation{false, -6, 0U, budget_s, 0.0, false, "planning_failed"};
    });

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.attempts.size(), 3U);
  EXPECT_EQ(attempted, (std::vector<std::size_t>{0U, 1U, 2U}));
  EXPECT_DOUBLE_EQ(result.elapsed_planning_s, 2.5);
}

}  // namespace
}  // namespace arm_cell_motion_moveit2
