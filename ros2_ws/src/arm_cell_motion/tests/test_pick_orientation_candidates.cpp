#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include <arm_cell_interfaces/action/execute_task.hpp>
#include "arm_cell_motion/pick_approach_geometry.hpp"
#include "arm_cell_motion/pick_collision_seed_policy.hpp"
#include "arm_cell_motion/pick_grasp_pose_candidates.hpp"
#include "arm_cell_motion/pick_rectangle_geometry.hpp"
#include "arm_cell_motion/pick_search_budget.hpp"
#include "arm_cell_motion/pick_search_controller.hpp"
#include "arm_cell_motion/pick_symmetry.hpp"

namespace arm_cell_motion
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

TEST(PickRectangleGeometry, EstimatedRectangleIsAvailableWithoutObservedOutline)
{
  EXPECT_EQ(classify_rectangle_geometry(true, true, 0.08, 0.07, 0.07),
    RectangleGeometryStatus::AVAILABLE);
  EXPECT_STREQ(rectangle_geometry_status_name(RectangleGeometryStatus::AVAILABLE), "available");
}

TEST(PickRectangleGeometry, MissingAndInvalidGeometryHaveDistinctStatuses)
{
  EXPECT_EQ(classify_rectangle_geometry(false, false, 0.0, 0.0, 0.0),
    RectangleGeometryStatus::UNAVAILABLE);
  EXPECT_EQ(classify_rectangle_geometry(true, false, 0.08, 0.07, 0.07),
    RectangleGeometryStatus::INVALID);
  EXPECT_EQ(classify_rectangle_geometry(true, true, 0.08, 0.0, 0.07),
    RectangleGeometryStatus::INVALID);
}

TEST(PickOrientationCandidates, TwofoldComposesObjectOrientationWithTcpRotationOnce)
{
  const Eigen::Quaterniond object_orientation = Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond object_to_tcp_rotation(
    Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()));
  const Eigen::Quaterniond target_tcp_orientation = compose_object_tcp_orientation(
    object_orientation, object_to_tcp_rotation);

  const auto seed = pick_orientation_seed(
    object_orientation, target_tcp_orientation,
    arm_cell_interfaces::action::ExecuteTask::Goal::ORIENTATION_TWOFOLD, false);
  const bool compose_object_tcp = pick_candidate_requires_object_tcp_composition(
    arm_cell_interfaces::action::ExecuteTask::Goal::ORIENTATION_TWOFOLD, false);
  ASSERT_TRUE(compose_object_tcp);
  const auto tcp_orientation = compose_object_tcp ?
    compose_object_tcp_orientation(seed, object_to_tcp_rotation) : seed;
  const auto tcp_z_axis = tcp_orientation * Eigen::Vector3d::UnitZ();

  EXPECT_NEAR(tcp_z_axis.x(), 0.0, 1e-12);
  EXPECT_NEAR(tcp_z_axis.y(), 0.0, 1e-12);
  EXPECT_NEAR(tcp_z_axis.z(), -1.0, 1e-12);
}

TEST(PickOrientationCandidates, TwofoldRectangleComposesDownwardTcpOrientationOnce)
{
  const Eigen::Quaterniond object_orientation = Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond object_to_tcp_rotation(
    Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()));
  const Eigen::Quaterniond target_tcp_orientation = compose_object_tcp_orientation(
    object_orientation, object_to_tcp_rotation);

  const auto seed = pick_orientation_seed(
    object_orientation, target_tcp_orientation,
    arm_cell_interfaces::action::ExecuteTask::Goal::ORIENTATION_TWOFOLD, true);
  ASSERT_TRUE(pick_candidate_requires_object_tcp_composition(
      arm_cell_interfaces::action::ExecuteTask::Goal::ORIENTATION_TWOFOLD, true));
  const auto tcp_orientation = compose_object_tcp_orientation(seed, object_to_tcp_rotation);

  EXPECT_NEAR((tcp_orientation * Eigen::Vector3d::UnitZ()).z(), -1.0, 1e-12);
}

TEST(PickOrientationCandidates, UniqueRectangleUsesObjectOrientationAndComposesOnce)
{
  const Eigen::Quaterniond object_orientation = Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond object_to_tcp_rotation(
    Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()));
  const Eigen::Quaterniond target_tcp_orientation = compose_object_tcp_orientation(
    object_orientation, object_to_tcp_rotation);

  const auto seed = pick_orientation_seed(
    object_orientation, target_tcp_orientation,
    arm_cell_interfaces::action::ExecuteTask::Goal::ORIENTATION_UNIQUE, true);
  ASSERT_TRUE(pick_candidate_requires_object_tcp_composition(
      arm_cell_interfaces::action::ExecuteTask::Goal::ORIENTATION_UNIQUE, true));
  const auto tcp_orientation = compose_object_tcp_orientation(seed, object_to_tcp_rotation);

  EXPECT_NEAR((tcp_orientation * Eigen::Vector3d::UnitZ()).z(), -1.0, 1e-12);
}

TEST(PickOrientationCandidates, TwofoldKeepsTheHalfTurnEquivalent)
{
  const Eigen::Quaterniond object_orientation = Eigen::Quaterniond::Identity();
  const auto candidates = twofold_pick_orientations(object_orientation);

  ASSERT_EQ(candidates.size(), 2U);
  EXPECT_NEAR(std::abs(candidates[0].dot(Eigen::Quaterniond::Identity())), 1.0, 1e-12);
  const Eigen::Quaterniond half_turn(
    Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitZ()));
  EXPECT_NEAR(std::abs(candidates[1].dot(half_turn)), 1.0, 1e-12);
}

TEST(PickOrientationCandidates, FourfoldUsesObjectOrientationSeed)
{
  const Eigen::Quaterniond object_orientation(
    Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()));
  const Eigen::Quaterniond tcp_orientation(
    Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitX()));

  const auto seed = pick_orientation_seed(
    object_orientation, tcp_orientation,
    arm_cell_interfaces::action::ExecuteTask::Goal::ORIENTATION_FOURFOLD, false);

  EXPECT_NEAR(std::abs(seed.dot(object_orientation)), 1.0, 1e-12);
  EXPECT_TRUE(pick_candidate_requires_object_tcp_composition(
      arm_cell_interfaces::action::ExecuteTask::Goal::ORIENTATION_FOURFOLD, false));
}

TEST(PickOrientationCandidates, UniqueAndUnobservableKeepTcpSeed)
{
  const Eigen::Quaterniond object_orientation(
    Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()));
  const Eigen::Quaterniond tcp_orientation(
    Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitX()));

  for (const auto symmetry_order : {
      arm_cell_interfaces::action::ExecuteTask::Goal::ORIENTATION_UNIQUE,
      arm_cell_interfaces::action::ExecuteTask::Goal::ORIENTATION_UNOBSERVABLE})
  {
    const auto seed = pick_orientation_seed(object_orientation, tcp_orientation, symmetry_order, false);
    EXPECT_NEAR(std::abs(seed.dot(tcp_orientation)), 1.0, 1e-12);
    EXPECT_FALSE(pick_candidate_requires_object_tcp_composition(symmetry_order, false));
  }
}

TEST(PickGraspPoseCandidates, EnumeratesDistinctRecipeValidBoxFacePairs)
{
  const Eigen::Quaterniond object_orientation = Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond reference_grasp_tcp_orientation(
    Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()));

  const auto candidates = generate_pick_grasp_pose_candidates(
    object_orientation, reference_grasp_tcp_orientation, 0.07, 0.06, 0.08, 60.0, 85.0, 85.0, 5U);

  ASSERT_EQ(candidates.size(), 3U);
  ASSERT_TRUE(candidates[0].contact.has_value());
  ASSERT_TRUE(candidates[1].contact.has_value());
  ASSERT_TRUE(candidates[2].contact.has_value());
  EXPECT_EQ(candidates[0].contact->closing_axis, 0U);
  EXPECT_EQ(candidates[1].contact->closing_axis, 1U);
  EXPECT_EQ(candidates[2].contact->closing_axis, 2U);
  EXPECT_DOUBLE_EQ(candidates[0].contact->width_mm, 70.0);
  EXPECT_DOUBLE_EQ(candidates[1].contact->width_mm, 60.0);
  EXPECT_DOUBLE_EQ(candidates[2].contact->width_mm, 80.0);
  EXPECT_EQ(candidates[0].contact->face_evidence, BoxFaceEvidence::INFERRED_SIDE_PAIR);
  EXPECT_EQ(candidates[1].contact->face_evidence, BoxFaceEvidence::INFERRED_SIDE_PAIR);
  EXPECT_EQ(
    candidates[2].contact->face_evidence, BoxFaceEvidence::OBSERVED_TOP_INFERRED_BOTTOM);
  EXPECT_NEAR(std::abs(candidates[0].tcp_orientation.dot(reference_grasp_tcp_orientation)), 1.0, 1e-12);
  EXPECT_LT(std::abs(candidates[1].tcp_orientation.dot(candidates[2].tcp_orientation)), 1.0);
  EXPECT_NEAR(
    std::abs((candidates[2].tcp_orientation * Eigen::Vector3d::UnitZ()).dot(
      Eigen::Vector3d::UnitZ())), 0.0, 1e-12);
}

TEST(PickGraspPoseCandidates, AppliesRecipeContactWidthBoundsAndCandidateCap)
{
  const Eigen::Quaterniond object_orientation = Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond reference_grasp_tcp_orientation(
    Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()));

  const auto recipe_filtered = generate_pick_grasp_pose_candidates(
    object_orientation, reference_grasp_tcp_orientation, 0.07, 0.06, 0.08, 65.0, 75.0, 85.0, 5U);
  ASSERT_EQ(recipe_filtered.size(), 1U);
  ASSERT_TRUE(recipe_filtered.front().contact.has_value());
  EXPECT_EQ(recipe_filtered.front().contact->closing_axis, 0U);
  EXPECT_DOUBLE_EQ(recipe_filtered.front().contact->width_mm, 70.0);

  const auto capped = generate_pick_grasp_pose_candidates(
    object_orientation, reference_grasp_tcp_orientation, 0.07, 0.06, 0.08, 60.0, 85.0, 85.0, 1U);
  ASSERT_EQ(capped.size(), 1U);
  ASSERT_TRUE(capped.front().contact.has_value());
  EXPECT_EQ(capped.front().contact->closing_axis, 0U);

  EXPECT_TRUE(generate_pick_grasp_pose_candidates(
    object_orientation, reference_grasp_tcp_orientation, 0.07, 0.06, 0.08, 90.0, 100.0, 85.0, 5U).empty());
}

TEST(PickGraspPoseCandidates, SquareKeepsDifferentAxesDespiteEqualContactWidths)
{
  const Eigen::Quaterniond object_orientation = Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond reference_grasp_tcp_orientation(
    Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()));

  const auto candidates = generate_pick_grasp_pose_candidates(
    object_orientation, reference_grasp_tcp_orientation, 0.07, 0.07, 0.07, 60.0, 85.0, 85.0, 5U);

  ASSERT_EQ(candidates.size(), 3U);
  for (std::size_t lhs = 0U; lhs < candidates.size(); ++lhs) {
    for (std::size_t rhs = lhs + 1U; rhs < candidates.size(); ++rhs) {
      EXPECT_LT(std::abs(candidates[lhs].tcp_orientation.dot(candidates[rhs].tcp_orientation)), 1.0);
    }
    ASSERT_TRUE(candidates[lhs].contact.has_value());
    EXPECT_DOUBLE_EQ(candidates[lhs].contact->width_mm, 70.0);
  }
}

TEST(PickSearchBudget, SharesOneMonotonicBudgetAndCapsIndividualCalls)
{
  using Budget = arm_cell_motion::PickSearchBudget;
  const auto start = Budget::Clock::time_point{};
  const Budget budget(100.0, start);

  EXPECT_DOUBLE_EQ(budget.remaining_s(start + std::chrono::seconds(35)), 65.0);
  EXPECT_DOUBLE_EQ(budget.call_timeout_s(1.0, start + std::chrono::seconds(35)), 1.0);
  EXPECT_DOUBLE_EQ(budget.call_timeout_s(15.0, start + std::chrono::seconds(98)), 2.0);
  EXPECT_DOUBLE_EQ(budget.remaining_s(start + std::chrono::seconds(100)), 0.0);
  EXPECT_DOUBLE_EQ(budget.call_timeout_s(1.0, start + std::chrono::seconds(101)), 0.0);
}

TEST(PickCollisionSeedPolicy, PrioritizesAncestorJointsByKinematicChain)
{
  const std::vector<PickCollisionContact> contacts{
    {"link_4", "amr_tray_rail_rear", "approach_endpoint", false, 0U},
    {"link_5", "amr_tray_rail_rear", "approach_path", true, 7U}};
  const std::vector<std::string> group_variables{
    "joint_base", "elbow_joint", "shoulder_joint", "wrist_roll"};
  const auto influenced = collision_influenced_variables(
    contacts, group_variables, [](const std::string & body) {
      if (body == "link_4") {return std::vector<std::string>{"shoulder_joint", "elbow_joint"};}
      if (body == "link_5") {
        return std::vector<std::string>{"shoulder_joint", "elbow_joint", "wrist_roll"};
      }
      return std::vector<std::string>{};
    });

  EXPECT_EQ(influenced, (std::vector<std::string>{
    "elbow_joint", "shoulder_joint", "wrist_roll"}));
  EXPECT_EQ(
    diversify_pick_ik_seed(
      {0.0, 0.1, 0.2, 0.3}, {1.0, 1.1, 1.2, 1.3}, group_variables, influenced),
    (std::vector<double>{0.0, 1.1, 1.2, 1.3}));
}

TEST(PickCollisionSeedPolicy, MissingOrUnusableCollisionInfoUsesBasicDiversification)
{
  const std::vector<std::string> group_variables{"joint_a", "joint_b"};
  const auto no_contacts = collision_influenced_variables(
    {}, group_variables, [](const std::string &) {return std::vector<std::string>{};});

  EXPECT_TRUE(no_contacts.empty());
  EXPECT_EQ(
    diversify_pick_ik_seed({0.0, 0.1}, {1.0, 1.1}, group_variables, no_contacts),
    (std::vector<double>{1.0, 1.1}));
  EXPECT_EQ(
    diversify_pick_ik_seed({0.0}, {1.0, 1.1}, group_variables, {"joint_a"}),
    (std::vector<double>{1.0, 1.1}));
}

TEST(PickOrientationCandidates, PlanningStopsAtFirstSuccessfulRankedCandidate)
{
  std::vector<std::size_t> attempted;
  const auto selected = first_successful_candidate(
    std::vector<std::size_t>{4U, 2U, 8U}, [&attempted](std::size_t candidate) {
      attempted.push_back(candidate);
      return candidate == 2U;
    });

  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(*selected, 2U);
  EXPECT_EQ(attempted, (std::vector<std::size_t>{4U, 2U}));
}

TEST(PickApproachGeometry, PlanningFrameVerticalKeepsOrientationAndOffsetsUpward)
{
  geometry_msgs::msg::PoseStamped grasp;
  grasp.header.frame_id = "base_link";
  grasp.pose.position.x = 0.3;
  grasp.pose.position.y = 0.2;
  grasp.pose.position.z = 0.4;
  grasp.pose.orientation.x = 0.1;
  grasp.pose.orientation.w = std::sqrt(0.99);
  geometry_msgs::msg::Vector3 insertion_axis;
  insertion_axis.z = -1.0;
  const auto approach = pick_approach_pose(
    grasp, PickApproachAxis::PLANNING_FRAME_VERTICAL, insertion_axis, 0.15);

  EXPECT_DOUBLE_EQ(approach.pose.position.x, grasp.pose.position.x);
  EXPECT_DOUBLE_EQ(approach.pose.position.y, grasp.pose.position.y);
  EXPECT_NEAR(approach.pose.position.z, grasp.pose.position.z + 0.15, 1e-12);
  EXPECT_DOUBLE_EQ(approach.pose.orientation.x, grasp.pose.orientation.x);
  EXPECT_DOUBLE_EQ(approach.pose.orientation.w, grasp.pose.orientation.w);
}

TEST(PickApproachGeometry, TcpInsertionAxisRetainsExistingGeometry)
{
  geometry_msgs::msg::PoseStamped grasp;
  grasp.pose.position.z = 0.4;
  grasp.pose.orientation.w = 1.0;
  geometry_msgs::msg::Vector3 insertion_axis;
  insertion_axis.z = -1.0;
  const auto approach = pick_approach_pose(
    grasp, PickApproachAxis::TCP_INSERTION_AXIS, insertion_axis, 0.15);

  EXPECT_NEAR(approach.pose.position.z, 0.55, 1e-12);
  EXPECT_DOUBLE_EQ(approach.pose.orientation.w, 1.0);
}

TEST(PickApproachGeometry, RotatesTcpInsertionAxisIntoThePlanningFrame)
{
  geometry_msgs::msg::PoseStamped grasp;
  grasp.pose.position.x = 0.4;
  grasp.pose.position.y = -0.2;
  grasp.pose.position.z = 0.3;
  const Eigen::Quaterniond orientation(Eigen::AngleAxisd(
      kPi / 2.0, Eigen::Vector3d::UnitY()));
  grasp.pose.orientation.x = orientation.x();
  grasp.pose.orientation.y = orientation.y();
  grasp.pose.orientation.z = orientation.z();
  grasp.pose.orientation.w = orientation.w();
  geometry_msgs::msg::Vector3 insertion_axis;
  insertion_axis.z = 1.0;

  const auto approach = pick_approach_pose(
    grasp, PickApproachAxis::TCP_INSERTION_AXIS, insertion_axis, 0.15);

  EXPECT_NEAR(approach.pose.position.x, 0.25, 1e-12);
  EXPECT_NEAR(approach.pose.position.y, -0.2, 1e-12);
  EXPECT_NEAR(approach.pose.position.z, 0.3, 1e-12);
  EXPECT_NEAR(approach.pose.orientation.y, grasp.pose.orientation.y, 1e-12);
  EXPECT_NEAR(approach.pose.orientation.w, grasp.pose.orientation.w, 1e-12);
}

TEST(PickGraspSearch, PreservesEligibleCenterFacesWithoutPrivilegedEvaluation)
{
  const Eigen::Quaterniond reference_grasp_tcp_orientation(Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()));
  const auto legacy = generate_pick_grasp_pose_candidates(
    Eigen::Quaterniond::Identity(), reference_grasp_tcp_orientation, 0.07, 0.06, 0.08,
    60.0, 85.0, 85.0, 3U);
  const auto candidates = generate_pick_grasp_search_candidates(
    Eigen::Quaterniond::Identity(), reference_grasp_tcp_orientation, Eigen::Vector3d::Zero(),
    Eigen::Vector3d(0.5, 0.0, 0.0), Eigen::Vector3d::UnitZ(),
    0.07, 0.06, 0.08, 60.0, 85.0, 85.0,
    PickGraspSearchSampling{90.0, 0.0, 1U, 8U, 48U});
  for (const auto & seed : legacy) {
    EXPECT_TRUE(
      std::any_of(
        candidates.begin(), candidates.end(), [&](const auto & candidate) {
          return candidate.grasp_position_offset_object.norm() < 1e-12 &&
          std::abs(candidate.tcp_orientation.dot(seed.tcp_orientation)) > 1.0 - 1e-12;
        }));
  }
}

TEST(PickGraspSearch, ExpandsRotationAndBothFaceTangentsWithinMeasuredBounds)
{
  const Eigen::Quaterniond object(Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond reference_grasp_tcp_orientation =
    object * Eigen::Quaterniond(Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()));
  const auto candidates = generate_pick_grasp_search_candidates(
    object, reference_grasp_tcp_orientation, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.5, -0.2, 0.0),
    Eigen::Vector3d::UnitZ(), 0.07, 0.06, 0.08, 60.0, 85.0, 85.0,
    PickGraspSearchSampling{80.0, 0.5, 3U, 8U, 100U});
  ASSERT_GT(candidates.size(), 3U);
  bool rotated = false;
  bool offset_a = false;
  bool offset_b = false;
  bool diagonal = false;
  const std::array<double, 3> lengths{0.07, 0.06, 0.08};
  for (const auto & candidate : candidates) {
    ASSERT_TRUE(candidate.contact.has_value());
    const auto axis = candidate.contact->closing_axis;
    const auto a = (axis + 1U) % 3U;
    const auto b = (axis + 2U) % 3U;
    EXPECT_NEAR(candidate.grasp_position_offset_object[axis], 0.0, 1e-12);
    EXPECT_LT(std::abs(candidate.grasp_position_offset_object[a]),
      lengths[a] / 2.0);
    EXPECT_LT(std::abs(candidate.grasp_position_offset_object[b]),
      lengths[b] / 2.0);
    rotated |= candidate.grasp_position_offset_object.norm() < 1e-12 &&
      std::abs(candidate.tcp_orientation.dot(reference_grasp_tcp_orientation)) < 0.99;
    offset_a |= std::abs(candidate.grasp_position_offset_object[a]) > 1e-9;
    offset_b |= std::abs(candidate.grasp_position_offset_object[b]) > 1e-9;
    diagonal |= std::abs(candidate.grasp_position_offset_object[a]) > 1e-9 &&
      std::abs(candidate.grasp_position_offset_object[b]) > 1e-9;
  }
  EXPECT_TRUE(rotated);
  EXPECT_TRUE(offset_a);
  EXPECT_TRUE(offset_b);
  EXPECT_TRUE(diagonal);
}

TEST(PickGraspSearch, LimitsOnlyDownwardObjectNormalOffsets)
{
  const Eigen::Quaterniond reference_grasp_tcp_orientation(Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()));
  auto sampling = PickGraspSearchSampling{80.0, 0.5, 3U, 8U, 100U};
  sampling.maximum_downward_position_offset_m = 0.005;
  const auto candidates = generate_pick_grasp_search_candidates(
    Eigen::Quaterniond::Identity(), reference_grasp_tcp_orientation, Eigen::Vector3d::Zero(),
    Eigen::Vector3d(0.5, 0.0, 0.0), Eigen::Vector3d::UnitZ(),
    0.07, 0.06, 0.0745, 60.0, 85.0, 85.0, sampling);
  ASSERT_GT(candidates.size(), 3U);
  bool horizontal_positive = false;
  bool horizontal_negative = false;
  bool upward = false;
  bool downward_at_limit = false;
  for (const auto & candidate : candidates) {
    horizontal_positive |= candidate.grasp_position_offset_object.y() > 0.0074;
    horizontal_negative |= candidate.grasp_position_offset_object.y() < -0.0074;
    upward |= candidate.grasp_position_offset_object.z() > 0.009;
    downward_at_limit |= std::abs(candidate.grasp_position_offset_object.z() + 0.005) < 1e-12;
    EXPECT_GE(candidate.grasp_position_offset_object.z(), -0.005 - 1e-12);
  }
  EXPECT_TRUE(horizontal_positive);
  EXPECT_TRUE(horizontal_negative);
  EXPECT_TRUE(upward);
  EXPECT_TRUE(downward_at_limit);
}

TEST(PickGraspSearch, DownwardLimitUsesRotatedObjectTopNormal)
{
  const Eigen::Quaterniond object(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitY()));
  const Eigen::Quaterniond reference_grasp_tcp_orientation =
    object * Eigen::Quaterniond(Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()));
  auto sampling = PickGraspSearchSampling{80.0, 0.5, 3U, 8U, 100U};
  sampling.maximum_downward_position_offset_m = 0.005;
  const auto candidates = generate_pick_grasp_search_candidates(
    object, reference_grasp_tcp_orientation, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.5, 0.0, 0.0),
    Eigen::Vector3d::UnitZ(), 0.07, 0.06, 0.0745, 60.0, 85.0, 85.0, sampling);
  bool rotated_downward_at_limit = false;
  const Eigen::Vector3d support_normal = object * Eigen::Vector3d::UnitZ();
  for (const auto & candidate : candidates) {
    const double normal_offset = candidate.grasp_position_offset_object.dot(Eigen::Vector3d::UnitZ());
    EXPECT_GE(normal_offset, -0.005 - 1e-12);
    if (std::abs(normal_offset + 0.005) < 1e-12) {
      const Eigen::Vector3d base_offset = object * candidate.grasp_position_offset_object;
      EXPECT_NEAR(base_offset.dot(support_normal), -0.005, 1e-12);
      rotated_downward_at_limit = true;
    }
  }
  EXPECT_TRUE(rotated_downward_at_limit);
}

TEST(PickGraspSearch, FailedDownwardPositionCandidateIsClampedBelowFailureDepth)
{
  const Eigen::Quaterniond reference_grasp_tcp_orientation(Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()));
  auto sampling = PickGraspSearchSampling{80.0, 0.5, 3U, 8U, 100U};
  sampling.maximum_downward_position_offset_m = 0.005;
  const auto candidates = generate_pick_grasp_search_candidates(
    Eigen::Quaterniond::Identity(), reference_grasp_tcp_orientation, Eigen::Vector3d::Zero(),
    Eigen::Vector3d(0.5, 0.0, 0.0), Eigen::Vector3d::UnitZ(),
    0.07, 0.06, 0.0745465, 60.0, 85.0, 85.0, sampling);
  for (const auto & candidate : candidates) {
    EXPECT_FALSE(std::abs(candidate.grasp_position_offset_object.z() + 0.0093183) < 1e-6);
    EXPECT_GE(candidate.grasp_position_offset_object.z(), -0.005 - 1e-12);
  }
}

TEST(PickGraspSearch, TiltBoundAppliesToEveryPose)
{
  const Eigen::Quaterniond reference_grasp_tcp_orientation(Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()));
  const auto candidates = generate_pick_grasp_search_candidates(
    Eigen::Quaterniond::Identity(), reference_grasp_tcp_orientation, Eigen::Vector3d::Zero(),
    Eigen::Vector3d(0.5, 0.0, 0.0), Eigen::Vector3d::UnitZ(),
    0.07, 0.06, 0.08, 60.0, 85.0, 85.0,
    PickGraspSearchSampling{0.0, 0.0, 1U, 8U, 100U});
  ASSERT_FALSE(candidates.empty());
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const Eigen::Vector3d withdrawal =
      -(candidates[i].tcp_orientation * Eigen::Vector3d::UnitZ());
    EXPECT_NEAR(withdrawal.z(), 1.0, 1e-9);
  }
}

TEST(PickGraspSearch, SelectionSpreadsFacesOrientationsAndOffsetsDeterministically)
{
  const Eigen::Quaterniond reference_grasp_tcp_orientation(Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()));
  const auto generate = [&]() {
      return generate_pick_grasp_search_candidates(
        Eigen::Quaterniond::Identity(), reference_grasp_tcp_orientation, Eigen::Vector3d::Zero(),
        Eigen::Vector3d(0.5, 0.0, 0.0), Eigen::Vector3d::UnitZ(),
        0.07, 0.06, 0.08, 60.0, 85.0, 85.0, PickGraspSearchSampling{});
    };
  auto candidates = generate();
  const auto repeated = generate();
  ASSERT_EQ(candidates.size(), 48U);
  ASSERT_EQ(repeated.size(), candidates.size());
  std::array<std::size_t, 3> counts{};
  std::array<std::size_t, 24> orientations{};
  std::size_t zero_count = 0U;
  bool outer = false;
  bool inner = false;
  bool rotated_offset = false;
  for (std::size_t i = 0U; i < candidates.size(); ++i) {
    const auto & pose = candidates[i];
    ASSERT_TRUE(pose.contact);
    ++counts[pose.contact->closing_axis];
    ASSERT_LT(pose.orientation_candidate_index, orientations.size());
    ++orientations[pose.orientation_candidate_index];
    zero_count += pose.grasp_position_offset_object.norm() < 1e-12 ? 1U : 0U;
    outer |= pose.grasp_position_offset_object.cwiseAbs().maxCoeff() >= 0.015 - 1e-12;
    inner |= pose.grasp_position_offset_object.norm() > 1e-12 && pose.grasp_position_offset_object.norm() < 0.01;
    rotated_offset |= pose.axis_orientation_sample_index > 0U && pose.grasp_position_offset_object.norm() > 1e-12;
    EXPECT_EQ(pose.orientation_candidate_index, repeated[i].orientation_candidate_index);
    EXPECT_NEAR((pose.grasp_position_offset_object - repeated[i].grasp_position_offset_object).norm(), 0.0, 1e-12);
    for (std::size_t j = 0U; j < i; ++j) {
      EXPECT_FALSE(
        (pose.grasp_position_offset_object - candidates[j].grasp_position_offset_object).norm() < 1e-9 &&
        std::abs(pose.tcp_orientation.dot(candidates[j].tcp_orientation)) > 1.0 - 1e-9);
    }
  }
  for (const auto count : counts) {
    EXPECT_GE(count, 10U);
  }
  EXPECT_GE(zero_count, 15U);
  EXPECT_LT(zero_count, candidates.size() / 2U);
  for (const auto count : orientations) {
    EXPECT_LE(count, 4U);
  }
  EXPECT_TRUE(outer);
  EXPECT_TRUE(inner);
  EXPECT_TRUE(rotated_offset);

  order_pick_grasp_search_candidates(
    candidates, PickGraspPoseOrderPolicy::DISTANCE_THEN_DIVERSITY);
  PickSearchController controller(candidates.size(), 2U);
  for (std::size_t i = 0U; i < candidates.size(); ++i) {
    if (i > 0U) {
      EXPECT_GE(
        candidates[i].grasp_position_offset_object.norm() + 1e-12,
        candidates[i - 1U].grasp_position_offset_object.norm());
    }
    // This is the production evaluator boundary: controller indices select
    // the already ordered target; both existing IK seed attempts keep it.
    for (std::size_t seed = 0U; seed < 2U; ++seed) {
      ASSERT_EQ(controller.next(100.0), PickSearchStatus::SEARCHING);
      EXPECT_EQ(controller.pose_index(), i);
      EXPECT_EQ(
        controller.seed_source(), seed == 0U ?
        PickSearchSeed::MEASURED : PickSearchSeed::DIVERSIFIED);
      controller.evaluate(PickSearchEvaluation::GRASP_VALIDATION_FAILED);
    }
  }
  EXPECT_EQ(controller.next(100.0), PickSearchStatus::ALL_CANDIDATES_EXHAUSTED);
}

TEST(PickGraspSearch, EvaluationTiesCycleOrientationsThenTiltAndStableOrder)
{
  // Equal-distance poses with two face-local orientation sample indices. A second
  // position of the same orientation must wait for other orientations.
  std::vector<PickGraspPoseCandidate> poses(5U);
  for (auto & pose : poses) {
    pose.grasp_position_offset_object.x() = 0.01;
  }
  poses[0].orientation_candidate_index = 0U;
  poses[0].withdrawal_tilt_rad = 1.0;
  poses[1] = poses[0];
  poses[1].grasp_position_offset_object.x() = -0.01;
  poses[2].orientation_candidate_index = 1U;
  poses[2].withdrawal_tilt_rad = 0.5;
  poses[3].orientation_candidate_index = 2U;
  poses[3].axis_orientation_sample_index = 1U;
  poses[4].orientation_candidate_index = 3U;
  poses[4].withdrawal_tilt_rad = 0.5;
  order_pick_grasp_search_candidates(
    poses, PickGraspPoseOrderPolicy::DISTANCE_THEN_DIVERSITY);
  const std::array<std::size_t, 5> expected{1U, 3U, 0U, 2U, 0U};
  for (std::size_t i = 0U; i < poses.size(); ++i) {
    EXPECT_EQ(poses[i].orientation_candidate_index, expected[i]);
  }
}

TEST(PickGraspSearch, OrderingPoliciesChangeOnlyTheEvaluationOrder)
{
  std::vector<PickGraspPoseCandidate> selected(5U);
  selected[0].grasp_position_offset_object.x() = 0.01;
  selected[0].withdrawal_tilt_rad = kPi / 2.0;
  selected[0].orientation_candidate_index = 0U;
  selected[0].axis_orientation_sample_index = 0U;
  selected[1] = selected[0];
  selected[1].withdrawal_tilt_rad = 0.1;
  selected[1].orientation_candidate_index = 1U;
  selected[1].axis_orientation_sample_index = 1U;
  selected[2] = selected[0];
  selected[2].grasp_position_offset_object.x() = 0.02;
  selected[2].withdrawal_tilt_rad = 0.0;
  selected[2].orientation_candidate_index = 2U;
  selected[3] = selected[0];
  selected[3].grasp_position_offset_object.x() = 0.0;
  selected[3].grasp_position_offset_object.y() = 0.01;
  selected[3].withdrawal_tilt_rad = 0.5;
  selected[3].orientation_candidate_index = 3U;
  selected[3].axis_orientation_sample_index = 0U;
  selected[4] = selected[0];
  selected[4].grasp_position_offset_object.x() = -0.01;
  selected[4].withdrawal_tilt_rad = kPi / 2.0;
  selected[4].orientation_candidate_index = 0U;
  selected[4].generation_index = 4U;

  const auto same_candidate_set = [](auto lhs, auto rhs) {
      const auto by_generation = [](const auto & a, const auto & b) {
          return a.generation_index < b.generation_index;
        };
      std::stable_sort(lhs.begin(), lhs.end(), by_generation);
      std::stable_sort(rhs.begin(), rhs.end(), by_generation);
      if (lhs.size() != rhs.size()) {return false;}
      for (std::size_t i = 0U; i < lhs.size(); ++i) {
        if (lhs[i].generation_index != rhs[i].generation_index ||
          (lhs[i].grasp_position_offset_object - rhs[i].grasp_position_offset_object).norm() >
          1e-12)
        {return false;}
      }
      return true;
    };
  for (std::size_t i = 0U; i < selected.size(); ++i) {
    selected[i].generation_index = i;
  }
  auto diversity = selected;
  auto tilt = selected;
  order_pick_grasp_search_candidates(
    diversity, PickGraspPoseOrderPolicy::DISTANCE_THEN_DIVERSITY);
  order_pick_grasp_search_candidates(tilt, PickGraspPoseOrderPolicy::DISTANCE_THEN_TILT);
  EXPECT_TRUE(same_candidate_set(diversity, tilt));
  ASSERT_EQ(diversity.size(), 5U);
  ASSERT_EQ(tilt.size(), 5U);
  const std::array<std::size_t, 5> legacy_order{3U, 0U, 1U, 4U, 2U};
  const std::array<std::size_t, 5> tilt_order{1U, 3U, 0U, 4U, 2U};
  for (std::size_t i = 0U; i < selected.size(); ++i) {
    EXPECT_EQ(diversity[i].generation_index, legacy_order[i]);
    EXPECT_EQ(tilt[i].generation_index, tilt_order[i]);
  }
  // Legacy ordering keeps orientation sample index ahead of Tilt.
  EXPECT_EQ(diversity[0].orientation_candidate_index, 3U);
  EXPECT_EQ(diversity[1].orientation_candidate_index, 0U);
  EXPECT_EQ(diversity[2].orientation_candidate_index, 1U);
  // Tilt ordering prefers the lower tilt at equal distance, including the 90-degree
  // candidate's continued presence when it is not first.
  EXPECT_EQ(tilt[0].withdrawal_tilt_rad, 0.1);
  EXPECT_EQ(tilt[1].withdrawal_tilt_rad, 0.5);
  EXPECT_TRUE(
    std::any_of(
      tilt.begin(), tilt.end(), [](const auto & pose) {
        return std::abs(pose.withdrawal_tilt_rad - kPi / 2.0) < 1e-12;
      }));
  EXPECT_EQ(tilt.back().grasp_position_offset_object.norm(), 0.02);
}

TEST(PickGraspSearch, HorizontalNinetyDegreePoseRemainsInSelectedSet)
{
  const Eigen::Quaterniond reference(Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()));
  const auto candidates = generate_pick_grasp_search_candidates(
    Eigen::Quaterniond::Identity(), reference, Eigen::Vector3d::Zero(),
    Eigen::Vector3d(0.5, 0.0, 0.0), Eigen::Vector3d::UnitZ(),
    0.07, 0.06, 0.08, 60.0, 85.0, 85.0,
    PickGraspSearchSampling{90.0, 0.0, 1U, 8U, 48U});
  ASSERT_FALSE(candidates.empty());
  const auto horizontal = std::find_if(
    candidates.begin(), candidates.end(), [](const auto & pose) {
      return std::abs(pose.withdrawal_tilt_rad - kPi / 2.0) < 1e-9;
    });
  ASSERT_NE(horizontal, candidates.end());
  auto distance_diversity = candidates;
  auto distance_tilt = candidates;
  order_pick_grasp_search_candidates(
    distance_diversity, PickGraspPoseOrderPolicy::DISTANCE_THEN_DIVERSITY);
  order_pick_grasp_search_candidates(
    distance_tilt, PickGraspPoseOrderPolicy::DISTANCE_THEN_TILT);
  EXPECT_EQ(distance_diversity.size(), candidates.size());
  EXPECT_EQ(distance_tilt.size(), candidates.size());
  EXPECT_TRUE(
    std::any_of(
      distance_tilt.begin(), distance_tilt.end(), [](const auto & pose) {
        return std::abs(pose.withdrawal_tilt_rad - kPi / 2.0) < 1e-9;
      }));
}

TEST(PickGraspSearch, OrderPolicyNamesAreExplicitAndRejectTypos)
{
  EXPECT_EQ(
    parse_pick_grasp_pose_order_policy("distance_then_tilt"),
    PickGraspPoseOrderPolicy::DISTANCE_THEN_TILT);
  EXPECT_EQ(
    parse_pick_grasp_pose_order_policy("distance_then_diversity"),
    PickGraspPoseOrderPolicy::DISTANCE_THEN_DIVERSITY);
  EXPECT_THROW(parse_pick_grasp_pose_order_policy("tilt_then_distance"), std::invalid_argument);
}

TEST(PickGraspSearch, ReferenceTranslationAndCapPreserveEligibleFacePoses)
{
  const Eigen::Quaterniond reference_grasp_tcp_orientation(Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()));
  const Eigen::Vector3d reference_grasp_position_object(0.004, -0.003, 0.002);
  for (const auto cap : {0U, 1U, 3U, 48U}) {
    const auto poses = generate_pick_grasp_search_candidates(
      Eigen::Quaterniond::Identity(), reference_grasp_tcp_orientation, reference_grasp_position_object,
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(),
      0.07, 0.06, 0.08, 60.0, 85.0, 85.0,
      PickGraspSearchSampling{90.0, 0.5, 3U, 8U, cap});
    EXPECT_LE(poses.size(), cap);
    if (cap >= 3U) {
      for (std::size_t axis = 0U; axis < 3U; ++axis) {
        EXPECT_TRUE(
          std::any_of(
            poses.begin(), poses.end(), [&](const auto & pose) {
              return pose.contact->closing_axis == axis && pose.axis_orientation_sample_index == 0U &&
              pose.grasp_position_offset_object.norm() < 1e-12;
            }));
      }
    }
    for (const auto & pose : poses) {
      EXPECT_NEAR(pose.grasp_position_offset_object[pose.contact->closing_axis], 0.0, 1e-12);
    }
  }
}

TEST(PickGraspSearch, FiniteLatticeDeduplicatesClampedOffsetsAndQuaternionSigns)
{
  const Eigen::Quaterniond reference_grasp_tcp_orientation(Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()));
  const auto generate = [&](Eigen::Quaterniond rotation) {
      return generate_pick_grasp_search_candidates(
        Eigen::Quaterniond::Identity(), rotation, Eigen::Vector3d(0.004, -0.003, 0.002),
        Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(),
        0.07, 0.06, 0.08, 60.0, 85.0, 85.0,
        PickGraspSearchSampling{90.0, 0.5, 3U, 8U, 1000U});
    };
  const auto poses = generate(reference_grasp_tcp_orientation);
  Eigen::Quaterniond opposite = reference_grasp_tcp_orientation;
  opposite.coeffs() *= -1.0;
  const auto equivalent = generate(opposite);
  ASSERT_GT(poses.size(), 48U);
  ASSERT_LT(poses.size(), 1000U);
  ASSERT_EQ(poses.size(), equivalent.size());
  const std::array<double, 3> lengths{0.07, 0.06, 0.08};
  for (std::size_t i = 0U; i < poses.size(); ++i) {
    EXPECT_NEAR(std::abs(poses[i].tcp_orientation.dot(equivalent[i].tcp_orientation)), 1.0, 1e-12);
    EXPECT_NEAR((poses[i].grasp_position_offset_object - equivalent[i].grasp_position_offset_object).norm(), 0.0, 1e-12);
    const Eigen::Vector3d point = Eigen::Vector3d(0.004, -0.003, 0.002) + poses[i].grasp_position_offset_object;
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
      if (axis == poses[i].contact->closing_axis || poses[i].grasp_position_offset_object.norm() < 1e-12) {continue;}
      EXPECT_LT(std::abs(point[axis]), lengths[axis] * 0.5);
    }
    for (std::size_t j = 0U; j < i; ++j) {
      EXPECT_FALSE((poses[i].grasp_position_offset_object - poses[j].grasp_position_offset_object).norm() < 1e-9 &&
        std::abs(poses[i].tcp_orientation.dot(poses[j].tcp_orientation)) > 1.0 - 1e-9);
    }
  }
}

TEST(PickGraspSearch, ExhaustedVerticalPoseCanAdvanceToAnotherDirection)
{
  const Eigen::Quaterniond reference_grasp_tcp_orientation(Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()));
  const auto candidates = generate_pick_grasp_search_candidates(
    Eigen::Quaterniond::Identity(), reference_grasp_tcp_orientation, Eigen::Vector3d::Zero(),
    Eigen::Vector3d(0.5, 0.0, 0.0), Eigen::Vector3d::UnitZ(),
    0.07, 0.06, 0.08, 60.0, 85.0, 85.0,
    PickGraspSearchSampling{90.0, 0.0, 1U, 8U, 48U});
  auto ordered_candidates = candidates;
  order_pick_grasp_search_candidates(
    ordered_candidates, PickGraspPoseOrderPolicy::DISTANCE_THEN_TILT);
  std::vector<std::size_t> indices;
  for (std::size_t i = 0; i < ordered_candidates.size(); ++i) {
    indices.push_back(i);
  }
  std::size_t attempts = 0U;
  bool saw_vertical = false;
  bool saw_tilted = false;
  const auto selected = first_successful_candidate(
    indices, [&](std::size_t i) {
      ++attempts;
      const double tilt = ordered_candidates[i].withdrawal_tilt_rad;
      saw_vertical |= std::abs(tilt) < 1e-9;
      saw_tilted |= tilt > 1e-9 && tilt < kPi / 2.0 - 1e-9;
      return std::abs(tilt - kPi / 2.0) < 1e-9;
    });
  ASSERT_TRUE(selected.has_value());
  EXPECT_TRUE(saw_vertical);
  EXPECT_TRUE(saw_tilted);
  EXPECT_GT(attempts, 2U);
  EXPECT_EQ(attempts, *selected + 1U);
  EXPECT_NEAR(ordered_candidates[*selected].withdrawal_tilt_rad, kPi / 2.0, 1e-9);
}

}  // namespace
}  // namespace arm_cell_motion
