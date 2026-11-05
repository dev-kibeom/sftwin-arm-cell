#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <future>
#include <string>
#include <vector>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>

#include "arm_cell_motion/grasp_contact_policy.hpp"
#include "arm_cell_motion/pick_retract_collision_policy.hpp"
#include "arm_cell_motion/pick_retract_action_wait.hpp"

namespace arm_cell_motion
{
namespace
{
using moveit_msgs::msg::CollisionObject;

CollisionObject object(const std::string & id)
{
  CollisionObject result;
  result.id = id;
  return result;
}

const std::vector<std::string> kLinks{
  "gripper_robotiq_85_left_finger_tip_link",
  "gripper_robotiq_85_right_finger_tip_link"};

CollisionObject fixture_box()
{
  auto result = object("wu14_fixture_current");
  result.header.frame_id = "base_link";
  result.pose.orientation.w = 1.0;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {0.07, 0.06, 0.08};
  result.primitives = {box};
  geometry_msgs::msg::Pose local_pose;
  local_pose.orientation.w = 1.0;
  result.primitive_poses = {local_pose};
  return result;
}

TEST(GraspContactPolicyTest, FixtureWidthUsesSelectedClosingAxisAndShapeRotation)
{
  auto fixture = fixture_box();
  geometry_msgs::msg::PoseStamped tcp;
  tcp.header.frame_id = "base_link";
  tcp.pose.orientation.w = 1.0;
  EXPECT_DOUBLE_EQ(*fixture_contact_width_mm({fixture}, fixture.id, tcp, 50, 80, 85), 70);
  // Side-on TCP: +X closes along the fixture Z axis, without a top-down gate.
  tcp.pose.orientation.y = std::sqrt(0.5);
  tcp.pose.orientation.w = std::sqrt(0.5);
  EXPECT_DOUBLE_EQ(*fixture_contact_width_mm({fixture}, fixture.id, tcp, 50, 80, 85), 80);
  // Compose the CollisionObject and primitive rotations, not just the object pose.
  fixture.primitive_poses[0].orientation = tcp.pose.orientation;
  EXPECT_DOUBLE_EQ(*fixture_contact_width_mm({fixture}, fixture.id, tcp, 50, 80, 85), 70);
  fixture.pose.orientation = tcp.pose.orientation;
  EXPECT_DOUBLE_EQ(*fixture_contact_width_mm({fixture}, fixture.id, tcp, 50, 80, 85), 80);
}

TEST(GraspContactPolicyTest, FixtureWidthRejectsMissingAmbiguousInvalidOrOutOfPolicyGeometry)
{
  auto fixture = fixture_box();
  geometry_msgs::msg::PoseStamped tcp;
  tcp.header.frame_id = "base_link";
  tcp.pose.orientation.w = 1.0;
  const auto width = [&](const std::vector<CollisionObject> & objects) {
      return fixture_contact_width_mm(objects, fixture.id, tcp, 69, 71, 85);
    };
  EXPECT_FALSE(width({}));
  EXPECT_FALSE(width({fixture, fixture}));
  auto invalid = fixture;
  invalid.header.frame_id = "world";
  EXPECT_FALSE(width({invalid}));
  invalid = fixture;
  invalid.primitives[0].dimensions[2] = 0.0;
  EXPECT_FALSE(width({invalid}));
  invalid = fixture;
  invalid.primitive_poses[0].orientation.w = 0.0;
  EXPECT_FALSE(width({invalid}));
  invalid = fixture;
  invalid.primitives[0].dimensions[0] = 0.09;
  EXPECT_FALSE(width({invalid}));
  tcp.pose.orientation.z = std::sin(0.2);
  tcp.pose.orientation.w = std::cos(0.2);
  EXPECT_FALSE(width({fixture}));
}

TEST(GraspContactPolicyTest, PlanningSceneRequestIncludesWorldObjectNames)
{
  const auto components = grasp_contact_policy_planning_scene_components();

  EXPECT_NE(
    components & moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX, 0U);
  EXPECT_NE(components & moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES, 0U);
}

TEST(GraspContactPolicyTest, CurrentTargetAcceptedWithUnrelatedFixture)
{
  const auto result = resolve_current_target(
    {object("wu14_fixture_run_cube_profile"), object("wu14_fixture_run_other_fixture")},
    "wu14_fixture_", "wu14_fixture_run_cube_profile", kLinks);

  EXPECT_EQ(result.reason, GraspContactPolicyReason::ACCEPTED);
  EXPECT_EQ(result.resolved_collision_object_id, "wu14_fixture_run_cube_profile");
  EXPECT_EQ(result.prefix_matches.size(), 2U);
}

TEST(GraspContactPolicyTest, MultipleFixturesDoNotRejectValidCurrentTarget)
{
  const auto result = resolve_current_target(
    {object("wu14_fixture_a_cube_profile"), object("wu14_fixture_b_other")},
    "wu14_fixture_", "wu14_fixture_a_cube_profile", kLinks);

  EXPECT_EQ(result.reason, GraspContactPolicyReason::ACCEPTED);
}

TEST(GraspContactPolicyTest, MissingTargetRejectsClosed)
{
  const auto result = resolve_current_target(
    {object("wu14_fixture_run_other_fixture")}, "wu14_fixture_",
    "wu14_fixture_run_cube_profile", kLinks);

  EXPECT_EQ(result.reason, GraspContactPolicyReason::TARGET_NOT_FOUND);
}

TEST(GraspContactPolicyTest, AmbiguousTargetRejectsClosed)
{
  const auto result = resolve_current_target(
    {object("wu14_fixture_a_cube_profile"), object("wu14_fixture_a_cube_profile")},
    "wu14_fixture_", "wu14_fixture_a_cube_profile", kLinks);

  EXPECT_EQ(result.reason, GraspContactPolicyReason::TARGET_AMBIGUOUS);
}

TEST(GraspContactPolicyTest, InvalidConfigurationRejectsClosed)
{
  const auto result = resolve_current_target(
    {object("wu14_fixture_run_cube_profile")}, "",
    "wu14_fixture_run_cube_profile", kLinks);

  EXPECT_EQ(result.reason, GraspContactPolicyReason::INVALID_CONFIG);
}

TEST(GraspContactPolicyTest, ExactIdentityDoesNotUseTargetIdSuffix)
{
  const auto result = resolve_current_target(
    {object("wu14_fixture_run_cube_profile"), object("wu14_fixture_new_cube_profile")},
    "wu14_fixture_", "wu14_fixture_new_cube_profile", kLinks);

  EXPECT_EQ(result.reason, GraspContactPolicyReason::ACCEPTED);
  EXPECT_EQ(result.resolved_collision_object_id, "wu14_fixture_new_cube_profile");
}

TEST(GraspContactPolicyTest, PlanningSceneServiceFailureRejectsClosed)
{
  EXPECT_EQ(
    planning_scene_failure_reason(false, false, false),
    GraspContactPolicyReason::SCENE_SERVICE_UNAVAILABLE);
}

TEST(GraspContactPolicyTest, PlanningSceneReadFailureRejectsClosed)
{
  EXPECT_EQ(
    planning_scene_failure_reason(true, false, false),
    GraspContactPolicyReason::SCENE_READ_TIMEOUT);
}

TEST(GraspContactPolicyTest, PlanningSceneApplyFailureRejectsClosed)
{
  EXPECT_EQ(
    planning_scene_failure_reason(true, true, false),
    GraspContactPolicyReason::ACM_APPLY_FAILED);
}

TEST(GraspContactPolicyTest, ACMAllowsOnlyCurrentTargetAndAuthorizedLinks)
{
  moveit_msgs::msg::AllowedCollisionMatrix original;
  original.entry_names = {"robot_link", "wu14_fixture_run_other"};
  moveit_msgs::msg::AllowedCollisionEntry row;
  row.enabled = {false, true};
  original.entry_values = {row, row};

  const auto result = allow_current_target_contacts(
    original, "wu14_fixture_run_cube_profile", kLinks);

  ASSERT_TRUE(result.has_value());
  const auto & names = result->entry_names;
  const auto target = static_cast<std::size_t>(
    std::distance(
      names.begin(), std::find(
        names.begin(), names.end(),
        "wu14_fixture_run_cube_profile")));
  for (const auto & link : kLinks) {
    const auto index = static_cast<std::size_t>(
      std::distance(names.begin(), std::find(names.begin(), names.end(), link)));
    EXPECT_TRUE(result->entry_values[target].enabled[index]);
  }
  const auto unrelated = static_cast<std::size_t>(
    std::distance(
      names.begin(), std::find(
        names.begin(), names.end(),
        "wu14_fixture_run_other")));
  EXPECT_FALSE(result->entry_values[target].enabled[unrelated]);
}

TEST(PickRetractCollisionPolicyTest, AllowsOnlyTheTwoFingerTipTrayPairsAndPreservesOtherEntries)
{
  moveit_msgs::msg::AllowedCollisionMatrix original;
  original.entry_names = {
    "robot_link", kPickRetractTrayCollisionObject,
    kPickRetractTrayAllowedLinks[0], kPickRetractTrayAllowedLinks[1], "other_world_object"};
  original.entry_values.resize(original.entry_names.size());
  for (auto & row : original.entry_values) {
    row.enabled.assign(original.entry_names.size(), false);
  }
  original.entry_values[0].enabled[4] = true;
  original.entry_values[4].enabled[0] = true;
  original.entry_values[1].enabled[0] = true;
  original.entry_values[0].enabled[1] = true;

  const auto allowed = make_pick_retract_allowed_collision_matrix(original);
  ASSERT_TRUE(allowed.has_value());
  const auto index = [&allowed](const std::string & name) {
      return static_cast<std::size_t>(std::distance(
        allowed->entry_names.begin(),
        std::find(allowed->entry_names.begin(), allowed->entry_names.end(), name)));
    };
  const auto tray = index(kPickRetractTrayCollisionObject);
  const auto left = index(kPickRetractTrayAllowedLinks[0]);
  const auto right = index(kPickRetractTrayAllowedLinks[1]);
  const auto robot = index("robot_link");
  const auto other = index("other_world_object");
  EXPECT_TRUE(allowed->entry_values[tray].enabled[left]);
  EXPECT_TRUE(allowed->entry_values[left].enabled[tray]);
  EXPECT_TRUE(allowed->entry_values[tray].enabled[right]);
  EXPECT_TRUE(allowed->entry_values[right].enabled[tray]);
  EXPECT_TRUE(allowed->entry_values[tray].enabled[robot]);
  EXPECT_TRUE(allowed->entry_values[robot].enabled[tray]);
  EXPECT_FALSE(allowed->entry_values[tray].enabled[other]);
  EXPECT_FALSE(allowed->entry_values[left].enabled[other]);
  EXPECT_TRUE(allowed->entry_values[robot].enabled[other]);
}

TEST(PickRetractCollisionPolicyTest, BuildsOnlyARequestScopedPlanningSceneDiff)
{
  const moveit_msgs::msg::AllowedCollisionMatrix original;
  const auto diff = make_pick_retract_planning_scene_diff(original);
  EXPECT_TRUE(diff.is_diff);
  EXPECT_TRUE(diff.robot_state.is_diff);
  EXPECT_FALSE(diff.world.collision_objects.size());
  EXPECT_EQ(diff.allowed_collision_matrix.entry_names.size(), 3U);
  const auto tray = static_cast<std::size_t>(std::distance(
      diff.allowed_collision_matrix.entry_names.begin(),
      std::find(
        diff.allowed_collision_matrix.entry_names.begin(),
        diff.allowed_collision_matrix.entry_names.end(), kPickRetractTrayCollisionObject)));
  for (const auto * link : kPickRetractTrayAllowedLinks) {
    const auto index = static_cast<std::size_t>(std::distance(
        diff.allowed_collision_matrix.entry_names.begin(),
        std::find(
          diff.allowed_collision_matrix.entry_names.begin(),
          diff.allowed_collision_matrix.entry_names.end(), link)));
    EXPECT_TRUE(diff.allowed_collision_matrix.entry_values[tray].enabled[index]);
    EXPECT_TRUE(diff.allowed_collision_matrix.entry_values[index].enabled[tray]);
  }
}

TEST(PickRetractCollisionPolicyTest, ExceptionScopeIsOnlyPickRetract)
{
  EXPECT_TRUE(is_pick_retract_collision_exception_scope(true, true));
  EXPECT_FALSE(is_pick_retract_collision_exception_scope(true, false));
  EXPECT_FALSE(is_pick_retract_collision_exception_scope(false, true));
  EXPECT_FALSE(is_pick_retract_collision_exception_scope(false, false));
}

TEST(PickRetractActionWaitTest, ReturnsWhenMoveGroupResultArrives)
{
  const auto outcome = wait_for_pick_retract_action_result(
    [](std::chrono::milliseconds) {return std::future_status::ready;},
    std::chrono::milliseconds(100), []() {return false;}, []() {},
    std::chrono::milliseconds(20));

  EXPECT_EQ(outcome.termination, PickRetractActionWaitTermination::RESULT_READY);
  EXPECT_TRUE(outcome.result_ready);
}

TEST(PickRetractActionWaitTest, CancelsAndReturnsWhenPlanningBudgetExpiresWithoutResult)
{
  bool cancel_requested = false;
  const auto start = std::chrono::steady_clock::now();
  const auto outcome = wait_for_pick_retract_action_result(
    [](std::chrono::milliseconds) {return std::future_status::timeout;},
    std::chrono::milliseconds(20), []() {return false;},
    [&]() {cancel_requested = true;}, std::chrono::milliseconds(20));

  EXPECT_TRUE(cancel_requested);
  EXPECT_EQ(outcome.termination, PickRetractActionWaitTermination::PLANNING_BUDGET_EXPIRED);
  EXPECT_FALSE(outcome.result_ready);
  EXPECT_LT(
    std::chrono::steady_clock::now() - start, std::chrono::milliseconds(250));
}

TEST(PickRetractActionWaitTest, CancelsAndReturnsWhenMotionIsInterruptedWithoutResult)
{
  bool cancel_requested = false;
  const auto outcome = wait_for_pick_retract_action_result(
    [](std::chrono::milliseconds) {return std::future_status::timeout;},
    std::chrono::milliseconds(100), []() {return true;},
    [&]() {cancel_requested = true;}, std::chrono::milliseconds(20));

  EXPECT_TRUE(cancel_requested);
  EXPECT_EQ(outcome.termination, PickRetractActionWaitTermination::MOTION_INTERRUPTED);
  EXPECT_FALSE(outcome.result_ready);
}

TEST(GraspContactPolicyTest, ContactClassificationSeparatesObservationFromPolicy)
{
  EXPECT_EQ(
    classify_grasp_contact(
      "wu14_fixture_run_cube_profile", 2,
      kLinks.front(), 1, "wu14_fixture_run_cube_profile", kLinks),
    GraspContactClass::EXPECTED_TARGET_FINGERTIP);
  EXPECT_EQ(
    classify_grasp_contact(
      "wu14_fixture_run_cube_profile", 2,
      "forearm_link", 1, "wu14_fixture_run_cube_profile", kLinks),
    GraspContactClass::TARGET_UNAUTHORIZED_ROBOT_LINK);
  EXPECT_EQ(
    classify_grasp_contact(
      "forearm_link", 1, "wrist_link", 1,
      "wu14_fixture_run_cube_profile", kLinks),
    GraspContactClass::ROBOT_SELF_CONTACT);
  EXPECT_EQ(
    classify_grasp_contact(
      "table", 2, "forearm_link", 1,
      "wu14_fixture_run_cube_profile", kLinks),
    GraspContactClass::UNRELATED_WORLD_OBJECT);
  EXPECT_EQ(
    classify_grasp_contact(
      "wu14_fixture_run_other", 2, "forearm_link", 1,
      "wu14_fixture_run_cube_profile", kLinks),
    GraspContactClass::UNRELATED_FIXTURE);
}
}  // namespace
}  // namespace arm_cell_motion
