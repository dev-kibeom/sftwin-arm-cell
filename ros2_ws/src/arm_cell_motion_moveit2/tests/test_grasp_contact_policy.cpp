#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>

#include "arm_cell_motion_moveit2/grasp_contact_policy.hpp"

namespace arm_cell_motion_moveit2
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
}  // namespace arm_cell_motion_moveit2
