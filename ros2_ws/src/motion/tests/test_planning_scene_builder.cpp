#include <gtest/gtest.h>

#include "arm_cell_motion/planning_scene_builder.hpp"

namespace arm_cell_motion
{
namespace
{
using Primitive = arm_cell_interfaces::msg::CollisionPrimitive;

Primitive occupancy_box()
{
  Primitive value;
  value.occupancy_class = Primitive::OBSERVED_OCCUPANCY;
  value.shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  value.shape.dimensions = {0.1, 0.2, 0.3};
  value.pose.orientation.w = 1.0;
  return value;
}

TEST(PlanningSceneBuilder, VisionAndTrayPolicyAreIndependentRequestInputs)
{
  moveit_msgs::msg::PlanningScene snapshot;
  snapshot.world.collision_objects.resize(1U);
  snapshot.world.collision_objects.front().id = "static_fixture";
  snapshot.world.collision_objects.push_back(moveit_msgs::msg::CollisionObject{});
  snapshot.world.collision_objects.back().id = "vision_candidate_occupancy_previous";
  PlanningSceneBuilder builder(snapshot, {occupancy_box()}, std_msgs::msg::Header{});

  PlanningScenePolicy excluded;
  const auto baseline = builder.build(excluded);
  ASSERT_TRUE(baseline.valid);
  EXPECT_EQ(baseline.diff.world.collision_objects.size(), 1U);
  EXPECT_EQ(baseline.diff.world.collision_objects.front().operation,
    moveit_msgs::msg::CollisionObject::REMOVE);
  EXPECT_TRUE(baseline.candidate_ids.empty());

  PlanningScenePolicy included;
  included.vision_occupancy = VisionOccupancyPolicy::INCLUDE;
  const auto with_vision = builder.build(included);
  ASSERT_TRUE(with_vision.valid);
  ASSERT_EQ(with_vision.diff.world.collision_objects.size(), 2U);
  EXPECT_EQ(with_vision.diff.world.collision_objects.front().operation,
    moveit_msgs::msg::CollisionObject::REMOVE);
  EXPECT_EQ(with_vision.diff.world.collision_objects.back().operation,
    moveit_msgs::msg::CollisionObject::ADD);
  EXPECT_EQ(with_vision.candidate_ids.size(), 1U);

  const auto still_excluded = builder.build(excluded);
  ASSERT_TRUE(still_excluded.valid);
  EXPECT_EQ(still_excluded.diff.world.collision_objects.size(), 1U);
  EXPECT_EQ(builder.snapshot().world.collision_objects.front().id, "static_fixture");
}

TEST(PlanningSceneBuilder, RetractUsesSelectedContactsAndRequiresSelectedObject)
{
  moveit_msgs::msg::PlanningScene snapshot;
  snapshot.world.collision_objects.resize(1U);
  snapshot.world.collision_objects.front().id = "selected_support";
  PlanningSceneBuilder builder(snapshot, {}, std_msgs::msg::Header{},
    "selected_support", {"tool_contact_a", "tool_contact_b"});
  PlanningScenePolicy policy;
  policy.apply_pick_retract_tray_exception = true;
  const auto result = builder.build(policy);
  ASSERT_TRUE(result.valid);
  const auto & acm = result.diff.allowed_collision_matrix;
  const auto index = [&acm](const std::string & name) {
      return std::distance(
        acm.entry_names.begin(),
        std::find(acm.entry_names.begin(), acm.entry_names.end(), name));
    };
  const auto support = index("selected_support");
  const auto first = index("tool_contact_a");
  const auto second = index("tool_contact_b");
  ASSERT_LT(support, acm.entry_names.size());
  ASSERT_LT(first, acm.entry_names.size());
  ASSERT_LT(second, acm.entry_names.size());
  EXPECT_TRUE(acm.entry_values[support].enabled[first]);
  EXPECT_TRUE(acm.entry_values[support].enabled[second]);
  EXPECT_FALSE(acm.entry_values[first].enabled[second]);
  EXPECT_TRUE(snapshot.allowed_collision_matrix.entry_names.empty());
  EXPECT_TRUE(builder.build(PlanningScenePolicy{}).diff.allowed_collision_matrix.entry_names.empty());

  PlanningSceneBuilder missing(snapshot, {}, std_msgs::msg::Header{},
    "other_support", {"tool_contact_a", "tool_contact_b"});
  EXPECT_FALSE(missing.build(policy).valid);
}

TEST(PlanningSceneBuilder, DuplicateCandidateIdentityFailsClosed)
{
  moveit_msgs::msg::PlanningScene snapshot;
  snapshot.world.collision_objects.resize(2U);
  for (auto & object : snapshot.world.collision_objects) {
    object.id = "vision_candidate_occupancy_duplicate";
  }
  PlanningSceneBuilder builder(snapshot, {}, std_msgs::msg::Header{});
  const auto result = builder.build(PlanningScenePolicy{});
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.failure_reason, "VISION_OCCUPANCY_IDENTITY_AMBIGUOUS");
}
}  // namespace
}  // namespace arm_cell_motion
