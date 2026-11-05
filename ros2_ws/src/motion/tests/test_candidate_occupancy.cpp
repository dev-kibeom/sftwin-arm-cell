#include <gtest/gtest.h>
#include <arm_cell_interfaces/msg/pick_candidate.hpp>
#include "arm_cell_motion/candidate_occupancy.hpp"

namespace arm_cell_motion
{
TEST(CandidateOccupancy, AvailabilityShapeAndFrameValidation)
{
  geometry_msgs::msg::PoseStamped pose;pose.header.frame_id = "base_link";
  arm_cell_interfaces::msg::CollisionPrimitive box;
  box.shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.shape.dimensions = {0.1, 0.2, 0.3};box.pose.orientation.w = 1;
  EXPECT_TRUE(valid_candidate_occupancy(false, {}, pose));
  EXPECT_TRUE(valid_candidate_occupancy(true, {}, pose));
  EXPECT_TRUE(valid_candidate_occupancy(true, {box}, pose));
  EXPECT_FALSE(valid_candidate_occupancy(false, {box}, pose));
  auto invalid = box;invalid.occupancy_class = 255;
  EXPECT_FALSE(valid_candidate_occupancy(true, {invalid}, pose));
  invalid = box;invalid.shape.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  EXPECT_FALSE(valid_candidate_occupancy(true, {invalid}, pose));
  invalid = box;invalid.shape.dimensions.pop_back();
  EXPECT_FALSE(valid_candidate_occupancy(true, {invalid}, pose));
  invalid = box;invalid.pose.orientation.w = 0;
  EXPECT_FALSE(valid_candidate_occupancy(true, {invalid}, pose));
  pose.header.frame_id = "camera";
  EXPECT_FALSE(valid_candidate_occupancy(true, {box}, pose));
}
TEST(CandidateOccupancy, ReplacementAndEmptyClearAffectOnlyTemporaryIds)
{
  arm_cell_interfaces::msg::CollisionPrimitive box;
  box.shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.shape.dimensions = {0.1, 0.2, 0.3};box.pose.orientation.w = 1;
  std_msgs::msg::Header header;header.frame_id = "base_link";header.stamp.sec = 42;
  std::vector<std::string> ids;
  auto first = candidate_scene_diff({}, {box, box}, header, ids);
  ASSERT_EQ(ids.size(), 2U);
  ASSERT_EQ(first.world.collision_objects.size(), 2U);
  std::vector<std::string> next;
  auto replace = candidate_scene_diff(ids, {box}, header, next);
  ASSERT_EQ(replace.world.collision_objects.size(), 3U);
  EXPECT_EQ(
    replace.world.collision_objects[0].operation,
    moveit_msgs::msg::CollisionObject::REMOVE);
  EXPECT_EQ(
    replace.world.collision_objects[1].operation,
    moveit_msgs::msg::CollisionObject::REMOVE);
  EXPECT_EQ(replace.world.collision_objects[2].operation, moveit_msgs::msg::CollisionObject::ADD);
  EXPECT_EQ(replace.world.collision_objects[2].header, header);
  auto clear = candidate_scene_diff(next, {}, header, ids);
  EXPECT_TRUE(ids.empty());
  ASSERT_EQ(clear.world.collision_objects.size(), 1U);
  EXPECT_TRUE(clear.allowed_collision_matrix.entry_names.empty());
  EXPECT_TRUE(clear.robot_state.is_diff);
}

TEST(CandidateOccupancy, VisionCandidatesProduceBoxesExceptSelectedCandidate)
{
  using Candidate = arm_cell_interfaces::msg::PickCandidate;
  Candidate selected;
  selected.acquisition_id = "acq-1";
  selected.candidate_id = "selected";
  selected.target_pose.header.frame_id = "base_link";
  selected.target_pose.pose.orientation.w = 1.0;
  selected.has_observed_rectangle = true;
  selected.has_estimated_height = true;
  selected.rectangle_long_side_m = 0.20F;
  selected.rectangle_short_side_m = 0.10F;
  selected.estimated_height_m = 0.08F;

  Candidate other = selected;
  other.candidate_id = "other";
  other.target_pose.pose.position.x = 0.3;
  other.target_pose.pose.position.z = 0.5;
  Candidate foreign = other;
  foreign.acquisition_id = "acq-2";
  foreign.candidate_id = "foreign";
  Candidate estimated_geometry = other;
  estimated_geometry.candidate_id = "estimated-geometry";
  estimated_geometry.has_observed_rectangle = false;
  EXPECT_FALSE(estimated_geometry.has_observed_rectangle);
  estimated_geometry.target_pose.pose.position.x = 0.6;
  estimated_geometry.target_pose.pose.position.z = 0.7;

  const auto boxes = candidate_occupancy_from_candidates(
    {selected, other, foreign, estimated_geometry}, "acq-1", "selected");
  ASSERT_EQ(boxes.size(), 2U);
  EXPECT_EQ(boxes.front().shape.type, shape_msgs::msg::SolidPrimitive::BOX);
  EXPECT_EQ(boxes.front().occupancy_class,
    arm_cell_interfaces::msg::CollisionPrimitive::OBSERVED_OCCUPANCY);
  EXPECT_FLOAT_EQ(boxes.front().shape.dimensions.at(0), 0.20F);
  EXPECT_FLOAT_EQ(boxes.front().shape.dimensions.at(1), 0.10F);
  EXPECT_FLOAT_EQ(boxes.front().shape.dimensions.at(2), 0.08F);
  EXPECT_DOUBLE_EQ(boxes.front().pose.position.x, 0.3);
  EXPECT_NEAR(boxes.front().pose.position.z, 0.46, 1e-8);
  EXPECT_DOUBLE_EQ(boxes.front().pose.orientation.w, 1.0);
  EXPECT_EQ(boxes[1].shape.type, shape_msgs::msg::SolidPrimitive::BOX);
  EXPECT_FLOAT_EQ(boxes[1].shape.dimensions.at(0), 0.20F);
  EXPECT_FLOAT_EQ(boxes[1].shape.dimensions.at(1), 0.10F);
  EXPECT_FLOAT_EQ(boxes[1].shape.dimensions.at(2), 0.08F);
  EXPECT_DOUBLE_EQ(boxes[1].pose.position.x, 0.6);
  EXPECT_NEAR(boxes[1].pose.position.z, 0.66, 1e-8);
  std_msgs::msg::Header header = other.target_pose.header;
  std::vector<std::string> ids;
  const auto scene = candidate_scene_diff({}, boxes, header, ids);
  ASSERT_EQ(scene.world.collision_objects.size(), 2U);
  ASSERT_EQ(scene.world.collision_objects.front().primitives.size(), 1U);
  EXPECT_EQ(ids, (std::vector<std::string>{
    "vision_candidate_occupancy_0", "vision_candidate_occupancy_1"}));
  EXPECT_EQ(scene.world.collision_objects.front().primitives.front().dimensions,
    boxes.front().shape.dimensions);
  EXPECT_EQ(scene.world.collision_objects.front().primitive_poses.front(), boxes.front().pose);
  EXPECT_EQ(scene.world.collision_objects[1].primitives.front().dimensions,
    boxes[1].shape.dimensions);
  EXPECT_EQ(scene.world.collision_objects[1].primitive_poses.front(), boxes[1].pose);
  EXPECT_TRUE(candidate_occupancy_from_candidates(
      {selected}, "acq-1", "selected").empty());
}
}  // namespace arm_cell_motion

TEST(CandidateOccupancy, ConvexTopUsesMeshWithObservedVertices)
{
  using Primitive = arm_cell_interfaces::msg::CollisionPrimitive;
  Primitive p;
  p.shape_type = Primitive::CONVEX_TOP;
  p.pose.orientation.w = 1;
  p.bottom_height_m = 0;
  for (const auto & v : std::vector<std::array<double, 3>>{
      {0, 0, 0.1}, {0.2, 0, 0.2}, {0.2, 0.1, 0.2}, {0, 0.1, 0.1}}) {
    geometry_msgs::msg::Point point;
    point.x = v[0]; point.y = v[1]; point.z = v[2];
    p.top_vertices.push_back(point);
  }
  geometry_msgs::msg::PoseStamped observation;
  observation.header.frame_id = "base_link";
  ASSERT_TRUE(arm_cell_motion::valid_candidate_occupancy(true, {p}, observation));
  auto object = arm_cell_motion::candidate_occupancy_object(p, observation.header, 0);
  ASSERT_EQ(object.meshes.size(), 1U);
  EXPECT_TRUE(object.primitives.empty());
  EXPECT_EQ(object.meshes[0].vertices.size(), 8U);
  EXPECT_EQ(object.meshes[0].triangles.size(), 12U);
  EXPECT_DOUBLE_EQ(object.meshes[0].vertices[1].z, 0.2);
  auto invalid = p;
  invalid.top_vertices.push_back(invalid.top_vertices.front());
  EXPECT_FALSE(arm_cell_motion::valid_candidate_occupancy(true, {invalid}, observation));
  invalid = p; invalid.top_vertices[2].z += 0.1;
  EXPECT_FALSE(arm_cell_motion::valid_candidate_occupancy(true, {invalid}, observation));
  invalid = p; invalid.pose.orientation.w = 0;
  EXPECT_FALSE(arm_cell_motion::valid_candidate_occupancy(true, {invalid}, observation));
  invalid = p; invalid.top_vertices.clear();
  for (int i : {0, 2, 4, 1, 3}) {
    const double angle = i * 2 * std::acos(-1.0) / 5;
    geometry_msgs::msg::Point vertex;
    vertex.x = std::cos(angle); vertex.y = std::sin(angle); vertex.z = 0.1;
    invalid.top_vertices.push_back(vertex);
  }
  EXPECT_FALSE(arm_cell_motion::valid_candidate_occupancy(true, {invalid}, observation));
  p.shape.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  EXPECT_TRUE(arm_cell_motion::valid_candidate_occupancy(true, {p}, observation));
}

TEST(CandidateOccupancy, ConvexTopAcceptsObservedSupportTransformRotation)
{
  using Primitive = arm_cell_interfaces::msg::CollisionPrimitive;
  Primitive p;
  p.shape_type = Primitive::CONVEX_TOP;
  p.occupancy_class = Primitive::UNKNOWN_OCCUPANCY;
  p.bottom_height_m = 0;
  p.pose.orientation.x = 0.0139519940274;
  p.pose.orientation.y = -1.58374243264e-05;
  p.pose.orientation.z = 4.5322955654e-06;
  p.pose.orientation.w = 0.999902666059;
  for (const auto & v : std::vector<std::array<double, 3>>{
      {0, 0, 0.1}, {0.2, 0, 0.2}, {0.2, 0.1, 0.2}, {0, 0.1, 0.1}}) {
    geometry_msgs::msg::Point point;
    point.x = v[0]; point.y = v[1]; point.z = v[2];
    p.top_vertices.push_back(point);
  }
  geometry_msgs::msg::PoseStamped observation;
  observation.header.frame_id = "base_link";

  ASSERT_TRUE(arm_cell_motion::valid_candidate_occupancy(true, {p}, observation));

  const auto object = arm_cell_motion::candidate_occupancy_object(p, observation.header, 0);
  ASSERT_EQ(object.mesh_poses.size(), 1U);
  EXPECT_EQ(object.mesh_poses[0].orientation.x, p.pose.orientation.x);
  EXPECT_EQ(object.mesh_poses[0].orientation.y, p.pose.orientation.y);
  EXPECT_EQ(object.mesh_poses[0].orientation.z, p.pose.orientation.z);
  EXPECT_EQ(object.mesh_poses[0].orientation.w, p.pose.orientation.w);
}
