#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include "arm_cell_motion/candidate_occupancy.hpp"

namespace arm_cell_motion
{
namespace
{
class RequestScopedOccupancyTest : public ::testing::Test
{
protected:
  using Primitive = arm_cell_interfaces::msg::CollisionPrimitive;

  void SetUp() override
  {
    urdf_ =
      R"(
<robot name="vision_request_test">
  <link name="base"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
  <link name="tool"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
  <link name="brace"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
  <joint name="brace_fixed" type="fixed"><parent link="base"/><child link="brace"/>
    <origin xyz="-1 0 0"/></joint>
  <joint name="slide" type="prismatic"><parent link="base"/><child link="tool"/>
    <origin xyz="1 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-2" upper="6" effort="1" velocity="1"/></joint>
</robot>)";
    srdf_ =
      R"(<robot name="vision_request_test"><group name="arm"><joint name="slide"/></group></robot>)";
    auto urdf_model = urdf::parseURDF(urdf_);
    ASSERT_NE(urdf_model, nullptr);
    auto srdf_model = std::make_shared<srdf::Model>();
    ASSERT_TRUE(srdf_model->initString(*urdf_model, srdf_));
    model_ = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
    fixed_scene_ = std::make_shared<planning_scene::PlanningScene>(model_);
    add_box(*fixed_scene_, "fixed_equipment", 3.0);
    add_box(*fixed_scene_, "tray", 4.0);
    add_box(*fixed_scene_, "rail", 5.0);
    fixed_scene_->getPlanningSceneMsg(fixed_message_);
  }

  static void add_box(planning_scene::PlanningScene & scene, const std::string & id, double x)
  {
    moveit_msgs::msg::CollisionObject box;
    box.header.frame_id = "base";
    box.id = id;
    box.operation = moveit_msgs::msg::CollisionObject::ADD;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {0.2, 0.2, 0.2};
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.orientation.w = 1.0;
    box.primitives.push_back(primitive);
    box.primitive_poses.push_back(pose);
    EXPECT_TRUE(scene.processCollisionObjectMsg(box));
  }

  static Primitive box_occupancy(double x, std::uint8_t occupancy_class)
  {
    Primitive primitive;
    primitive.occupancy_class = occupancy_class;
    primitive.shape.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.shape.dimensions = {0.2, 0.2, 0.2};
    primitive.pose.position.x = x;
    primitive.pose.orientation.w = 1.0;
    return primitive;
  }

  bool collides(const planning_scene::PlanningScene & scene, double q) const
  {
    moveit::core::RobotState state(scene.getCurrentState());
    state.setVariablePosition("slide", q);
    state.update();
    collision_detection::CollisionRequest request;
    collision_detection::CollisionResult result;
    scene.checkCollision(request, result, state);
    return result.collision;
  }

  moveit::core::RobotModelPtr model_;
  planning_scene::PlanningScenePtr fixed_scene_;
  moveit_msgs::msg::PlanningScene fixed_message_;
  std::string urdf_;
  std::string srdf_;
};

TEST_F(RequestScopedOccupancyTest, PickApproachDiffAddsOnlyItsObservedAndUnknownGeometry)
{
  const std::vector<Primitive> occupancy{
    box_occupancy(1.0, Primitive::OBSERVED_OCCUPANCY),
    box_occupancy(2.0, Primitive::UNKNOWN_OCCUPANCY)};
  std_msgs::msg::Header header;
  header.frame_id = "base";
  std::vector<std::string> ids;
  const auto diff = candidate_scene_diff({}, occupancy, header, ids);
  ASSERT_EQ(
    ids, (std::vector<std::string>{
        "vision_candidate_occupancy_0", "vision_candidate_occupancy_1"}));
  ASSERT_EQ(diff.world.collision_objects.size(), 2U);
  EXPECT_TRUE(
    std::all_of(
      diff.world.collision_objects.begin(), diff.world.collision_objects.end(),
      [](const auto & object) {
        return object.operation == moveit_msgs::msg::CollisionObject::ADD;
      }));

  planning_scene::PlanningScene pick_context(model_);
  ASSERT_TRUE(pick_context.setPlanningSceneMsg(fixed_message_));
  pick_context.setPlanningSceneDiffMsg(diff);
  EXPECT_TRUE(collides(pick_context, 0.0));
  EXPECT_TRUE(collides(pick_context, 1.0));
  EXPECT_TRUE(collides(pick_context, 2.0));  // Fixed equipment remains active.
  EXPECT_TRUE(collides(pick_context, 3.0));  // Tray remains active.
  EXPECT_TRUE(collides(pick_context, -2.0));  // Robot self-collision remains active.
  EXPECT_TRUE(candidate_occupancy_ids(fixed_message_).empty());
}

TEST_F(RequestScopedOccupancyTest, CandidateTransitionAndPlaceUseIndependentContexts)
{
  std_msgs::msg::Header header;
  header.frame_id = "base";
  std::vector<std::string> first_ids;
  std::vector<std::string> second_ids;
  const auto first_diff = candidate_scene_diff(
    {}, {box_occupancy(1.0, Primitive::OBSERVED_OCCUPANCY)}, header, first_ids);
  const auto second_diff = candidate_scene_diff(
    {}, {box_occupancy(2.0, Primitive::UNKNOWN_OCCUPANCY)}, header, second_ids);

  planning_scene::PlanningScene first_context(model_);
  planning_scene::PlanningScene second_context(model_);
  planning_scene::PlanningScene place_context(model_);
  ASSERT_TRUE(first_context.setPlanningSceneMsg(fixed_message_));
  ASSERT_TRUE(second_context.setPlanningSceneMsg(fixed_message_));
  ASSERT_TRUE(place_context.setPlanningSceneMsg(fixed_message_));
  first_context.setPlanningSceneDiffMsg(first_diff);
  second_context.setPlanningSceneDiffMsg(second_diff);
  EXPECT_TRUE(collides(first_context, 0.0));
  EXPECT_FALSE(collides(second_context, 0.0));
  EXPECT_TRUE(collides(second_context, 1.0));
  EXPECT_FALSE(collides(place_context, 0.0));
  EXPECT_FALSE(collides(place_context, 1.0));
  EXPECT_TRUE(collides(place_context, 2.0));
  EXPECT_TRUE(candidate_occupancy_ids(fixed_message_).empty());
}

TEST_F(RequestScopedOccupancyTest, PlaceDiffRemovesLegacyTemporaryIdsButPreservesOtherSceneState)
{
  auto legacy_scene = std::make_shared<planning_scene::PlanningScene>(model_);
  ASSERT_TRUE(legacy_scene->setPlanningSceneMsg(fixed_message_));
  add_box(*legacy_scene, "vision_candidate_occupancy_0", 0.5);
  add_box(*legacy_scene, "vision_candidate_occupancy_1", 1.5);
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = "tool";
  attached.touch_links = {"tool"};
  attached.object.id = "held_raw_part";
  attached.object.header.frame_id = "tool";
  attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  primitive.dimensions = {0.1};
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  attached.object.primitives.push_back(primitive);
  attached.object.primitive_poses.push_back(pose);
  ASSERT_TRUE(legacy_scene->processAttachedCollisionObjectMsg(attached));
  legacy_scene->getAllowedCollisionMatrixNonConst().setEntry("base", "tray", true);
  moveit_msgs::msg::PlanningScene legacy_message;
  legacy_scene->getPlanningSceneMsg(legacy_message);

  const auto old_ids = candidate_occupancy_ids(legacy_message);
  ASSERT_EQ(old_ids.size(), 2U);
  std_msgs::msg::Header header;
  header.frame_id = "base";
  std::vector<std::string> ignored;
  const auto place_diff = candidate_scene_diff(old_ids, {}, header, ignored);
  planning_scene::PlanningScene place_context(model_);
  ASSERT_TRUE(place_context.setPlanningSceneMsg(legacy_message));
  place_context.setPlanningSceneDiffMsg(place_diff);

  EXPECT_FALSE(collides(place_context, 0.0));
  EXPECT_FALSE(collides(place_context, 1.0));
  EXPECT_TRUE(collides(place_context, 2.0));
  std::vector<const moveit::core::AttachedBody *> attached_bodies;
  place_context.getCurrentState().getAttachedBodies(attached_bodies);
  EXPECT_EQ(attached_bodies.size(), 1U);
  collision_detection::AllowedCollision::Type allowed;
  ASSERT_TRUE(place_context.getAllowedCollisionMatrix().getEntry("base", "tray", allowed));
  EXPECT_EQ(allowed, collision_detection::AllowedCollision::ALWAYS);
  EXPECT_EQ(candidate_occupancy_ids(legacy_message), old_ids);

  std::vector<std::string> excluded_ids;
  const auto reusable_excluded_diff = candidate_scene_diff(
    old_ids, {box_occupancy(0.75, Primitive::OBSERVED_OCCUPANCY)}, header,
    VisionOccupancyPolicy::EXCLUDE, excluded_ids);
  EXPECT_TRUE(excluded_ids.empty());
  ASSERT_EQ(reusable_excluded_diff.world.collision_objects.size(), old_ids.size());
  EXPECT_TRUE(
    std::all_of(
      reusable_excluded_diff.world.collision_objects.begin(),
      reusable_excluded_diff.world.collision_objects.end(), [](const auto & object) {
        return object.operation == moveit_msgs::msg::CollisionObject::REMOVE;
      }));
}

TEST_F(RequestScopedOccupancyTest, MoveItPlanningPipelineUsesPickAndPlaceRequestSceneDiffs)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>(
    "request_scene_pipeline_test", rclcpp::NodeOptions().use_global_arguments(false));
  node->declare_parameter<std::string>("robot_description", urdf_);
  node->declare_parameter<std::string>("robot_description_semantic", srdf_);
  node->declare_parameter<std::string>(
    "planning_pipelines.ompl.planning_plugin", "ompl_interface/OMPLPlanner");
  node->declare_parameter<std::string>("planning_pipelines.ompl.request_adapters", "");
  node->declare_parameter<std::string>(
    "planning_pipelines.ompl.planner_configs.RRTConnect.type", "geometric::RRTConnect");
  node->declare_parameter<double>(
    "planning_pipelines.ompl.planner_configs.RRTConnect.range", 0.0);
  node->declare_parameter<std::string>(
    "planning_pipelines.ompl.arm.default_planner_config", "RRTConnect");
  node->declare_parameter<std::vector<std::string>>(
    "planning_pipelines.ompl.arm.planner_configs", {"RRTConnect"});
  node->declare_parameter<double>(
    "planning_pipelines.ompl.arm.longest_valid_segment_fraction", 0.005);

  auto planning_scene = std::make_shared<planning_scene::PlanningScene>(model_);
  planning_scene_monitor::PlanningSceneMonitor scene_monitor(
    node, planning_scene, "robot_description", "pick_request_scene");
  planning_pipeline::PlanningPipeline pipeline(
    scene_monitor.getRobotModel(), node, "planning_pipelines.ompl",
    "ompl_interface/OMPLPlanner", std::vector<std::string>{});

  moveit_msgs::msg::MotionPlanRequest request;
  request.group_name = "arm";
  request.planner_id = "RRTConnect";
  request.allowed_planning_time = 0.5;
  request.num_planning_attempts = 1;
  moveit::core::RobotState start(scene_monitor.getRobotModel());
  start.setToDefaultValues();
  start.setVariablePosition("slide", 0.0);
  start.update();
  moveit::core::robotStateToRobotStateMsg(start, request.start_state, true);
  moveit_msgs::msg::JointConstraint goal;
  goal.joint_name = "slide";
  goal.position = 4.0;
  goal.tolerance_above = 0.001;
  goal.tolerance_below = 0.001;
  goal.weight = 1.0;
  request.goal_constraints.resize(1);
  request.goal_constraints.front().joint_constraints.push_back(goal);

  std_msgs::msg::Header header;
  header.frame_id = "base";
  std::vector<std::string> pick_ids;
  const auto pick_diff = candidate_scene_diff(
    {}, {box_occupancy(3.0, Primitive::UNKNOWN_OCCUPANCY)}, header, pick_ids);
  const auto pick_scene = scene_monitor.copyPlanningScene(pick_diff);
  ASSERT_NE(pick_scene, nullptr);
  planning_interface::MotionPlanResponse pick_result;
  EXPECT_FALSE(pipeline.generatePlan(pick_scene, request, pick_result));

  auto legacy_scene = std::make_shared<planning_scene::PlanningScene>(
    scene_monitor.getRobotModel());
  add_box(*legacy_scene, "vision_candidate_occupancy_legacy", 3.0);
  planning_scene_monitor::PlanningSceneMonitor legacy_scene_monitor(
    node, legacy_scene, scene_monitor.getRobotModelLoader(), "legacy_place_request_scene");
  std::vector<std::string> ignored_ids;
  const auto place_diff = candidate_scene_diff(
    {"vision_candidate_occupancy_legacy"}, {}, header, ignored_ids);
  const auto place_scene = legacy_scene_monitor.copyPlanningScene(place_diff);
  ASSERT_NE(place_scene, nullptr);
  planning_interface::MotionPlanResponse place_result;
  EXPECT_TRUE(pipeline.generatePlan(place_scene, request, place_result));
  ASSERT_NE(place_result.trajectory_, nullptr);
  EXPECT_GT(place_result.trajectory_->getWayPointCount(), 1U);

  const auto unchanged_scene = legacy_scene_monitor.copyPlanningScene();
  ASSERT_NE(unchanged_scene, nullptr);
  EXPECT_TRUE(collides(*unchanged_scene, 2.0));
}

}  // namespace
}  // namespace arm_cell_motion
