#include <gtest/gtest.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <nlohmann/json.hpp>
#include "arm_cell_motion/pick_approach_planning_bt.hpp"
#include "arm_cell_motion/pick_planning_diagnostics.hpp"

namespace arm_cell_motion
{
namespace
{
using Primitive = arm_cell_interfaces::msg::CollisionPrimitive;
constexpr GraspOccupancyMode kModes[] = {GraspOccupancyMode::NONE,
  GraspOccupancyMode::FULL_COLLISION, GraspOccupancyMode::FINGER_CLEARANCE};
Primitive box(double x, double y = 0.0, double z = -0.01, double size = 0.002)
{
  Primitive p;
  p.shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  p.shape.dimensions = {size, size, size};
  p.pose.orientation.w = 1.0;
  p.pose.position.x = x; p.pose.position.y = y; p.pose.position.z = z;
  return p;
}
class GraspCollisionValidatorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    moveit::core::RobotModelBuilder builder("grasp_collision", "base_link");
    builder.addChain("base_link->sf_grasp_tcp", "prismatic");
    builder.addChain("base_link->other_link", "prismatic");
    builder.addGroupChain("base_link", "sf_grasp_tcp", "arm");
    geometry_msgs::msg::Pose origin;
    origin.orientation.w = 1.0;
    builder.addCollisionBox("sf_grasp_tcp", {0.01, 0.01, 0.01}, origin);
    builder.addCollisionBox("other_link", {0.01, 0.01, 0.01}, origin);
    model = builder.build();
    ASSERT_NE(model, nullptr);
    context = std::make_shared<PlanningContext>();
    context->robot_model = model;
    context->approach_axis = PickApproachAxis::TCP_INSERTION_AXIS;
    context->insertion_axis_tcp.z = 1.0;
    context->object_pose.header.frame_id = "base_link";
    context->attempt.current_grasp_goal.header.frame_id = "base_link";
    context->attempt.current_grasp_goal.pose.orientation.w = 1.0;
    context->attempt.grasp_contact_result.contact = BoxGraspContact{0U, 60.0};
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    // Keep the second link separate unless explicitly testing self collision.
    state.setVariablePosition("base_link-other_link-joint", 0.5);
    state.update();
    context->attempt.grasp_validation_result.emplace(
      PickGraspKinematics{
          PickGraspKinematicCheck::VALID, state, Eigen::Quaterniond::Identity(), ""});
  }
  void scenes(const std::vector<Primitive> & occupancy, bool fixture = false)
  {
    moveit_msgs::msg::PlanningScene snapshot;
    planning_scene::PlanningScene baseline(model);
    baseline.getPlanningSceneMsg(snapshot);
    if (fixture) {
      moveit_msgs::msg::CollisionObject object;
      object.header.frame_id = "base_link";
      object.id = "static_fixture";
      object.operation = object.ADD;
      auto p = box(0.0, 0.0, 0.0, 0.1);
      object.primitives.push_back(p.shape);
      object.primitive_poses.push_back(p.pose);
      snapshot.world.collision_objects.push_back(object);
    }
    context->scene_builder = std::make_unique<PlanningSceneBuilder>(
      snapshot, occupancy, context->object_pose.header);
    context->non_target_occupancy = occupancy;
  }
  BT::NodeStatus tick(const std::string & leaves)
  {
    BT::BehaviorTreeFactory factory;
    register_pick_approach_planning_nodes(factory);
    const auto kinematics = context->attempt.grasp_validation_result;
    factory.registerSimpleAction(
      "RestoreValidatedContact", [this, kinematics](BT::TreeNode &) {
        context->attempt.grasp_validation_result = kinematics;
        context->attempt.current_grasp_goal.pose.orientation.w = 1.0;
        context->attempt.grasp_contact_result.contact = BoxGraspContact{0U, 60.0};
        return BT::NodeStatus::SUCCESS;
      });
    auto bb = BT::Blackboard::create();
    bb->set("context", context);
    auto tree = factory.createTreeFromText(
      "<root main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\"><Sequence>" + leaves +
      "</Sequence></BehaviorTree></root>", bb);
    return tree.tickRoot();
  }
  BT::NodeStatus validate()
  {
    return tick("<SetGraspScenePolicy/><ValidateGraspCollision/>");
  }
  moveit::core::RobotModelPtr model;
  std::shared_ptr<PlanningContext> context;
};
TEST_F(GraspCollisionValidatorTest, ProductionLeafSelectsAllThreeStrategies)
{
  scenes({box(0.0, 0.0, 0.0, 0.01)});  // Robot collision away from finger protection.
  for (auto mode : kModes) {
    context->grasp_occupancy_mode = mode;
    const auto status = validate();
    EXPECT_EQ(
      status, mode == GraspOccupancyMode::FULL_COLLISION ?
      BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS);
    const auto & result = context->attempt.grasp_occupancy_result;
    EXPECT_EQ(result.mode, mode);
    EXPECT_FALSE(result.common.collision);
    if (mode == GraspOccupancyMode::FULL_COLLISION) {
      const auto old = validate_pick_state_collision(
        *context->grasp_scene, context->attempt.grasp_validation_result->state);
      EXPECT_EQ(result.occupancy.collision, old.collision);
      EXPECT_EQ(result.occupancy.contacts, old.contacts);
    }
  }
}
TEST_F(GraspCollisionValidatorTest, StaticFixtureAndSelfCollisionAreRejectedInEveryMode)
{
  for (bool fixture : {true, false}) {
    scenes({}, fixture);
    auto & state = context->attempt.grasp_validation_result->state;
    state.setVariablePosition("base_link-other_link-joint", fixture ? 0.5 : 0.0);
    state.update();
    for (auto mode : kModes) {
      context->grasp_occupancy_mode = mode;
      EXPECT_EQ(validate(), BT::NodeStatus::FAILURE);
      EXPECT_TRUE(context->attempt.grasp_occupancy_result.common.collision);
      EXPECT_EQ(context->attempt.attempt_evaluation, PickSearchEvaluation::GRASP_COLLISION);
      EXPECT_EQ(context->attempt.failure_reason, "COMMON_COLLISION");
    }
  }
}
TEST_F(GraspCollisionValidatorTest, FullCollisionPreservesLegacyFeedbackWithCommonAndVisionContacts)
{
  scenes({box(0.0, 0.0, 0.0, 0.01)}, true);
  EXPECT_EQ(validate(), BT::NodeStatus::FAILURE);
  const auto legacy = validate_pick_state_collision(
    *context->grasp_scene, context->attempt.grasp_validation_result->state);
  EXPECT_EQ(context->attempt.grasp_collision_result.contacts, legacy.contacts);
  EXPECT_TRUE(context->attempt.grasp_occupancy_result.common.collision);
  EXPECT_TRUE(context->attempt.grasp_occupancy_result.occupancy_evaluated);
  EXPECT_EQ(context->latest_collision_contacts.size(), legacy.contacts.size());
}
TEST_F(GraspCollisionValidatorTest, OneFingerIntrusionHasIdAndNoFabricatedRobotContacts)
{
  scenes({box(0.035)});
  context->grasp_occupancy_mode = GraspOccupancyMode::FINGER_CLEARANCE;
  EXPECT_EQ(validate(), BT::NodeStatus::FAILURE);
  const auto & result = context->attempt.grasp_occupancy_result;
  ASSERT_EQ(result.intrusions.size(), 1U);
  EXPECT_EQ(result.intrusions[0].finger_index, 1U);
  EXPECT_EQ(result.intrusions[0].occupancy_id, "vision_candidate_occupancy_0");
  EXPECT_TRUE(context->latest_collision_contacts.empty());
  EXPECT_EQ(context->attempt.attempt_evaluation, PickSearchEvaluation::GRASP_COLLISION);
  EXPECT_EQ(context->attempt.failure_reason, "FINGER_CLEARANCE_INTRUSION");
  PickPlanningDiagnostics diagnostics;
  diagnostics.resize(1U, true);
  diagnostics.collect(context->attempt, context->result, PickPlanningDiagnosticMetadata{});
  const auto record = nlohmann::json::parse(diagnostics.summary().pick_attempt_records.front());
  const auto & validation = record.at("grasp_collision_validation");
  EXPECT_EQ(validation.at("mode"), "FINGER_CLEARANCE");
  EXPECT_EQ(validation.at("finger_volumes").size(), 2U);
  EXPECT_EQ(validation.at("intrusions")[0].at("finger_index"), 1U);
  EXPECT_EQ(validation.at("allowed"), false);
}
TEST_F(GraspCollisionValidatorTest, FullCollisionUsesConvexMeshInsteadOfBoundingBox)
{
  Primitive p; p.shape_type = Primitive::CONVEX_TOP; p.pose.orientation.w = 1;
  p.bottom_height_m = -0.01;
  const auto set_vertices = [&](const std::vector<Eigen::Vector2d> & vertices) {
      p.top_vertices.clear();
      for (const auto & v : vertices) {
        geometry_msgs::msg::Point point; point.x=v.x(); point.y=v.y(); point.z=0.01;
        p.top_vertices.push_back(point);
      }
    };
  context->grasp_occupancy_mode = GraspOccupancyMode::FULL_COLLISION;
  set_vertices({{0.004,0.04},{0.04,0.004},{0.04,0.04}});
  scenes({p});
  EXPECT_EQ(validate(), BT::NodeStatus::SUCCESS);
  set_vertices({{-0.003,-0.003},{0.006,-0.003},{-0.003,0.006}});
  scenes({p});
  EXPECT_EQ(validate(), BT::NodeStatus::FAILURE);
}
TEST_F(GraspCollisionValidatorTest, ConvexTopIntrusionUsesFingerClearance)
{
  Primitive p; p.shape_type = Primitive::CONVEX_TOP; p.pose.orientation.w = 1;
  p.bottom_height_m = -0.015;
  for (const auto & v : std::vector<Eigen::Vector3d>{{0.034,-0.002,-0.005},
      {0.036,-0.002,-0.004}, {0.036,0.002,-0.004}, {0.034,0.002,-0.005}}) {
    geometry_msgs::msg::Point point; point.x=v.x(); point.y=v.y(); point.z=v.z();
    p.top_vertices.push_back(point);
  }
  scenes({p});
  context->grasp_occupancy_mode = GraspOccupancyMode::FINGER_CLEARANCE;
  EXPECT_EQ(validate(), BT::NodeStatus::FAILURE);
  ASSERT_EQ(context->attempt.grasp_occupancy_result.intrusions.size(), 1U);
  EXPECT_EQ(context->attempt.grasp_occupancy_result.intrusions.front().finger_index, 1U);
}
TEST_F(GraspCollisionValidatorTest, TouchingMaterialAwayFromActualContactSpanIsAllowed)
{
  // Neighbour touches the +X target face, but away from this candidate's finger pads.
  scenes({box(0.035, 0.045, -0.01, 0.01)});
  context->grasp_occupancy_mode = GraspOccupancyMode::FINGER_CLEARANCE;
  EXPECT_EQ(validate(), BT::NodeStatus::SUCCESS);
  scenes({box(0.035, 0.0, -0.01, 0.01)});
  EXPECT_EQ(validate(), BT::NodeStatus::FAILURE);
}
TEST_F(GraspCollisionValidatorTest, UnusedFingerDimensionsCannotChangeOtherModeDecisions)
{
  scenes({box(0.0, 0.0, 0.0, 0.01)});
  context->finger_clearance.radius_m = std::nan("");
  context->finger_clearance.contact_span_m = -1.0;
  context->finger_clearance.depth_m = 1000.0;
  context->attempt.grasp_contact_result.contact.reset();
  for (auto mode : {GraspOccupancyMode::NONE, GraspOccupancyMode::FULL_COLLISION}) {
    context->grasp_occupancy_mode = mode;
    EXPECT_EQ(
      validate(), mode == GraspOccupancyMode::NONE ?
      BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE);
    EXPECT_TRUE(context->attempt.grasp_occupancy_result.valid);
  }
}
TEST_F(GraspCollisionValidatorTest, ClearSceneMatchesLegacyAndBothFingerIdsAreRecorded)
{
  scenes({});
  for (auto mode : kModes) {
    context->grasp_occupancy_mode = mode;
    EXPECT_EQ(validate(), BT::NodeStatus::SUCCESS);
    EXPECT_FALSE(
      validate_pick_state_collision(
        *context->grasp_scene,
        context->attempt.grasp_validation_result->state).collision);
  }
  scenes({box(-0.035), box(0.035)});
  context->grasp_occupancy_mode = GraspOccupancyMode::FINGER_CLEARANCE;
  EXPECT_EQ(validate(), BT::NodeStatus::FAILURE);
  const auto & intrusions = context->attempt.grasp_occupancy_result.intrusions;
  ASSERT_EQ(intrusions.size(), 2U);
  EXPECT_EQ(intrusions[0].finger_index, 0U);
  EXPECT_EQ(intrusions[1].finger_index, 1U);
  EXPECT_NE(intrusions[0].occupancy_id, intrusions[1].occupancy_id);
}
TEST_F(GraspCollisionValidatorTest, UnsupportedGeometryIsAnExplicitCalculationError)
{
  auto cylinder = box(0.035);
  cylinder.shape.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  cylinder.shape.dimensions = {0.01, 0.002};
  scenes({cylinder});
  context->grasp_occupancy_mode = GraspOccupancyMode::FINGER_CLEARANCE;
  EXPECT_EQ(validate(), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(context->attempt.grasp_occupancy_result.valid);
  EXPECT_EQ(context->attempt.failure_reason, "FINGER_OCCUPANCY_REQUIRES_BOX");
  EXPECT_EQ(context->attempt.attempt_evaluation, PickSearchEvaluation::BACKEND_ERROR);
}
TEST_F(GraspCollisionValidatorTest, FingerCollisionUsesExistingPoseSeedRetry)
{
  scenes({box(0.035)});
  context->grasp_occupancy_mode = GraspOccupancyMode::FINGER_CLEARANCE;
  context->planning_budget = std::make_unique<PickSearchBudget>(5.0);
  context->search_controller = std::make_unique<PickSearchController>(1U, 2U);
  EXPECT_EQ(
    tick(
      "<SearchPoseSeed><Sequence><RestoreValidatedContact/><SetGraspScenePolicy/>"
      "<ValidateGraspCollision/></Sequence></SearchPoseSeed>"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(context->search_controller->attempt_count(), 2U);
  EXPECT_EQ(context->attempt.attempt_evaluation, PickSearchEvaluation::GRASP_COLLISION);
}
}  // namespace
}  // namespace arm_cell_motion
