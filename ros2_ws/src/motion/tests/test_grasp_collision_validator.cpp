#include <gtest/gtest.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <nlohmann/json.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
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
    context->joint_group = model->getJointModelGroup("arm");
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
        *context->grasp_scene, context->attempt.grasp_validation_result->state, "arm");
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
    *context->grasp_scene, context->attempt.grasp_validation_result->state, "arm");
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
        geometry_msgs::msg::Point point; point.x = v.x(); point.y = v.y(); point.z = 0.01;
        p.top_vertices.push_back(point);
      }
    };
  context->grasp_occupancy_mode = GraspOccupancyMode::FULL_COLLISION;
  set_vertices({{0.004, 0.04}, {0.04, 0.004}, {0.04, 0.04}});
  scenes({p});
  EXPECT_EQ(validate(), BT::NodeStatus::SUCCESS);
  set_vertices({{-0.003, -0.003}, {0.006, -0.003}, {-0.003, 0.006}});
  scenes({p});
  EXPECT_EQ(validate(), BT::NodeStatus::FAILURE);
}
TEST_F(GraspCollisionValidatorTest, ConvexTopIntrusionUsesFingerClearance)
{
  Primitive p; p.shape_type = Primitive::CONVEX_TOP; p.pose.orientation.w = 1;
  p.bottom_height_m = -0.015;
  for (const auto & v : std::vector<Eigen::Vector3d>{{0.034, -0.002, -0.005},
      {0.036, -0.002, -0.004}, {0.036, 0.002, -0.004}, {0.034, 0.002, -0.005}})
  {
    geometry_msgs::msg::Point point; point.x = v.x(); point.y = v.y(); point.z = v.z();
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
        context->attempt.grasp_validation_result->state, "arm").collision);
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
class GraspPosePrecheck : public ::testing::TestWithParam<std::string> {};
TEST_P(GraspPosePrecheck, RejectsSelectedObjectFingerIntersectionBeforeCallingIK)
{
  moveit::core::RobotModelBuilder builder("precheck", "base_link");
  builder.addChain("base_link->sf_grasp_tcp", "prismatic");
  builder.addChain("sf_grasp_tcp->tool_contact_pad", "fixed");
  builder.addGroupChain("base_link", "sf_grasp_tcp", "arm");
  builder.addChain("base_link->gripper_unrelated", "prismatic");
  geometry_msgs::msg::Pose origin;
  origin.orientation.w = 1.0;
  builder.addCollisionBox("tool_contact_pad", {0.02, 0.02, 0.02}, origin);
  builder.addCollisionBox("gripper_unrelated", {0.02, 0.02, 0.02}, origin);
  const auto model = builder.build();
  ASSERT_NE(model, nullptr);
  auto context = std::make_shared<PlanningContext>();
  context->robot_model = model;
  context->joint_group = model->getJointModelGroup("arm");
  auto state = std::make_shared<moveit::core::RobotState>(model);
  state->setToDefaultValues();
  state->update();
  // Measured snapshots can have dirty transforms; the precheck updates its own copy.
  state->setVariablePosition(context->joint_group->getVariableNames().front(), 0.5);
  context->robot_state = state;
  context->common_grasp_scene = std::make_shared<planning_scene::PlanningScene>(model);
  context->pick_clearance_objects = {GetParam(), "absent_object"};
  context->tool_grasp_collision_links = {"tool_contact_pad"};
  context->common_grasp_scene->getWorldNonConst()->addToObject(
    GetParam(), shapes::ShapeConstPtr(new shapes::Box(0.02, 0.02, 0.02)),
    Eigen::Isometry3d::Identity());
  context->attempt.current_grasp_goal.header.frame_id = "base_link";
  context->attempt.current_grasp_goal.pose.orientation.w = 1.0;
  context->planning_budget = std::make_unique<PickSearchBudget>(5.0);
  BT::BehaviorTreeFactory factory;
  register_pick_approach_planning_nodes(factory);
  auto bb = BT::Blackboard::create();
  bb->set("context", context);
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\"><Sequence><ValidateGraspPose/><SolveGraspIK/></Sequence>"
    "</BehaviorTree></root>",
    bb);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(context->attempt.failure_reason, "TCP_RIGID_RAIL_COLLISION");
  EXPECT_EQ(context->attempt.attempt_evaluation, PickSearchEvaluation::GRASP_COLLISION);
  EXPECT_FALSE(context->attempt.grasp_ik_result.solver_returned);
  EXPECT_EQ(context->attempt.stage_elapsed_s.count("SolveGraspIK"), 0U);
  ASSERT_EQ(context->attempt.grasp_collision_result.contacts.size(), 1U);
  EXPECT_EQ(context->attempt.grasp_collision_result.contacts.front().first, GetParam());

  // Relocating the TCP away from the rail must pass through to the IK leaf.
  context->attempt = PickPlanningAttempt();
  context->attempt.current_grasp_goal.header.frame_id = "base_link";
  context->attempt.current_grasp_goal.pose.orientation.w = 1.0;
  context->attempt.current_grasp_goal.pose.position.x = 0.1;
  tree.haltTree();
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(context->attempt.failure_reason, "IK_SOLVER_UNAVAILABLE");

  // An explicit allowed-collision entry must also defer to normal IK checks.
  context->common_grasp_scene->getAllowedCollisionMatrixNonConst().setEntry(
    GetParam(), "tool_contact_pad", true);
  context->attempt.current_grasp_goal.pose.position.x = 0.0;
  tree.haltTree();
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(context->attempt.failure_reason, "IK_SOLVER_UNAVAILABLE");

  // An unselected object still exists in the scene and its ACM is untouched.
  context->common_grasp_scene->getAllowedCollisionMatrixNonConst().setEntry(
    GetParam(), "tool_contact_pad", false);
  context->pick_clearance_objects = {"absent_object"};
  tree.haltTree();
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(context->attempt.failure_reason, "IK_SOLVER_UNAVAILABLE");
  collision_detection::AllowedCollision::Type allowed;
  ASSERT_TRUE(
    context->common_grasp_scene->getAllowedCollisionMatrix().getAllowedCollision(
      GetParam(), "tool_contact_pad", allowed));
  EXPECT_EQ(allowed, collision_detection::AllowedCollision::NEVER);

}

INSTANTIATE_TEST_SUITE_P(
  CellObjectIds, GraspPosePrecheck,
  ::testing::Values("amr_tray_rail_left", "fixture_guard"));

TEST_F(GraspCollisionValidatorTest, SelectedSceneBoxesPreserveFingerClearanceResults)
{
  planning_scene::PlanningScene scene(model);
  const auto world = scene.getWorldNonConst();
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(0.035, 0.0, -0.01);
  const auto guard = shapes::ShapeConstPtr(new shapes::Box(0.002, 0.002, 0.002));
  world->addToObject("fixture_guard", guard, pose);
  world->addToObject("unrelated", shapes::ShapeConstPtr(new shapes::Box(1.0, 1.0, 1.0)), pose);
  world->addToObject("round_guard", shapes::ShapeConstPtr(new shapes::Sphere(0.1)), pose);
  scene.getAllowedCollisionMatrixNonConst().setEntry("unrelated", "sf_grasp_tcp", true);
  moveit_msgs::msg::PlanningScene before;
  scene.getPlanningSceneMsg(before);
  std::vector<Primitive> occupancy;
  append_pick_clearance_scene_occupancy(
    scene, {"fixture_guard", "missing", "round_guard"}, occupancy);
  ASSERT_EQ(occupancy.size(), 1U);
  EXPECT_EQ(occupancy.front(), box(0.035));
  const auto evaluate = [](const std::vector<Primitive> & values) {
      return evaluate_finger_occupancy(
        Eigen::Isometry3d::Identity(), Eigen::Vector3d::Zero(),
        Eigen::Quaterniond::Identity(), BoxGraspContact{0U, 60.0},
        Eigen::Vector3d::UnitZ(), values, FingerClearanceParameters{});
    };
  const auto intrusion = evaluate(occupancy);
  const auto previous = evaluate({box(0.035)});
  ASSERT_TRUE(intrusion.valid);
  EXPECT_FALSE(intrusion.allowed);
  EXPECT_EQ(intrusion.allowed, previous.allowed);
  ASSERT_EQ(intrusion.intrusions.size(), previous.intrusions.size());
  EXPECT_EQ(intrusion.intrusions.front().finger_index, previous.intrusions.front().finger_index);

  // Scene transforms and every BOX on a selected object remain the geometry source.
  Eigen::Isometry3d far = Eigen::Isometry3d::Identity();
  far.translation().x() = 0.5;
  far.linear() = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  world->addToObject("clear_guard", shapes::ShapeConstPtr(new shapes::Box(0.01, 0.02, 0.03)), far);
  world->addToObject("clear_guard", shapes::ShapeConstPtr(new shapes::Box(0.02, 0.03, 0.04)), far);
  occupancy = {box(-0.5)};
  append_pick_clearance_scene_occupancy(scene, {"clear_guard"}, occupancy);
  ASSERT_EQ(occupancy.size(), 3U);
  EXPECT_EQ(occupancy.front(), box(-0.5));
  EXPECT_DOUBLE_EQ(occupancy[1].pose.position.x, 0.5);
  EXPECT_DOUBLE_EQ(occupancy[1].pose.orientation.z, std::sin(0.2));
  EXPECT_EQ(
    (std::vector<double>(
      occupancy[1].shape.dimensions.begin(),
      occupancy[1].shape.dimensions.end())),
    (std::vector<double>{0.01, 0.02, 0.03}));
  EXPECT_TRUE(evaluate(occupancy).allowed);
  // Neither projection nor the precheck writes world objects or ACM entries.
  world->removeObject("clear_guard");
  moveit_msgs::msg::PlanningScene after;
  scene.getPlanningSceneMsg(after);
  EXPECT_EQ(before.world, after.world);
  EXPECT_EQ(before.allowed_collision_matrix, after.allowed_collision_matrix);
}
}  // namespace arm_cell_motion

namespace arm_cell_motion
{
TEST(ClosedGraspCollision, MissingToolModelCannotSkipRequestedTargetCheck)
{
  moveit::core::RobotModelBuilder builder("missing_tool", "base_link");
  builder.addChain("base_link->sf_grasp_tcp", "prismatic");
  auto model = builder.build();
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  planning_scene::PlanningScene scene(model);
  const auto result = validate_grasp_collision(
    GraspOccupancyMode::NONE, scene, scene, state, Eigen::Isometry3d::Identity(),
    Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), BoxGraspContact{0U, 70.0},
    Eigen::Vector3d::UnitZ(), {}, FingerClearanceParameters{}, "",
    Eigen::Vector3d(0.07, 0.07, 0.08));
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.allowed);
}
}  // namespace arm_cell_motion

namespace arm_cell_motion
{
namespace
{
class ClosedGraspCollisionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const std::string xml =
      R"(<robot name="closed_tool">
      <link name="base_link"><collision><origin xyz="0 0 0.3"/>
      <geometry><box size="0.02 0.02 0.02"/></geometry></collision></link>
      <link name="gripper_robotiq_85_left_knuckle_link"/>
      <joint name="gripper_robotiq_85_left_knuckle_joint" type="revolute">
      <parent link="base_link"/><child link="gripper_robotiq_85_left_knuckle_link"/>
      <axis xyz="0 1 0"/><limit lower="0" upper="0.8" effort="1" velocity="1"/></joint>
      <link name="gripper_robotiq_85_left_finger_tip_link"><collision>
      <geometry><box size="0.01 0.01 0.01"/></geometry></collision></link>
      <joint name="tip" type="fixed"><parent link="gripper_robotiq_85_left_knuckle_link"/>
      <child link="gripper_robotiq_85_left_finger_tip_link"/><origin xyz="0.05 0 0"/></joint>
      </robot>)";
    auto urdf = urdf::parseURDF(xml);
    auto srdf = std::make_shared<srdf::Model>();
    ASSERT_TRUE(srdf->initString(*urdf, "<robot name=\"closed_tool\"/>"));
    model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    state = std::make_unique<moveit::core::RobotState>(model);
    state->setToDefaultValues();
    scene = std::make_unique<planning_scene::PlanningScene>(model);
  }
  GraspCollisionValidationResult check(
    double width = 85.0,
    Eigen::Vector3d center = Eigen::Vector3d::Zero(),
    GraspOccupancyMode mode = GraspOccupancyMode::NONE)
  {
    return validate_grasp_collision(
      mode, *scene, *scene, *state, Eigen::Isometry3d::Identity(),
      center, Eigen::Quaterniond::Identity(), BoxGraspContact{0U, width},
      Eigen::Vector3d::UnitZ(), {}, FingerClearanceParameters{}, "",
      Eigen::Vector3d(0.07, 0.07, 0.08));
  }
  void floor(double depth, const std::string & id = "amr_tray")
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation().z() = -0.005 - 0.01 + depth;
    scene->getWorldNonConst()->removeObject(id);
    scene->getWorldNonConst()->addToObject(
      id,
      shapes::ShapeConstPtr(new shapes::Box(0.2, 0.2, 0.02)), pose);
  }
  moveit::core::RobotModelPtr model;
  std::unique_ptr<moveit::core::RobotState> state;
  std::unique_ptr<planning_scene::PlanningScene> scene;
};
TEST_F(ClosedGraspCollisionTest, AllowsOnlyShallowTipFloorContactAndPreservesInput)
{
  floor(0.0002);
  EXPECT_TRUE(check().allowed);
  EXPECT_DOUBLE_EQ(state->getVariablePosition("gripper_robotiq_85_left_knuckle_joint"), 0.0);
  collision_detection::AllowedCollision::Type allowed;
  EXPECT_FALSE(
    scene->getAllowedCollisionMatrix().getEntry(
      "amr_tray", "gripper_robotiq_85_left_finger_tip_link", allowed));
  floor(0.001);
  const auto collision = check();
  EXPECT_TRUE(collision.valid);
  EXPECT_FALSE(collision.allowed);
  EXPECT_TRUE(collision.common.collision);
  EXPECT_FALSE(collision.common.contacts.empty());
}
TEST_F(ClosedGraspCollisionTest, CandidateWidthChangesTipGeometryInsteadOfUsingMeasuredOpening)
{
  floor(0.0002);
  EXPECT_TRUE(check(85.0).allowed);
  EXPECT_FALSE(check(70.0).allowed);
}
TEST_F(ClosedGraspCollisionTest, TargetBodyCollisionIsRejectedAndFingerContactAllowed)
{
  EXPECT_TRUE(check(85.0, Eigen::Vector3d(0.05, 0.0, 0.0)).allowed);
  for (auto mode : kModes) {
    const auto body = check(85.0, Eigen::Vector3d(0.0, 0.0, 0.3), mode);
    EXPECT_FALSE(body.allowed);
    EXPECT_TRUE(body.common.collision);
    EXPECT_FALSE(body.common.contacts.empty());
  }
}
TEST_F(ClosedGraspCollisionTest, OtherFloorObjectsAndBodyFloorContactsStayForbidden)
{
  floor(0.0002, "other_floor");
  EXPECT_FALSE(check().allowed);
  scene->getWorldNonConst()->removeObject("other_floor");
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation().z() = 0.3;
  scene->getWorldNonConst()->addToObject(
    "amr_tray",
    shapes::ShapeConstPtr(new shapes::Box(0.02, 0.02, 0.02)), pose);
  EXPECT_FALSE(check().allowed);
}
}  // namespace
}  // namespace arm_cell_motion
