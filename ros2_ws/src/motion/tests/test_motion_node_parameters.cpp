#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "arm_cell_motion/fake_motion_backend.hpp"
#include "arm_cell_motion/motion_node.hpp"

TEST(MotionNodeParameters, ValidatesRuntimeTuningAndRejectsStaticParameters)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    rclcpp::Parameter("pick_retract_distance_m", 0.05)});
  auto node = std::make_shared<arm_cell_motion::MotionNode>(
    std::make_shared<arm_cell_motion::FakeMotionBackend>(), options);

  EXPECT_EQ(node->get_parameter("grasp_occupancy_mode").as_string(), "FULL_COLLISION");
  EXPECT_DOUBLE_EQ(node->get_parameter("finger_clearance_radius_m").as_double(), 0.010);
  EXPECT_DOUBLE_EQ(node->get_parameter("finger_contact_span_m").as_double(), 0.020);
  EXPECT_DOUBLE_EQ(node->get_parameter("finger_clearance_depth_m").as_double(), 0.025);
  EXPECT_DOUBLE_EQ(node->get_parameter("pick_ik_timeout_s").as_double(), 1.0);
  EXPECT_DOUBLE_EQ(node->get_parameter("pick_grasp_ik_timeout_s").as_double(), 1.0);
  EXPECT_DOUBLE_EQ(node->get_parameter("pick_approach_ik_timeout_s").as_double(), 1.0);
  EXPECT_DOUBLE_EQ(node->get_parameter("pick_search_budget_s").as_double(), 100.0);
  EXPECT_DOUBLE_EQ(node->get_parameter("pick_max_withdrawal_tilt_deg").as_double(), 90.0);
  EXPECT_EQ(
    node->get_parameter("pick_grasp_pose_order_policy").as_string(), "distance_then_tilt");
  EXPECT_DOUBLE_EQ(
    node->get_parameter("pick_position_range_half_length_fraction").as_double(),
    0.5);
  EXPECT_DOUBLE_EQ(node->get_parameter("pick_max_downward_position_offset_m").as_double(), 0.005);
  EXPECT_EQ(node->get_parameter("pick_position_offset_samples_per_tangent_axis").as_int(), 3);
  EXPECT_EQ(node->get_parameter("pick_orientation_samples_per_closing_axis").as_int(), 8);
  EXPECT_EQ(node->get_parameter("pick_grasp_pose_candidate_limit").as_int(), 48);
  EXPECT_EQ(node->get_parameter("pick_ik_seed_attempts_per_grasp_pose").as_int(), 2);
  EXPECT_EQ(node->get_parameter("planning_num_attempts").as_int(), 3);
  EXPECT_EQ(node->get_parameter("place_max_pose_candidates").as_int(), 13);
  EXPECT_EQ(node->get_parameter("pick_approach_global_max_requests").as_int(), 3);
  EXPECT_EQ(node->get_parameter("pick_final_cartesian_max_requests").as_int(), 1);
  EXPECT_EQ(node->get_parameter("pick_final_global_fallback_max_requests").as_int(), 3);
  EXPECT_EQ(node->get_parameter("place_approach_global_max_requests").as_int(), 3);
  EXPECT_EQ(node->get_parameter("place_final_cartesian_max_requests").as_int(), 1);
  EXPECT_EQ(node->get_parameter("place_final_global_fallback_max_requests").as_int(), 3);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("planning_num_attempts", 4)).successful);
  EXPECT_FALSE(
    node->set_parameter(
      rclcpp::Parameter("pick_grasp_pose_candidate_limit", 4)).successful);
  EXPECT_FALSE(
    node->set_parameter(
    rclcpp::Parameter("pick_grasp_pose_order_policy", "distance_then_diversity")).successful);
  EXPECT_FALSE(
    node->set_parameter(
      rclcpp::Parameter("pick_ik_seed_attempts_per_grasp_pose", 3)).successful);

  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("planning_time_s", 20.0)).successful);
  EXPECT_FALSE(
    node->set_parameter(rclcpp::Parameter("planning_time_s", std::nan(""))).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("planning_time_s", 0.0)).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("backend_mode", "isaac")).successful);
  EXPECT_FALSE(
    node->set_parameter(
      rclcpp::Parameter(
    "pick_ik_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin")).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("pick_ik_solve_type", "Speed")).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("pick_ik_timeout_s", 0.5)).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("pick_search_budget_s", 90.0)).successful);

  node.reset();
  rclcpp::shutdown();
}

TEST(MotionNodeParameters, LoadsPlaceWaypointBoundaryDirectionAndFixedTarget)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    rclcpp::Parameter("place_waypoint_clearance_y_boundary_m", -0.926),
    rclcpp::Parameter("place_waypoint_clearance_direction", "POSITIVE_Y"),
    rclcpp::Parameter("place_waypoint_approach_distance_m", 0.015),
    rclcpp::Parameter("place_waypoint_approach_direction_object", std::vector<double>{0.0, 0.0, 1.0}),
    rclcpp::Parameter(
      "place_waypoint_target_pose_xyzw",
      std::vector<double>{0.5, 0.25, 0.3675, 0.7071067811865476, 0.0, 0.0, 0.7071067811865476})});
  auto node = std::make_shared<arm_cell_motion::MotionNode>(
    std::make_shared<arm_cell_motion::FakeMotionBackend>(), options);

  const auto & geometry = node->motion_geometry();
  EXPECT_TRUE(geometry.place_waypoint_configured);
  EXPECT_TRUE(geometry.place_waypoint_clearance_toward_positive_y);
  EXPECT_DOUBLE_EQ(geometry.place_waypoint_clearance_y_boundary_m, -0.926);
  EXPECT_DOUBLE_EQ(geometry.place_waypoint_clearance_y_margin_m, 0.05);
  EXPECT_DOUBLE_EQ(geometry.place_waypoint_approach_distance_m, 0.015);
  EXPECT_EQ(geometry.place_waypoint_target_pose.header.frame_id, "base_link");
  EXPECT_DOUBLE_EQ(geometry.place_waypoint_target_pose.pose.position.x, 0.5);
  EXPECT_DOUBLE_EQ(geometry.place_waypoint_target_pose.pose.position.y, 0.25);
  EXPECT_DOUBLE_EQ(geometry.place_waypoint_target_pose.pose.position.z, 0.3675);

  node.reset();
  rclcpp::shutdown();
}

TEST(MotionNodeParameters, RejectsPlanningFallbackLimitsOutsideSupportedRange)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  for (const auto & parameter : std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("place_max_pose_candidates", 101),
      rclcpp::Parameter("pick_approach_global_max_requests", 0),
      rclcpp::Parameter("place_final_global_fallback_max_requests", 21)})
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({parameter});
    EXPECT_THROW(
      std::make_shared<arm_cell_motion::MotionNode>(
        std::make_shared<arm_cell_motion::FakeMotionBackend>(), options),
      std::invalid_argument);
  }
  rclcpp::shutdown();
}

TEST(MotionNodeParameters, RejectsGraspPoseCapThatWouldEvictLegacyCenterGroup)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
      rclcpp::Parameter("pick_grasp_pose_candidate_limit", 2)});
  EXPECT_THROW(
    std::make_shared<arm_cell_motion::MotionNode>(
      std::make_shared<arm_cell_motion::FakeMotionBackend>(), options),
    std::invalid_argument);
  rclcpp::shutdown();
}

TEST(MotionNodeParameters, RejectsInvalidPickSearchBudgetsAndIkTimeouts)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  for (const auto & parameter : std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("pick_search_budget_s", 0.0),
      rclcpp::Parameter("pick_search_budget_s", 601.0),
      rclcpp::Parameter("pick_grasp_ik_timeout_s", 0.0),
      rclcpp::Parameter("pick_approach_ik_timeout_s", 31.0)})
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({parameter});
    EXPECT_THROW(
      std::make_shared<arm_cell_motion::MotionNode>(
        std::make_shared<arm_cell_motion::FakeMotionBackend>(), options),
      std::invalid_argument);
  }
  rclcpp::shutdown();
}

TEST(MotionNodeParameters, RejectsIkSeedAttemptLimitBelowOne)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
      rclcpp::Parameter("pick_ik_seed_attempts_per_grasp_pose", 0)});
  EXPECT_THROW(
    std::make_shared<arm_cell_motion::MotionNode>(
      std::make_shared<arm_cell_motion::FakeMotionBackend>(), options),
    std::invalid_argument);
  rclcpp::shutdown();
}

TEST(MotionNodeParameters, RejectsPlanningAttemptLimitBelowOne)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("planning_num_attempts", 0)});
  EXPECT_THROW(
    std::make_shared<arm_cell_motion::MotionNode>(
      std::make_shared<arm_cell_motion::FakeMotionBackend>(), options),
    std::invalid_argument);
  rclcpp::shutdown();
}

TEST(MotionNodeParameters, RejectsInvalidPickSearchSampling)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  for (const auto & parameter : std::vector<rclcpp::Parameter>{
    rclcpp::Parameter("pick_max_withdrawal_tilt_deg", 180.1),
    rclcpp::Parameter(
      "pick_max_withdrawal_tilt_deg", std::numeric_limits<double>::quiet_NaN()),
    rclcpp::Parameter("pick_position_range_half_length_fraction", 1.0),
    rclcpp::Parameter("pick_max_downward_position_offset_m", -0.001),
    rclcpp::Parameter(
      "pick_max_downward_position_offset_m", std::numeric_limits<double>::infinity()),
    rclcpp::Parameter("pick_position_offset_samples_per_tangent_axis", 0),
    rclcpp::Parameter("pick_orientation_samples_per_closing_axis", 73)})
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({parameter});
    EXPECT_THROW(
      std::make_shared<arm_cell_motion::MotionNode>(
        std::make_shared<arm_cell_motion::FakeMotionBackend>(), options),
      std::invalid_argument);
  }
  rclcpp::shutdown();
}

TEST(MotionNodeParameters, RejectsUnknownPickGraspPoseOrderPolicy)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    rclcpp::Parameter("pick_grasp_pose_order_policy", "tilt_then_distance")});
  EXPECT_THROW(
    std::make_shared<arm_cell_motion::MotionNode>(
      std::make_shared<arm_cell_motion::FakeMotionBackend>(), options),
    std::invalid_argument);
  rclcpp::shutdown();
}

TEST(MotionNodeParameters, GraspOccupancyProfileModesAndDimensionsAreValidatedAtStartup)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  for (const auto & mode : {"NONE", "FULL_COLLISION", "FINGER_CLEARANCE"}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("grasp_occupancy_mode", mode),
      rclcpp::Parameter("finger_clearance_radius_m", 0.015),
      rclcpp::Parameter("finger_contact_span_m", 0.030),
      rclcpp::Parameter("finger_clearance_depth_m", 0.040)});
    auto node = std::make_shared<arm_cell_motion::MotionNode>(
      std::make_shared<arm_cell_motion::FakeMotionBackend>(), options);
    EXPECT_EQ(node->get_parameter("grasp_occupancy_mode").as_string(), mode);
    EXPECT_DOUBLE_EQ(node->get_parameter("finger_clearance_radius_m").as_double(), 0.015);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("grasp_occupancy_mode", "NONE")).successful);
  }
  rclcpp::NodeOptions bad_mode;
  bad_mode.parameter_overrides({rclcpp::Parameter("grasp_occupancy_mode", "unknown")});
  EXPECT_THROW(
    std::make_shared<arm_cell_motion::MotionNode>(
    std::make_shared<arm_cell_motion::FakeMotionBackend>(), bad_mode), std::invalid_argument);
  for (const auto & name : {"finger_clearance_radius_m", "finger_contact_span_m",
      "finger_clearance_depth_m"})
  {
    for (double value : {0.0, -0.1, 1.1, std::numeric_limits<double>::infinity(), std::nan("")}) {
      rclcpp::NodeOptions options;
      options.parameter_overrides({rclcpp::Parameter(name, value)});
      EXPECT_THROW(
        std::make_shared<arm_cell_motion::MotionNode>(
        std::make_shared<arm_cell_motion::FakeMotionBackend>(), options), std::invalid_argument);
    }
  }
}
