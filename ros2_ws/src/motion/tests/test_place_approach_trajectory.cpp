#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/duration.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <rclcpp/rclcpp.hpp>

#include "arm_cell_motion/place_approach_ik_policy.hpp"
#include "arm_cell_motion/place_approach_trajectory.hpp"
#include "arm_cell_motion/trajectory_validation.hpp"

namespace arm_cell_motion
{
namespace
{
struct PlaceApproachTrajectoryTest : public ::testing::Test
{
  void SetUp() override
  {
    moveit::core::RobotModelBuilder builder("timing_test", "base_link");
    builder.addChain("base_link->link_1->link_2", "revolute");
    builder.addGroupChain("base_link", "link_2", "arm");
    ASSERT_TRUE(builder.isValid());
    robot_model_ = builder.build();
    ASSERT_NE(robot_model_, nullptr);
    for (const auto & joint_name : robot_model_->getJointModelGroup("arm")->getJointModelNames()) {
      auto * joint_model = const_cast<moveit::core::JointModel *>(
        robot_model_->getJointModel(joint_name));
      moveit::core::VariableBounds bounds;
      bounds.min_velocity_ = -1.0;
      bounds.max_velocity_ = 1.0;
      bounds.velocity_bounded_ = true;
      bounds.min_acceleration_ = -1.0;
      bounds.max_acceleration_ = 1.0;
      bounds.acceleration_bounded_ = true;
      for (const auto & variable_name : joint_model->getVariableNames()) {
        joint_model->setVariableBounds(variable_name, bounds);
      }
    }
    start_state_ = std::make_unique<moveit::core::RobotState>(robot_model_);
    start_state_->setToDefaultValues();
  }

  moveit_msgs::msg::RobotTrajectory raw_trajectory(
    std::size_t point_count, bool moving = true) const
  {
    moveit_msgs::msg::RobotTrajectory trajectory;
    trajectory.joint_trajectory.joint_names =
      robot_model_->getJointModelGroup("arm")->getVariableNames();
    for (std::size_t point_index = 0; point_index < point_count; ++point_index) {
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = point_index == 0U || !moving ? std::vector<double>{0.0, 0.0} :
        std::vector<double>{0.5, -0.25};
      trajectory.joint_trajectory.points.push_back(point);
    }
    return trajectory;
  }

  PlaceApproachIkCandidate candidate(std::size_t seed_index) const
  {
    PlaceApproachIkCandidate result;
    result.seed_index = seed_index;
    result.valid = true;
    result.normalized_goal = {0.1 * static_cast<double>(seed_index + 1U)};
    return result;
  }

  moveit::core::RobotModelPtr robot_model_;
  std::unique_ptr<moveit::core::RobotState> start_state_;
};

TEST_F(PlaceApproachTrajectoryTest, TimeParameterizesRawCandidateForDownstreamValidation)
{
  auto trajectory = raw_trajectory(2U);
  ASSERT_EQ(trajectory.joint_trajectory.points.size(), 2U);
  EXPECT_TRUE(trajectory.joint_trajectory.points.front().velocities.empty());
  EXPECT_TRUE(trajectory.joint_trajectory.points.front().accelerations.empty());

  const auto timing = time_parameterize_place_approach_trajectory(
    robot_model_, *start_state_, trajectory);

  ASSERT_TRUE(timing.success) << timing.reason;
  EXPECT_GT(timing.duration_s, 0.0);
  const auto & points = trajectory.joint_trajectory.points;
  ASSERT_GE(points.size(), 2U);
  const auto time_ns = [](const builtin_interfaces::msg::Duration & duration) {
      return static_cast<std::int64_t>(duration.sec) * 1000000000LL + duration.nanosec;
    };
  EXPECT_LT(time_ns(points.front().time_from_start), time_ns(points.back().time_from_start));
  for (const auto & point : points) {
    EXPECT_EQ(point.positions.size(), trajectory.joint_trajectory.joint_names.size());
    EXPECT_EQ(point.velocities.size(), trajectory.joint_trajectory.joint_names.size());
    EXPECT_EQ(point.accelerations.size(), trajectory.joint_trajectory.joint_names.size());
  }
  const auto validation = validate_time_parameterized_trajectory(
    trajectory.joint_trajectory, {1.0, 1.0}, 10.0);
  EXPECT_TRUE(validation.valid) << validation.reason;
}

TEST(PlaceApproachMoveGroupTargetTest, ArmGroupVectorTargetUsesGroupVariableOrder)
{
  const std::string urdf = R"(
<robot name="target_test">
  <link name="base"/><link name="arm_link_1"/><link name="arm_link_2"/>
  <link name="gripper_link"/>
  <joint name="joint_1" type="revolute"><parent link="base"/><child link="arm_link_1"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
  <joint name="joint_2" type="revolute"><parent link="arm_link_1"/><child link="arm_link_2"/>
    <axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
  <joint name="finger_joint" type="revolute"><parent link="base"/><child link="gripper_link"/>
    <axis xyz="1 0 0"/><limit lower="-0.1" upper="0.1" effort="1" velocity="1"/></joint>
</robot>)";
  const std::string srdf = R"(
<robot name="target_test">
  <group name="arm"><joint name="joint_1"/><joint name="joint_2"/></group>
  <group name="gripper"><joint name="finger_joint"/></group>
</robot>)";

  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>(
    "place_move_group_target_test",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    .parameter_overrides({
      rclcpp::Parameter("robot_description", urdf),
      rclcpp::Parameter("robot_description_semantic", srdf)}));

  moveit::planning_interface::MoveGroupInterface move_group(
    node, "arm", std::shared_ptr<tf2_ros::Buffer>(), rclcpp::Duration::from_seconds(0.0));
  ASSERT_EQ(move_group.getName(), "arm");

  const auto * arm_group = move_group.getRobotModel()->getJointModelGroup("arm");
  ASSERT_NE(arm_group, nullptr);
  ASSERT_EQ(arm_group->getVariableNames(), (std::vector<std::string>{"joint_1", "joint_2"}));
  const std::vector<double> normalized_goal{0.25, -0.5};

  ASSERT_TRUE(move_group.setJointValueTarget(normalized_goal));
  std::vector<double> selected_group_goal;
  move_group.getJointValueTarget(selected_group_goal);
  EXPECT_EQ(selected_group_goal, normalized_goal);

  rclcpp::shutdown();
}

TEST_F(PlaceApproachTrajectoryTest, ZeroDurationTotgCandidateIsSkippedForNextTimedPlan)
{
  const std::vector<PlaceApproachIkCandidate> candidates{candidate(0U), candidate(1U)};
  PlaceApproachIkPolicy policy;
  policy.max_candidates = candidates.size();
  policy.per_candidate_planning_budget_s = 1.0;
  policy.total_planning_budget_s = 2.0;

  const auto result = plan_ranked_place_approach_candidates(
    candidates, {0U, 1U}, policy,
    [this](const PlaceApproachIkCandidate & candidate_value, double) {
      auto trajectory = raw_trajectory(candidate_value.seed_index == 0U ? 1U : 2U);
      const auto timing = time_parameterize_place_approach_trajectory(
        robot_model_, *start_state_, trajectory);
      const auto & points = trajectory.joint_trajectory.points;
      const auto point_count = points.size();
      return PlaceApproachPlanObservation{
        true, 1, point_count, 0.1, timing.duration_s, timing.success, timing.reason};
    });

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.selected_candidate_index, 1U);
  ASSERT_EQ(result.attempts.size(), 2U);
  EXPECT_FALSE(result.attempts[0].success);
  EXPECT_FALSE(result.attempts[0].time_parameterized);
  EXPECT_EQ(
    result.attempts[0].time_parameterization_reason,
    "time_parameterization_non_positive_duration");
  EXPECT_TRUE(result.attempts[1].success);
  EXPECT_TRUE(result.attempts[1].time_parameterized);
  EXPECT_GT(result.attempts[1].trajectory_duration_s, 0.0);
}

}  // namespace
}  // namespace arm_cell_motion
