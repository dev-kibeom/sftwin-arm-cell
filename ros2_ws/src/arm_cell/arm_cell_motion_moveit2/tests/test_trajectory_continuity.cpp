#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "arm_cell_motion_moveit2/trajectory_continuity.hpp"

namespace arm_cell_motion_moveit2
{

namespace
{
JointContinuityLimit revolute_limit()
{
  return JointContinuityLimit{-6.2832, 6.2832, true};
}

JointContinuityLimit narrow_revolute_limit()
{
  return JointContinuityLimit{-1.0, 1.0, true};
}
}

TEST(TrajectoryContinuityTest, ResolvesEquivalentEndpointToNearestMeasuredState)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint_6"};
  trajectory.points.resize(2);
  trajectory.points[0].positions = {-4.16802};
  trajectory.points[1].positions = {2.11579};

  const auto result = normalize_joint_trajectory(
    trajectory, {-4.16802}, {revolute_limit()});

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(trajectory.points.back().positions.size(), 1U);
  EXPECT_NEAR(trajectory.points.back().positions.front() - (-4.16802), 0.000625, 0.001);
  EXPECT_NEAR(result.raw_endpoint_delta.front(), 6.28381, 0.001);
  EXPECT_NEAR(result.wrap_aware_endpoint_delta.front(), 0.000625, 0.001);
  EXPECT_TRUE(result.normalized);
  ASSERT_EQ(result.normalized_joints.size(), 1U);
  EXPECT_TRUE(result.normalized_joints.front());
}

TEST(TrajectoryContinuityTest, RejectsUnresolvableContinuousJointEndpoint)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint_6"};
  trajectory.points.resize(2);
  trajectory.points[0].positions = {0.0};
  trajectory.points[1].positions = {10.0};

  const auto result = normalize_joint_trajectory(
    trajectory, {0.0}, {narrow_revolute_limit()});

  EXPECT_FALSE(result.valid);
}

TEST(TrajectoryContinuityTest, RejectsSmoothAccumulatedEquivalentFullTurn)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint_6"};
  for (int index = 0; index <= 12; ++index) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {-4.16802 + (6.28381 * static_cast<double>(index) / 12.0)};
    trajectory.points.push_back(point);
  }

  const auto result = normalize_joint_trajectory(
    trajectory, {-4.16802}, {revolute_limit()}, false);

  EXPECT_FALSE(result.valid);
}

TEST(TrajectoryContinuityTest, RejectsDirectEquivalentEndpointJumpWithoutPostProcessing)
{
  for (const auto & values : std::vector<std::pair<double, double>>{
      std::make_pair(-4.16802, 2.11579), std::make_pair(2.11579, -4.16802)})
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = {"joint_6"};
    trajectory.points.resize(2);
    trajectory.points[0].positions = {values.first};
    trajectory.points[1].positions = {values.second};

    const auto result = normalize_joint_trajectory(
      trajectory, {values.first}, {revolute_limit()}, false);

    EXPECT_FALSE(result.valid);
  }
}

TEST(TrajectoryContinuityTest, AcceptsLegitimateShortContinuousMotion)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint_6"};
  trajectory.points.resize(3);
  trajectory.points[0].positions = {-4.16802};
  trajectory.points[1].positions = {-4.10};
  trajectory.points[2].positions = {-4.00};

  const auto result = normalize_joint_trajectory(
    trajectory, {-4.16802}, {revolute_limit()});

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.normalized);
  EXPECT_NEAR(trajectory.points.back().positions.front(), -4.00, 1e-9);
}

TEST(TrajectoryContinuityTest, BoundedRevoluteDoesNotUseWrapEquivalence)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint_6"};
  trajectory.points.resize(2);
  trajectory.points[0].positions = {-0.2};
  trajectory.points[1].positions = {0.2};

  const auto result = normalize_joint_trajectory(
    trajectory, {-0.2}, {JointContinuityLimit{-1.0, 1.0, false}});

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.normalized);
  EXPECT_NEAR(trajectory.points.back().positions.front(), 0.2, 1e-9);
}

TEST(TrajectoryContinuityTest, SelectsNearestLegalEquivalentForJoint6)
{
  const auto result = select_nearest_joint_goal(
    {2.11579}, {-4.16802}, {revolute_limit()});

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.selected_goal.size(), 1U);
  EXPECT_NEAR(result.selected_goal.front(), -4.167395, 0.001);
  EXPECT_NEAR(result.raw_endpoint_delta.front(), 6.28381, 0.001);
  EXPECT_NEAR(result.wrap_aware_endpoint_delta.front(), 0.000625, 0.001);
  EXPECT_FALSE(result.unnecessary_full_turn);
}

TEST(TrajectoryContinuityTest, SelectsNearestLegalEquivalentForJoint1)
{
  const auto result = select_nearest_joint_goal(
    {2.11579}, {-4.16802}, {revolute_limit()});

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.selected_goal.front() - (-4.16802), 0.000625, 0.001);
}

TEST(TrajectoryContinuityTest, DoesNotAlterNonWrapEquivalentGoal)
{
  const auto result = select_nearest_joint_goal(
    {0.2}, {-0.2}, {JointContinuityLimit{-1.0, 1.0, false}});

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.selected_goal.front(), 0.2);
}

TEST(TrajectoryContinuityTest, RejectsCandidateOutsideLegalBounds)
{
  const auto result = select_nearest_joint_goal(
    {2.0}, {0.0}, {JointContinuityLimit{-1.0, 1.0, false}});

  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.reason.find("bounds"), std::string::npos);
}

TEST(TrajectoryContinuityTest, RejectsCandidateWhenStateValidityFails)
{
  const auto result = select_nearest_joint_goal(
    {0.2}, {-0.2}, {JointContinuityLimit{-1.0, 1.0, false}},
    [](const std::vector<double> &, std::string & reason) {
      reason = "collision";
      return false;
    });

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "collision");
}

TEST(TrajectoryContinuityTest, PlannedEndpointMatchesSelectedNearGoal)
{
  const auto selected_goal = select_nearest_joint_goal(
    {2.11579}, {-4.16802}, {revolute_limit()});
  ASSERT_TRUE(selected_goal.valid);

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint_1"};
  trajectory.points.resize(2);
  trajectory.points[0].positions = {-4.16802};
  trajectory.points[1].positions = selected_goal.selected_goal;

  const auto continuity = normalize_joint_trajectory(
    trajectory, {-4.16802}, {revolute_limit()}, false);
  ASSERT_TRUE(continuity.valid);
  EXPECT_NEAR(
    trajectory.points.back().positions.front(), selected_goal.selected_goal.front(), 1e-9);
}

}  // namespace arm_cell_motion_moveit2
