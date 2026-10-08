#include <gtest/gtest.h>

#include <vector>
#include <utility>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "arm_cell_motion_moveit2/trajectory_validation.hpp"

namespace arm_cell_motion_moveit2
{
namespace
{

trajectory_msgs::msg::JointTrajectoryPoint point(
  int seconds, std::vector<double> positions, std::vector<double> velocities,
  std::vector<double> accelerations)
{
  trajectory_msgs::msg::JointTrajectoryPoint result;
  result.time_from_start.sec = seconds;
  result.positions = std::move(positions);
  result.velocities = std::move(velocities);
  result.accelerations = std::move(accelerations);
  return result;
}

TEST(TrajectoryValidation, AcceptsFiniteMonotonicTimedTrajectoryWithinLimits)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint_1", "joint_2"};
  trajectory.points = {
    point(0, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}),
    point(1, {0.2, 0.1}, {0.2, 0.1}, {0.5, 0.4}),
    point(2, {0.4, 0.2}, {0.0, 0.0}, {-0.5, -0.4})};

  const auto result = validate_time_parameterized_trajectory(trajectory, {1.0, 1.0}, 1.0);

  EXPECT_TRUE(result.valid) << result.reason;
}

TEST(TrajectoryValidation, RejectsMissingTimingOrAcceleration)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint_1"};
  trajectory.points = {
    point(0, {0.0}, {0.0}, {0.0}),
    point(0, {0.1}, {0.1}, {0.1})};

  const auto result = validate_time_parameterized_trajectory(trajectory, {1.0}, 1.0);

  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.reason.find("time_from_start"), std::string::npos);
}

TEST(TrajectoryValidation, RejectsVelocityAndAccelerationLimitViolations)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint_1"};
  trajectory.points = {
    point(0, {0.0}, {0.0}, {0.0}),
    point(1, {0.1}, {1.1}, {1.1})};

  const auto result = validate_time_parameterized_trajectory(trajectory, {1.0}, 1.0);

  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.reason.find("limit"), std::string::npos);
}

TEST(TrajectoryValidation, AcceptsExactConfiguredJointOrder)
{
  EXPECT_TRUE(
    validate_trajectory_joint_order(
      {"joint_1", "joint_2", "joint_3"},
      {"joint_1", "joint_2", "joint_3"}).valid);
}

TEST(TrajectoryValidation, RejectsPermutedJointOrder)
{
  const auto result = validate_trajectory_joint_order(
    {"joint_2", "joint_1", "joint_3"},
    {"joint_1", "joint_2", "joint_3"});
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.reason.find("joint order"), std::string::npos);
}

TEST(TrajectoryValidation, RejectsMissingJoint)
{
  EXPECT_FALSE(
    validate_trajectory_joint_order(
      {"joint_1", "joint_2"}, {"joint_1", "joint_2", "joint_3"}).valid);
}

TEST(TrajectoryValidation, RejectsUnexpectedAdditionalJoint)
{
  EXPECT_FALSE(
    validate_trajectory_joint_order(
      {"joint_1", "joint_2", "joint_3", "joint_4"},
      {"joint_1", "joint_2", "joint_3"}).valid);
}

}  // namespace
}  // namespace arm_cell_motion_moveit2
