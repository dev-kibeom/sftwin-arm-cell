#include <gtest/gtest.h>
#include <limits>
#include "arm_cell_motion/taught_motion_trajectory.hpp"

using namespace arm_cell_motion;

TEST(TaughtTrajectory, PreservesPathAndAppliesIndependentVelocityAndAccelerationLimits)
{
  trajectory_msgs::msg::JointTrajectory original;
  original.joint_names = {"joint_1"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {0.5};
  point.velocities = {1.0};
  point.accelerations = {2.0};
  point.time_from_start.sec = 1;
  original.points = {point};
  for (const auto & scales : {std::pair<double, double>{0.25, 1.0}, {1.0, 0.0625}, {0.5, 0.25}}) {
    auto trajectory = original;
    ASSERT_TRUE(scale_taught_trajectory(trajectory, scales.first, scales.second));
    EXPECT_EQ(trajectory.points[0].positions, original.points[0].positions);
    EXPECT_LE(trajectory.points[0].velocities[0], scales.first);
    EXPECT_LE(trajectory.points[0].accelerations[0], 2.0 * scales.second);
    EXPECT_GT(trajectory.points[0].time_from_start.sec, 1);
  }
}

TEST(TaughtTrajectory, RejectsInvalidAndUnrepresentableTimingWithoutChangingInput)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start.sec = 2;
  trajectory.points = {point};
  for (const auto scale : {0.0, -0.5, 1.1, std::numeric_limits<double>::infinity(), 1e-30}) {
    EXPECT_FALSE(scale_taught_trajectory(trajectory, scale, 0.2));
    EXPECT_EQ(trajectory.points[0].time_from_start.sec, 2);
  }
}
