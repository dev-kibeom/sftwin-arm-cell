#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "arm_cell_motion/taught_motion_trajectory.hpp"
#include "arm_cell_motion/trajectory_sampler.hpp"
#include "arm_cell_motion/trajectory_execution_worker.hpp"

using namespace arm_cell_motion;

namespace
{
double seconds(const builtin_interfaces::msg::Duration & value)
{return value.sec + value.nanosec * 1e-9;}
}

TEST(TaughtTrajectory, GeneratesBoundedSynchronizedJointMotionWithoutPlanning)
{
  const std::vector<JointMotionLimit> limits{{-3, 3, 1, 2}, {-3, 3, 2, 3}};
  const std::vector<double> start{0.1, 0.5}, target{1.0, -0.2};
  const auto result = make_taught_joint_trajectory({"one", "two"}, start, target, limits, 0.4, 0.3);
  ASSERT_TRUE(result.valid) << result.reason;
  const auto & trajectory = result.trajectory;
  ASSERT_FALSE(trajectory.points.empty());
  EXPECT_EQ(trajectory.points.front().positions, start);
  EXPECT_EQ(trajectory.points.back().positions, target);
  EXPECT_EQ(trajectory.points.back().velocities, (std::vector<double>{0, 0}));
  LinearTrajectorySampler sampler;
  double previous = -1;
  for (const auto & point : trajectory.points) {
    const auto time = seconds(point.time_from_start);
    EXPECT_GT(time, previous);
    previous = time;
    for (std::size_t joint = 0; joint < limits.size(); ++joint) {
      EXPECT_LE(std::abs(point.velocities[joint]), limits[joint].max_velocity * 0.4 + 1e-9);
      EXPECT_LE(std::abs(point.accelerations[joint]), limits[joint].max_acceleration * 0.3 + 1e-9);
      EXPECT_GE(point.positions[joint], std::min(start[joint], target[joint]) - 1e-9);
      EXPECT_LE(point.positions[joint], std::max(start[joint], target[joint]) + 1e-9);
    }
    TimedJointSample sample;
    ASSERT_TRUE(sampler.sample_timed(trajectory, time, sample).valid);
    for (std::size_t joint = 0; joint < limits.size(); ++joint) {
      EXPECT_LE(std::abs(sample.velocities[joint]), limits[joint].max_velocity * 0.4 + 1e-6);
    }
  }
}

TEST(TaughtTrajectory, ReducingEitherLimitSlowsTheSameJointPath)
{
  const std::vector<JointMotionLimit> limits{{-3, 3, 1, 1}};
  const auto fast = make_taught_joint_trajectory({"joint"}, {0}, {2}, limits, 1, 1);
  const auto slow_velocity = make_taught_joint_trajectory({"joint"}, {0}, {2}, limits, 0.2, 1);
  const auto slow_acceleration = make_taught_joint_trajectory({"joint"}, {0}, {2}, limits, 1, 0.1);
  ASSERT_TRUE(fast.valid);
  ASSERT_TRUE(slow_velocity.valid);
  ASSERT_TRUE(slow_acceleration.valid);
  EXPECT_GT(
    seconds(slow_velocity.trajectory.points.back().time_from_start),
    seconds(fast.trajectory.points.back().time_from_start));
  EXPECT_GT(
    seconds(slow_acceleration.trajectory.points.back().time_from_start),
    seconds(fast.trajectory.points.back().time_from_start));
}

TEST(TaughtTrajectory, HandlesAlreadyReachedTargetsAndRejectsInvalidInputs)
{
  const std::vector<JointMotionLimit> limits{{-1, 1, 1, 1}};
  const auto stationary = make_taught_joint_trajectory({"joint"}, {0.3}, {0.3}, limits, 1, 1);
  ASSERT_TRUE(stationary.valid);
  EXPECT_GT(seconds(stationary.trajectory.points.back().time_from_start), 0);
  EXPECT_EQ(stationary.trajectory.points.back().positions, (std::vector<double>{0.3}));
  EXPECT_FALSE(make_taught_joint_trajectory({"joint"}, {0}, {2}, limits, 1, 1).valid);
  EXPECT_FALSE(make_taught_joint_trajectory({"joint"}, {2}, {0}, limits, 1, 1).valid);
  EXPECT_FALSE(make_taught_joint_trajectory({"joint"}, {}, {0}, limits, 1, 1).valid);
  for (const auto scale : {0.0, -0.5, 1.1, std::numeric_limits<double>::infinity(), 1e-30}) {
    EXPECT_FALSE(make_taught_joint_trajectory({"joint"}, {0}, {0.5}, limits, scale, 1).valid);
  }
}

TEST(TaughtTrajectory, ReadsPhysicalJointLimitsWithoutMoveGroup)
{
  const auto limits = joint_motion_limits_from_urdf(
    R"(
    <robot name="test"><link name="base"/><link name="tip"/>
    <joint name="joint" type="revolute"><parent link="base"/><child link="tip"/>
    <limit lower="-1" upper="2" velocity="0.7" effort="1"/></joint></robot>)", {"joint"},
    1.5);
  ASSERT_EQ(limits.size(), 1U);
  EXPECT_DOUBLE_EQ(limits.front().max_velocity, 0.7);
  EXPECT_DOUBLE_EQ(limits.front().max_acceleration, 1.5);
  EXPECT_DOUBLE_EQ(limits.front().lower, -1);
  EXPECT_DOUBLE_EQ(limits.front().upper, 2);
  EXPECT_THROW(joint_motion_limits_from_urdf("invalid", {"joint"}, 1), std::invalid_argument);
}

TEST(TaughtTrajectory, EncodedTimingObeysTheExistingWorkerVelocityLimits)
{
  const auto result = make_taught_joint_trajectory(
    {"joint"}, {0}, {0.5}, {{-1, 1, 1, 2}}, 0.4, 0.3);
  ASSERT_TRUE(result.valid);
  TrajectoryExecutionWorker worker(make_trajectory_sampler("linear"),
    [](const TimedJointSample &) {return true;});
  worker.set_velocity_limits({0.4});
  const auto started = worker.start(result.trajectory, 1, 0);
  EXPECT_TRUE(started.valid) << started.reason;
}
