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

TEST(TaughtMotionPath, PreservesIntermediateKnotsAndRejectsWrongStart)
{
  using namespace arm_cell_motion;
  const std::vector<std::string> names{"a", "b"};
  const std::vector<JointMotionLimit> limits(2, {-2.,2.,2.,4.,true});
  const std::vector<std::vector<double>> path{{0.,0.},{.2,-.2},{.4,0.}};
  auto result = make_taught_path_trajectory(names, {0.,0.}, path, limits, 4., .5, .5);
  ASSERT_TRUE(result.valid) << result.reason;
  const auto & middle = result.trajectory.points[result.trajectory.points.size()/2];
  EXPECT_NEAR(middle.positions[0], .2, 1e-8);
  EXPECT_NEAR(middle.positions[1], -.2, 1e-8);
  EXPECT_FALSE(make_taught_path_trajectory(names, {.1,0.}, path, limits, 4., .5,.5).valid);
  EXPECT_FALSE(make_taught_path_trajectory(names, {0.,0.}, path, limits, .01,.5,.5).valid);
}


TEST(TaughtTrajectoryTiming, SlowsTimedFieldsWithoutChangingThePlannedPath)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint"};
  for (int i = 0; i < 3; ++i) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {i == 2 ? 1.5 : double(i)};
    point.velocities = {i == 1 ? 2. : 0.};
    point.accelerations = {i == 1 ? 8. : 0.};
    point.time_from_start.sec = i;
    trajectory.points.push_back(point);
  }
  const auto original = trajectory;
  ASSERT_TRUE(arm_cell_motion::stretch_taught_trajectory_to_limits(trajectory, {.5}, {.5}));
  EXPECT_EQ(trajectory.joint_names, original.joint_names);
  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    EXPECT_EQ(trajectory.points[i].positions, original.points[i].positions);
    EXPECT_LE(std::abs(trajectory.points[i].velocities[0]), .5 + 1e-9);
    EXPECT_LE(std::abs(trajectory.points[i].accelerations[0]), .5 + 1e-9);
  }
  EXPECT_NEAR(seconds(trajectory.points.back().time_from_start), 8., 1e-6);
}

TEST(TaughtTrajectoryTiming, AlsoBoundsTheExecutionSamplerSegmentVelocity)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint"};
  trajectory.points.resize(2);
  for (auto & point : trajectory.points) {
    point.positions = {0.}; point.velocities = {0.}; point.accelerations = {0.};
  }
  trajectory.points.back().positions = {1.};
  trajectory.points.back().time_from_start.nanosec = 100000000;
  ASSERT_TRUE(arm_cell_motion::stretch_taught_trajectory_to_limits(trajectory, {.5}, {.5}));
  EXPECT_NEAR(seconds(trajectory.points.back().time_from_start), 2., 1e-6);
}


TEST(TaughtTrajectoryTiming, RejectsInvalidTimingOrLimitsWithoutChangingTheTrajectory)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint"};
  trajectory.points.resize(2);
  for (auto & point : trajectory.points) {
    point.positions = {0.}; point.velocities = {0.}; point.accelerations = {0.};
  }
  const auto original = trajectory;
  EXPECT_FALSE(stretch_taught_trajectory_to_limits(trajectory, {1.}, {1.}));
  EXPECT_EQ(trajectory, original);
  trajectory.points.back().time_from_start.sec = 1;
  const auto timed = trajectory;
  EXPECT_FALSE(stretch_taught_trajectory_to_limits(trajectory, {0.}, {1.}));
  EXPECT_EQ(trajectory, timed);
  EXPECT_FALSE(stretch_taught_trajectory_to_limits(trajectory, {1.}, {NAN}));
  EXPECT_EQ(trajectory, timed);
  EXPECT_TRUE(stretch_taught_trajectory_to_limits(trajectory, {1.}, {1.}));
  EXPECT_EQ(trajectory, timed);
}


TEST(TaughtTrajectoryTiming, NanosecondRoundingCannotShortenAnIntermediateSegment)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint"};
  for (int i = 0; i < 3; ++i) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {1.5e-9 * i};
    point.velocities = {0.}; point.accelerations = {0.};
    point.time_from_start.nanosec = i;
    trajectory.points.push_back(point);
  }
  ASSERT_TRUE(stretch_taught_trajectory_to_limits(trajectory, {1.}, {1.}));
  LinearTrajectorySampler sampler({1.});
  TimedJointSample sample;
  EXPECT_TRUE(sampler.sample_timed(trajectory, 2e-9, sample).valid);
}
