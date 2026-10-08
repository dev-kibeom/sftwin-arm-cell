#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "arm_cell_motion_moveit2/trajectory_execution_worker.hpp"
#include "arm_cell_motion_moveit2/trajectory_sampler.hpp"

namespace arm_cell_motion_moveit2
{
namespace
{
trajectory_msgs::msg::JointTrajectoryPoint point(
  double time, std::initializer_list<double> positions)
{
  trajectory_msgs::msg::JointTrajectoryPoint result;
  result.time_from_start.sec = static_cast<int32_t>(time);
  result.time_from_start.nanosec = static_cast<uint32_t>((time - std::floor(time)) * 1e9);
  result.positions = positions;
  return result;
}

trajectory_msgs::msg::JointTrajectory trajectory()
{
  trajectory_msgs::msg::JointTrajectory result;
  result.joint_names = {"joint_1", "joint_2"};
  result.points = {point(0.0, {0.0, 1.0}), point(2.0, {2.0, 3.0})};
  return result;
}

TEST(TrajectorySampler, PreservesEndpointsAndInterpolatesMidpoint)
{
  LinearTrajectorySampler sampler;
  std::vector<double> sampled;
  std::size_t segment = 0;

  ASSERT_TRUE(sampler.sample(trajectory(), 0.0, sampled, segment).valid);
  EXPECT_EQ(sampled, (std::vector<double>{0.0, 1.0}));
  ASSERT_TRUE(sampler.sample(trajectory(), 1.0, sampled, segment).valid);
  EXPECT_EQ(sampled, (std::vector<double>{1.0, 2.0}));
  ASSERT_TRUE(sampler.sample(trajectory(), 2.0, sampled, segment).valid);
  EXPECT_EQ(sampled, (std::vector<double>{2.0, 3.0}));
}

TEST(TrajectorySampler, HoldsFirstAndFinalPointOutsideTrajectory)
{
  LinearTrajectorySampler sampler;
  std::vector<double> sampled;
  std::size_t segment = 0;

  ASSERT_TRUE(sampler.sample(trajectory(), -1.0, sampled, segment).valid);
  EXPECT_EQ(sampled, (std::vector<double>{0.0, 1.0}));
  ASSERT_TRUE(sampler.sample(trajectory(), 3.0, sampled, segment).valid);
  EXPECT_EQ(sampled, (std::vector<double>{2.0, 3.0}));
}

TEST(TrajectorySampler, RejectsMalformedOrNonMonotonicTiming)
{
  LinearTrajectorySampler sampler;
  auto malformed = trajectory();
  malformed.points[1].time_from_start = malformed.points[0].time_from_start;
  std::vector<double> sampled;
  std::size_t segment = 0;

  const auto result = sampler.sample(malformed, 0.5, sampled, segment);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.reason.find("strictly increasing"), std::string::npos);
}

TEST(TrajectorySampler, RepeatedSamplingIsDeterministic)
{
  LinearTrajectorySampler sampler;
  std::vector<double> first;
  std::vector<double> second;
  std::size_t first_segment = 0;
  std::size_t second_segment = 0;

  ASSERT_TRUE(sampler.sample(trajectory(), 0.75, first, first_segment).valid);
  ASSERT_TRUE(sampler.sample(trajectory(), 0.75, second, second_segment).valid);
  EXPECT_EQ(first, second);
  EXPECT_EQ(first_segment, second_segment);
}

TEST(TrajectorySampler, DerivesVelocityFromTheSameLinearPositionPath)
{
  LinearTrajectorySampler sampler;
  TimedJointSample sampled;

  ASSERT_TRUE(sampler.sample_timed(trajectory(), 1.0, sampled).valid);
  EXPECT_EQ(sampled.positions, (std::vector<double>{1.0, 2.0}));
  EXPECT_EQ(sampled.velocities, (std::vector<double>{1.0, 1.0}));
  EXPECT_EQ(sampled.segment_index, 0u);
  EXPECT_DOUBLE_EQ(sampled.trajectory_time, 1.0);
}

TEST(TrajectorySampler, FinalSampleCommandsZeroVelocity)
{
  LinearTrajectorySampler sampler;
  TimedJointSample sampled;

  ASSERT_TRUE(sampler.sample_timed(trajectory(), 2.0, sampled).valid);
  EXPECT_EQ(sampled.positions, (std::vector<double>{2.0, 3.0}));
  EXPECT_EQ(sampled.velocities, (std::vector<double>{0.0, 0.0}));
}

TEST(TrajectorySampler, RejectsDerivedVelocityOutsideConfiguredLimits)
{
  LinearTrajectorySampler sampler({0.5, 2.0});
  TimedJointSample sampled;

  const auto result = sampler.sample_timed(trajectory(), 0.5, sampled);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.reason.find("velocity limit"), std::string::npos);
}

TEST(TrajectorySampler, UnsupportedNameIsUnavailable)
{
  EXPECT_EQ(make_trajectory_sampler("ruckig"), nullptr);
}

TEST(TrajectoryExecutionWorker, RejectsBackwardSimulationClock)
{
  std::vector<std::vector<double>> commands;
  auto sampler = std::make_shared<LinearTrajectorySampler>();
  TrajectoryExecutionWorker worker(
    sampler, [&commands](const std::vector<double> & command) {
      commands.push_back(command);
      return true;
    });

  ASSERT_TRUE(worker.start(trajectory(), 7, 10.0).valid);
  ASSERT_TRUE(worker.on_simulation_time(10.5).valid);
  const auto result = worker.on_simulation_time(9.5);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(worker.failed());
  EXPECT_NE(result.reason.find("backward"), std::string::npos);
}

TEST(TrajectoryExecutionWorker, ReachesFinalPointWithoutFinalPositionOnlyShortcut)
{
  std::vector<std::vector<double>> commands;
  auto sampler = std::make_shared<LinearTrajectorySampler>();
  TrajectoryExecutionWorker worker(
    sampler, [&commands](const std::vector<double> & command) {
      commands.push_back(command);
      return true;
    });

  ASSERT_TRUE(worker.start(trajectory(), 11, 20.0).valid);
  ASSERT_TRUE(worker.on_simulation_time(20.0).valid);
  ASSERT_TRUE(worker.on_simulation_time(21.0).valid);
  worker.update_actual_positions({1.0, 2.0});
  EXPECT_FALSE(worker.completion_eligible());
  ASSERT_TRUE(worker.on_simulation_time(22.0).valid);
  worker.update_actual_positions({2.0, 3.0});
  ASSERT_FALSE(commands.empty());
  EXPECT_EQ(commands.front(), (std::vector<double>{0.0, 1.0}));
  EXPECT_EQ(commands.back(), (std::vector<double>{2.0, 3.0}));
  EXPECT_EQ(commands.size(), 3u);
  EXPECT_FALSE(worker.failed());
  EXPECT_TRUE(worker.finished());
  EXPECT_TRUE(worker.completion_eligible());
  EXPECT_EQ(worker.diagnostics().execution_id, 11u);
  EXPECT_EQ(worker.diagnostics().point_count, 2u);
  EXPECT_DOUBLE_EQ(worker.diagnostics().trajectory_time, 2.0);
}

TEST(TrajectoryExecutionWorker, PropagatesPositionAndDerivedVelocityTogether)
{
  std::vector<TimedJointSample> commands;
  auto sampler = std::make_shared<LinearTrajectorySampler>();
  TrajectoryExecutionWorker worker(
    sampler, [&commands](const TimedJointSample & command) {
      commands.push_back(command);
      return true;
    });

  ASSERT_TRUE(worker.start(trajectory(), 15, 60.0).valid);
  ASSERT_TRUE(worker.on_simulation_time(61.0).valid);
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands.front().positions, (std::vector<double>{1.0, 2.0}));
  EXPECT_EQ(commands.front().velocities, (std::vector<double>{1.0, 1.0}));
}

TEST(TrajectoryExecutionWorker, RejectsAnySegmentOutsideConfiguredVelocityLimitsAtStart)
{
  auto sampler = std::make_shared<LinearTrajectorySampler>();
  TrajectoryExecutionWorker worker(
    sampler, [](const std::vector<double> &) {return true;});
  worker.set_velocity_limits({0.5, 2.0});

  const auto result = worker.start(trajectory(), 16, 70.0);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.reason.find("velocity limit"), std::string::npos);
  EXPECT_FALSE(worker.active());
}

TEST(TrajectoryExecutionWorker, ExecutionCompletesWithObservableSmallResidual)
{
  auto sampler = std::make_shared<LinearTrajectorySampler>();
  TrajectoryExecutionWorker worker(
    sampler, [](const std::vector<double> &) {return true;});

  ASSERT_TRUE(worker.start(trajectory(), 14, 50.0).valid);
  ASSERT_TRUE(worker.on_simulation_time(52.0).valid);
  worker.update_actual_positions({2.0133, 3.0});

  EXPECT_TRUE(worker.completion_eligible());
  EXPECT_TRUE(worker.finished());
  EXPECT_TRUE(worker.diagnostics().execution_complete);
  EXPECT_FALSE(worker.diagnostics().final_converged);
  EXPECT_NEAR(worker.diagnostics().max_tracking_error, 0.0133, 1e-9);
}

TEST(TrajectoryExecutionWorker, PlaceSpecificToleranceCanAcceptConfiguredResidual)
{
  auto sampler = std::make_shared<LinearTrajectorySampler>();
  TrajectoryExecutionWorker worker(
    sampler, [](const std::vector<double> &) {return true;});
  worker.set_convergence_tolerance(0.015);

  ASSERT_TRUE(worker.start(trajectory(), 18, 80.0).valid);
  ASSERT_TRUE(worker.on_simulation_time(82.0).valid);
  worker.update_actual_positions({2.0133, 3.0});

  EXPECT_TRUE(worker.completion_eligible());
  EXPECT_TRUE(worker.diagnostics().final_converged);
}

TEST(TrajectoryExecutionWorker, SafetyPreemptStopsFutureTrajectoryCommands)
{
  std::vector<std::vector<double>> commands;
  auto sampler = std::make_shared<LinearTrajectorySampler>();
  TrajectoryExecutionWorker worker(
    sampler, [&commands](const std::vector<double> & command) {
      commands.push_back(command);
      return true;
    });

  ASSERT_TRUE(worker.start(trajectory(), 12, 30.0).valid);
  ASSERT_TRUE(worker.on_simulation_time(30.5).valid);
  const auto command_count_before_preempt = commands.size();
  worker.cancel();
  ASSERT_FALSE(worker.active());
  ASSERT_TRUE(worker.on_simulation_time(31.5).valid);
  EXPECT_EQ(commands.size(), command_count_before_preempt);
}

TEST(TrajectoryExecutionWorker, CommandAuthorityIsPublishedBeforeSinkFeedback)
{
  std::vector<std::vector<double>> commands;
  std::shared_ptr<TrajectoryExecutionWorker> worker;
  auto sampler = std::make_shared<LinearTrajectorySampler>();
  worker = std::make_shared<TrajectoryExecutionWorker>(
    sampler, [&commands, &worker](const std::vector<double> & command) {
      commands.push_back(command);
      if (command == std::vector<double>{2.0, 3.0}) {
        worker->update_actual_positions(command);
      }
      return true;
    });

  ASSERT_TRUE(worker->start(trajectory(), 13, 40.0).valid);
  ASSERT_TRUE(worker->on_simulation_time(42.0).valid);
  EXPECT_TRUE(worker->completion_eligible());
  EXPECT_EQ(worker->diagnostics().commanded_positions, (std::vector<double>{2.0, 3.0}));
  EXPECT_EQ(worker->diagnostics().actual_positions, (std::vector<double>{2.0, 3.0}));
}
TEST(TrajectoryExecutionWorker, RecordsCommandAndFeedbackTelemetry)
{
  std::vector<std::vector<double>> commands;
  auto sampler = std::make_shared<LinearTrajectorySampler>();
  TrajectoryExecutionWorker worker(
    sampler, [&commands](const std::vector<double> & command) {
      commands.push_back(command);
      return true;
    });

  ASSERT_TRUE(worker.start(trajectory(), 21, 100.0).valid);
  ASSERT_TRUE(worker.on_simulation_time(100.0).valid);
  ASSERT_TRUE(worker.on_simulation_time(101.0).valid);
  worker.update_actual_positions({0.0, 1.0}, 100.0);
  worker.update_actual_positions({0.25, 1.25}, 101.0);

  const auto diagnostics = worker.diagnostics();
  EXPECT_EQ(diagnostics.execution_id, 21u);
  EXPECT_EQ(diagnostics.simulation_callback_count, 2u);
  EXPECT_EQ(diagnostics.sample_count, 2u);
  EXPECT_EQ(diagnostics.command_publish_attempt_count, 2u);
  EXPECT_EQ(diagnostics.command_publish_success_count, 2u);
  ASSERT_EQ(diagnostics.commanded_delta.size(), 2u);
  EXPECT_DOUBLE_EQ(diagnostics.commanded_delta[0], 1.0);
  EXPECT_DOUBLE_EQ(diagnostics.commanded_delta[1], 1.0);
  EXPECT_DOUBLE_EQ(diagnostics.commanded_velocity[0], 1.0);
  EXPECT_DOUBLE_EQ(diagnostics.commanded_velocity[1], 1.0);
  ASSERT_EQ(diagnostics.actual_delta.size(), 2u);
  EXPECT_DOUBLE_EQ(diagnostics.actual_delta[0], 0.25);
  EXPECT_DOUBLE_EQ(diagnostics.actual_delta[1], 0.25);
  EXPECT_DOUBLE_EQ(diagnostics.actual_velocity[0], 0.25);
  EXPECT_DOUBLE_EQ(diagnostics.actual_velocity[1], 0.25);
  EXPECT_DOUBLE_EQ(diagnostics.actual_simulation_time, 101.0);
  EXPECT_DOUBLE_EQ(diagnostics.tracking_error[0], 0.75);
  EXPECT_DOUBLE_EQ(diagnostics.tracking_error[1], 0.75);
}

TEST(TrajectoryExecutionWorker, RecordsFinalTargetLifecycle)
{
  auto sampler = std::make_shared<LinearTrajectorySampler>();
  TrajectoryExecutionWorker worker(
    sampler, [](const std::vector<double> &) {return true;});

  ASSERT_TRUE(worker.start(trajectory(), 22, 200.0).valid);
  ASSERT_TRUE(worker.on_simulation_time(200.0).valid);
  ASSERT_TRUE(worker.on_simulation_time(202.0).valid);
  auto diagnostics = worker.diagnostics();
  EXPECT_EQ(diagnostics.final_sample_publish_count, 1u);
  EXPECT_DOUBLE_EQ(diagnostics.final_sample_first_published_simulation_time, 202.0);
  EXPECT_TRUE(diagnostics.final_target_active);
  EXPECT_FALSE(diagnostics.final_target_changed_after_publish);

  worker.update_actual_positions({2.0, 3.0}, 203.0);
  diagnostics = worker.diagnostics();
  EXPECT_FALSE(diagnostics.final_target_active);
  EXPECT_DOUBLE_EQ(diagnostics.final_target_active_duration, 1.0);
}

}  // namespace
}  // namespace arm_cell_motion_moveit2
