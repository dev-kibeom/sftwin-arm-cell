#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <future>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "arm_cell_motion/trajectory_execution_worker.hpp"
#include "arm_cell_motion/trajectory_sampler.hpp"

namespace arm_cell_motion
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

TEST(TrajectoryExecutionWorker, FinalAcceptanceCompletesWithoutFeedbackAndStopsCommands)
{
  int commands = 0;
  TrajectoryExecutionWorker worker(
    std::make_shared<LinearTrajectorySampler>(),
    [&commands](const std::vector<double> &) {++commands; return true;});
  ASSERT_TRUE(worker.start(trajectory(), 1, 10.0).valid);
  ASSERT_TRUE(worker.on_simulation_time(11.0).valid);
  EXPECT_FALSE(worker.completion_eligible());
  ASSERT_TRUE(worker.on_simulation_time(12.0).valid);
  EXPECT_TRUE(worker.completion_eligible());
  EXPECT_FALSE(worker.active());
  ASSERT_TRUE(worker.on_simulation_time(13.0).valid);
  EXPECT_EQ(commands, 2);
}

TEST(TrajectoryExecutionWorker, CancelDuringFinalSinkCannotComplete)
{
  std::shared_ptr<TrajectoryExecutionWorker> worker;
  worker = std::make_shared<TrajectoryExecutionWorker>(
    std::make_shared<LinearTrajectorySampler>(),
    [&worker](const std::vector<double> &) {worker->cancel(); return true;});
  ASSERT_TRUE(worker->start(trajectory(), 1, 10.0).valid);
  ASSERT_TRUE(worker->on_simulation_time(12.0).valid);
  worker->update_actual_positions({2.0, 3.0});
  worker->update_actual_positions({2.0, 3.0});
  worker->update_actual_positions({2.0, 3.0});
  EXPECT_FALSE(worker->completion_eligible());
  EXPECT_FALSE(worker->active());
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

TEST(TrajectoryExecutionWorker, RejectedFinalReferenceIsNotReportedComplete)
{
  auto sampler = std::make_shared<LinearTrajectorySampler>();
  TrajectoryExecutionWorker worker(
    sampler, [](const std::vector<double> & command) {
      return command != std::vector<double>{2.0, 3.0};
    });

  ASSERT_TRUE(worker.start(trajectory(), 22, 90.0).valid);
  const auto result = worker.on_simulation_time(92.0);

  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(worker.failed());
  EXPECT_FALSE(worker.diagnostics().final_command_accepted);
  EXPECT_EQ(worker.diagnostics().final_command_publish_success_count, 0u);
  EXPECT_FALSE(worker.completion_eligible());
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
  worker.update_actual_positions({2.0, 3.0});
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

TEST(TrajectoryExecutionWorker, ResidualErrorAndOscillationRemainDiagnosticsOnly)
{
  TrajectoryExecutionWorker worker(
    std::make_shared<LinearTrajectorySampler>(),
    [](const std::vector<double> &) {return true;});
  worker.set_joint_target_accuracy_tolerance(0.03);
  ASSERT_TRUE(worker.start(trajectory(), 20, 90.0).valid);
  worker.update_actual_positions({0.0, 1.0}, 90.0);
  ASSERT_TRUE(worker.on_simulation_time(92.0).valid);
  EXPECT_TRUE(worker.completion_eligible());
  for (double position : {2.04, 2.05, 2.04}) {
    worker.update_actual_positions({position, 3.0}, 92.0);
    EXPECT_TRUE(worker.completion_eligible());
    EXPECT_FALSE(worker.diagnostics().final_joint_target_within_tolerance);
  }
  ASSERT_TRUE(worker.diagnostics().final_joint_target_error_max_rad);
  EXPECT_NEAR(*worker.diagnostics().final_joint_target_error_max_rad, 0.04, 1e-9);
  worker.update_actual_positions({});
  EXPECT_FALSE(worker.diagnostics().final_joint_target_error_max_rad);
  EXPECT_TRUE(worker.completion_eligible());
}

TEST(TrajectoryExecutionWorker, IntermediateRejectionCannotBeOverwritten)
{
  int calls = 0;
  TrajectoryExecutionWorker worker(
    std::make_shared<LinearTrajectorySampler>(),
    [&calls](const std::vector<double> &) {++calls; return false;});
  ASSERT_TRUE(worker.start(trajectory(), 1, 10.0).valid);
  EXPECT_FALSE(worker.on_simulation_time(11.0).valid);
  EXPECT_FALSE(worker.on_simulation_time(12.0).valid);
  EXPECT_FALSE(worker.completion_eligible());
  EXPECT_EQ(calls, 1);
}

TEST(TrajectoryExecutionWorker, LateSinkResultCannotMutateReplacementEvenWithReusedId)
{
  for (bool accepted : {false, true}) {
    std::shared_ptr<TrajectoryExecutionWorker> worker;
    bool replace = true;
    worker = std::make_shared<TrajectoryExecutionWorker>(
      std::make_shared<LinearTrajectorySampler>(),
      [&worker, &replace, accepted](const std::vector<double> &) {
        if (replace) {
          replace = false;
          worker->cancel();
          EXPECT_TRUE(worker->start(trajectory(), 1, 20.0).valid);
          return accepted;
        }
        return true;
      });
    ASSERT_TRUE(worker->start(trajectory(), 1, 10.0).valid);
    EXPECT_TRUE(worker->on_simulation_time(12.0).valid);
    EXPECT_TRUE(worker->active());
    EXPECT_FALSE(worker->failed());
    EXPECT_FALSE(worker->completion_eligible());
    EXPECT_EQ(worker->diagnostics().command_publish_success_count, 0u);
    EXPECT_TRUE(worker->on_simulation_time(22.0).valid);
    EXPECT_TRUE(worker->completion_eligible());
  }
}

TEST(TrajectoryExecutionWorker, CancelWhileSinkIsBlockedStopsQueuedClockCommands)
{
  std::promise<void> entered, release, queued;
  auto released = release.get_future();
  int commands = 0;
  TrajectoryExecutionWorker worker(std::make_shared<LinearTrajectorySampler>(),
    [&](const std::vector<double> &) {
      ++commands;
      entered.set_value();
      released.wait();
      return true;
    });
  ASSERT_TRUE(worker.start(trajectory(), 1, 10.0).valid);
  auto final_command = std::async(
    std::launch::async,
    [&]() {return worker.on_simulation_time(12.0);});
  entered.get_future().wait();
  EXPECT_FALSE(worker.completion_eligible());
  EXPECT_FALSE(worker.diagnostics().final_command_accepted_steady_time_s);
  auto later_clock = std::async(
    std::launch::async, [&]() {
      queued.set_value();
      return worker.on_simulation_time(13.0);
    });
  queued.get_future().wait();
  worker.cancel("halted");
  release.set_value();
  EXPECT_TRUE(final_command.get().valid);
  EXPECT_TRUE(later_clock.get().valid);
  EXPECT_EQ(commands, 1);
  EXPECT_FALSE(worker.completion_eligible());
  EXPECT_EQ(worker.diagnostics().terminal_reason, "halted");
  EXPECT_TRUE(worker.diagnostics().final_command_accepted);
}

TEST(TrajectoryExecutionWorker, InvalidClockAndThrowingSinkNeverComplete)
{
  for (double invalid : {9.0, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    TrajectoryExecutionWorker worker(std::make_shared<LinearTrajectorySampler>(),
      [](const std::vector<double> &) {return true;});
    ASSERT_TRUE(worker.start(trajectory(), 1, 10.0).valid);
    EXPECT_FALSE(worker.on_simulation_time(invalid).valid);
    EXPECT_FALSE(worker.on_simulation_time(12.0).valid);
    EXPECT_FALSE(worker.completion_eligible());
  }
  TrajectoryExecutionWorker worker(std::make_shared<LinearTrajectorySampler>(),
    [](const std::vector<double> &) -> bool {throw std::runtime_error("backend failure");});
  ASSERT_TRUE(worker.start(trajectory(), 1, 10.0).valid);
  EXPECT_FALSE(worker.on_simulation_time(12.0).valid);
  EXPECT_TRUE(worker.failed());
  EXPECT_FALSE(worker.completion_eligible());
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

TEST(TrajectoryExecutionWorker, RecordsFinalAttemptAndAcceptanceSeparately)
{
  for (bool accepted : {false, true}) {
    TrajectoryExecutionWorker worker(std::make_shared<LinearTrajectorySampler>(),
      [accepted](const std::vector<double> &) {return accepted;});
    ASSERT_TRUE(worker.start(trajectory(), 22, 200.0).valid);
    EXPECT_EQ(worker.on_simulation_time(202.0).valid, accepted);
    const auto diagnostics = worker.diagnostics();
    EXPECT_DOUBLE_EQ(diagnostics.planned_duration_s, 2.0);
    ASSERT_TRUE(diagnostics.final_command_attempt_simulation_time);
    EXPECT_DOUBLE_EQ(*diagnostics.final_command_attempt_simulation_time, 202.0);
    EXPECT_EQ(diagnostics.final_command_accepted_steady_time_s.has_value(), accepted);
    EXPECT_EQ(diagnostics.final_command_publish_success_count, accepted ? 1u : 0u);
    EXPECT_EQ(diagnostics.trajectory_execution_complete, accepted);
    EXPECT_FALSE(diagnostics.final_joint_target_error_max_rad);
    EXPECT_FALSE(diagnostics.terminal_reason.empty());
  }
}

}  // namespace
}  // namespace arm_cell_motion
