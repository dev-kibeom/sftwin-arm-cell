#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include "arm_cell_motion_moveit2/motion_profile_trace.hpp"
#include "arm_cell_motion_moveit2/trajectory_execution_worker.hpp"

namespace arm_cell_motion_moveit2
{
namespace
{
TEST(MotionProfileTrace, BoundsSamplesAndWritesManifestWithArtifactReference)
{
  const auto directory = std::filesystem::temp_directory_path() / "motion_profile_trace_test";
  std::filesystem::remove_all(directory);
  MotionProfileTrace trace(directory.string(), 1);
  trace.begin("run-1", "test-runtime", "test-profile", "simulation_time", "test-sha");
  trace.add_row({0.0, "GO_HOME", {0.1}, {0.0}, {}, {0.0}, {0.0}});
  trace.add_row({0.1, "GO_HOME", {0.1}, {0.0}, {}, {0.2}, {0.5}});

  const auto result = trace.finish("completed", {"joint_1"}, "active_limits=test");

  ASSERT_TRUE(result.success) << result.error;
  EXPECT_EQ(result.sample_count, 1u);
  ASSERT_TRUE(std::filesystem::exists(result.samples_path));
  ASSERT_TRUE(std::filesystem::exists(result.manifest_path));
  std::ifstream manifest(result.manifest_path);
  const std::string contents(
    (std::istreambuf_iterator<char>(manifest)), std::istreambuf_iterator<char>());
  EXPECT_NE(contents.find("run-1"), std::string::npos);
  EXPECT_NE(contents.find("simulation_time"), std::string::npos);
  EXPECT_NE(contents.find(result.samples_path), std::string::npos);
  EXPECT_NE(contents.find("truncated"), std::string::npos);
  EXPECT_NE(contents.find("motion-profile-run-1"), std::string::npos);
  std::filesystem::remove_all(directory);
}

TEST(MotionProfileTrace, CapturingDoesNotChangeTrajectoryCommandValues)
{
  auto make_trajectory = []() {
      trajectory_msgs::msg::JointTrajectory trajectory;
      trajectory.joint_names = {"joint_1"};
      trajectory_msgs::msg::JointTrajectoryPoint first;
      first.positions = {0.0};
      trajectory_msgs::msg::JointTrajectoryPoint last;
      last.positions = {1.0};
      last.time_from_start.sec = 1;
      trajectory.points = {first, last};
      return trajectory;
    };
  std::vector<TimedJointSample> baseline_commands;
  std::vector<TimedJointSample> captured_commands;
  auto baseline = std::make_shared<TrajectoryExecutionWorker>(
    std::make_shared<LinearTrajectorySampler>(), [&baseline_commands](const TimedJointSample & s) {
      baseline_commands.push_back(s);
      return true;
    });
  auto trace = std::make_shared<MotionProfileTrace>(
    std::filesystem::temp_directory_path().string(), 100);
  trace->begin("command-equivalence", "runtime", "profile", "sim_time", "sha");
  auto captured = std::make_shared<TrajectoryExecutionWorker>(
    std::make_shared<LinearTrajectorySampler>(), [&captured_commands](const TimedJointSample & s) {
      captured_commands.push_back(s);
      return true;
    });
  captured->set_profile_trace(trace);
  ASSERT_TRUE(baseline->start(make_trajectory(), 1, 2.0).valid);
  ASSERT_TRUE(captured->start(make_trajectory(), 1, 2.0).valid);
  ASSERT_TRUE(baseline->on_simulation_time(2.5).valid);
  ASSERT_TRUE(captured->on_simulation_time(2.5).valid);

  ASSERT_EQ(baseline_commands.size(), captured_commands.size());
  ASSERT_EQ(captured_commands.size(), 1u);
  EXPECT_EQ(baseline_commands[0].positions, captured_commands[0].positions);
  EXPECT_EQ(baseline_commands[0].velocities, captured_commands[0].velocities);
}

TEST(MotionProfileTrace, ConcurrentRowsRemainBoundedAndCounted)
{
  constexpr std::size_t max_samples = 256;
  constexpr std::size_t thread_count = 8;
  constexpr std::size_t rows_per_thread = 10000;
  MotionProfileTrace trace(std::filesystem::temp_directory_path().string(), max_samples);
  trace.begin("concurrent-trace", "runtime", "profile", "sim_time", "sha");
  std::atomic<bool> start{false};
  std::atomic<std::size_t> finished{0};
  std::vector<std::thread> writers;
  for (std::size_t thread_index = 0; thread_index < thread_count; ++thread_index) {
    writers.emplace_back(
      [&, thread_index]() {
        while (!start.load(std::memory_order_acquire)) {}
        for (std::size_t row_index = 0; row_index < rows_per_thread; ++row_index) {
          trace.add_row(
            {static_cast<double>(thread_index * rows_per_thread + row_index),
              "GLOBAL_MOVEIT", {}, {}, {}, {}, {}});
          (void)trace.sample_count();
        }
        finished.fetch_add(1, std::memory_order_release);
      });
  }
  start.store(true, std::memory_order_release);
  while (finished.load(std::memory_order_acquire) != thread_count) {
    (void)trace.sample_count();
  }
  for (auto & writer : writers) {
    writer.join();
  }

  EXPECT_EQ(trace.sample_count(), max_samples);
  const auto result = trace.finish("completed", {}, "");
  EXPECT_EQ(result.sample_count, max_samples);
  EXPECT_EQ(result.dropped_sample_count, thread_count * rows_per_thread - max_samples);
}
}  // namespace
}  // namespace arm_cell_motion_moveit2
