#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>

#include <arm_cell_interfaces/msg/motion_task_phase.hpp>

#include "arm_cell_orchestration/motion_feedback_supervision.hpp"

namespace arm_cell_orchestration
{

using Clock = std::chrono::steady_clock;

TEST(MotionFeedbackSupervisionTest, DetectsSilenceAndRecoversForMatchingGoal)
{
  MotionFeedbackSupervision monitor;
  const std::array<std::uint8_t, 16> goal_a{};
  const auto start = Clock::time_point{};
  monitor.accept(goal_a, start);
  monitor.observe(goal_a, 3U, start + std::chrono::milliseconds(500));

  EXPECT_FALSE(monitor.stale(start + std::chrono::milliseconds(1999)));
  EXPECT_TRUE(monitor.stale(start + std::chrono::milliseconds(2000)));
  EXPECT_EQ(monitor.observe(goal_a, 4U, start + std::chrono::milliseconds(2100)),
    MotionFeedbackSupervision::Observation::RECOVERED);
  EXPECT_FALSE(monitor.stale(start + std::chrono::milliseconds(2100)));
}

TEST(MotionFeedbackSupervisionTest, RepeatedGripperPhaseHeartbeatsKeepReleaseRetryLive)
{
  MotionFeedbackSupervision monitor;
  const std::array<std::uint8_t, 16> goal{};
  const auto start = Clock::time_point{};
  const auto gripper_phase =
    arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_GRIPPER;
  monitor.accept(goal, start);

  for (const auto elapsed : {500, 1000, 1500, 2000, 2500}) {
    EXPECT_EQ(
      monitor.observe(goal, gripper_phase, start + std::chrono::milliseconds(elapsed)),
      MotionFeedbackSupervision::Observation::RECEIVED);
    EXPECT_FALSE(monitor.stale(start + std::chrono::milliseconds(elapsed)));
  }
  EXPECT_TRUE(monitor.stale(start + std::chrono::milliseconds(4000)));
}

TEST(MotionFeedbackSupervisionTest, IgnoresFeedbackFromAnotherOrTerminalGoal)
{
  MotionFeedbackSupervision monitor;
  std::array<std::uint8_t, 16> goal_a{};
  auto goal_b = goal_a;
  goal_b[0] = 1U;
  const auto start = Clock::time_point{};
  monitor.accept(goal_a, start);
  monitor.terminal(goal_a);
  monitor.accept(goal_b, start + std::chrono::seconds(1));

  EXPECT_EQ(monitor.observe(goal_a, 5U, start + std::chrono::seconds(2)),
    MotionFeedbackSupervision::Observation::IGNORED);
  EXPECT_FALSE(monitor.stale(start + std::chrono::milliseconds(2499)));
  EXPECT_TRUE(monitor.stale(start + std::chrono::milliseconds(2500)));
}

TEST(MotionFeedbackSupervisionTest, OnlyForwardsAcceptedActiveGoalFeedback)
{
  MotionFeedbackSupervision monitor;
  std::array<std::uint8_t, 16> goal_a{};
  auto goal_b = goal_a;
  goal_b[0] = 1U;
  auto goal_c = goal_a;
  goal_c[0] = 2U;
  const auto start = Clock::time_point{};
  unsigned forwarded = 0U;
  unsigned gripper_phase_forwards = 0U;
  const auto gripper_phase =
    arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_GRIPPER;
  const auto forward = [&forwarded, &gripper_phase_forwards, gripper_phase](
    MotionFeedbackSupervision::Observation, std::uint8_t phase) {
      ++forwarded;
      if (phase == gripper_phase) {
        ++gripper_phase_forwards;
      }
    };

  EXPECT_EQ(monitor.observe_and_forward(
      goal_a, gripper_phase, start, forward), MotionFeedbackSupervision::Observation::IGNORED);
  monitor.accept(goal_b, start);
  EXPECT_EQ(monitor.observe_and_forward(
      goal_a, gripper_phase, start + std::chrono::milliseconds(1), forward),
    MotionFeedbackSupervision::Observation::IGNORED);
  EXPECT_EQ(forwarded, 0U);
  EXPECT_EQ(monitor.observe_and_forward(
      goal_b, gripper_phase, start + std::chrono::milliseconds(2), forward),
    MotionFeedbackSupervision::Observation::RECEIVED);
  EXPECT_EQ(forwarded, 1U);
  EXPECT_EQ(gripper_phase_forwards, 1U);
  EXPECT_TRUE(monitor.terminal(goal_b));
  EXPECT_EQ(monitor.observe_and_forward(
      goal_b, gripper_phase, start + std::chrono::milliseconds(3), forward),
    MotionFeedbackSupervision::Observation::IGNORED);
  monitor.accept(goal_c, start + std::chrono::milliseconds(4));
  EXPECT_EQ(monitor.observe_and_forward(
      goal_b, gripper_phase, start + std::chrono::milliseconds(5), forward),
    MotionFeedbackSupervision::Observation::IGNORED);
  EXPECT_EQ(monitor.observe_and_forward(
      goal_c, 0U, start + std::chrono::milliseconds(6), forward),
    MotionFeedbackSupervision::Observation::RECEIVED);
  EXPECT_EQ(forwarded, 2U);
  EXPECT_EQ(gripper_phase_forwards, 1U);
}

TEST(MotionFeedbackSupervisionTest, TerminalWaitsForAnAlreadyAcceptedFeedbackDelivery)
{
  MotionFeedbackSupervision monitor;
  const std::array<std::uint8_t, 16> goal{};
  const auto start = Clock::time_point{};
  monitor.accept(goal, start);
  std::mutex mutex;
  std::condition_variable condition;
  bool callback_entered = false;
  bool release_callback = false;
  bool terminal_started = false;
  std::atomic<bool> terminal_finished{false};
  std::thread feedback_thread([&]() {
      monitor.observe_and_forward(
        goal, 3U, start + std::chrono::milliseconds(1),
        [&](MotionFeedbackSupervision::Observation, std::uint8_t) {
          std::unique_lock<std::mutex> lock(mutex);
          callback_entered = true;
          condition.notify_all();
          condition.wait(lock, [&release_callback]() {return release_callback;});
        });
    });
  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(condition.wait_for(lock, std::chrono::seconds(1), [&callback_entered]() {
      return callback_entered;
    }));
  }
  std::thread terminal_thread([&]() {
      {
        std::lock_guard<std::mutex> lock(mutex);
        terminal_started = true;
      }
      condition.notify_all();
      monitor.terminal(goal);
      terminal_finished.store(true);
      condition.notify_all();
    });
  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(condition.wait_for(lock, std::chrono::seconds(1), [&terminal_started]() {
      return terminal_started;
    }));
    EXPECT_FALSE(condition.wait_for(lock, std::chrono::milliseconds(20), [&terminal_finished]() {
      return terminal_finished.load();
    }));
    release_callback = true;
  }
  condition.notify_all();
  feedback_thread.join();
  terminal_thread.join();
  EXPECT_TRUE(terminal_finished.load());
  unsigned forwarded = 0U;
  EXPECT_EQ(monitor.observe_and_forward(
      goal, 4U, start + std::chrono::milliseconds(2),
      [&forwarded](MotionFeedbackSupervision::Observation, std::uint8_t) {++forwarded;}),
    MotionFeedbackSupervision::Observation::IGNORED);
  EXPECT_EQ(forwarded, 0U);
}

}  // namespace arm_cell_orchestration
