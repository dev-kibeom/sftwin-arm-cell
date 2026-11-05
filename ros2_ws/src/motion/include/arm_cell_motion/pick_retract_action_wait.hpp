#pragma once

#include <algorithm>
#include <chrono>
#include <functional>
#include <future>

namespace arm_cell_motion
{

enum class PickRetractActionWaitTermination
{
  RESULT_READY,
  MOTION_INTERRUPTED,
  PLANNING_BUDGET_EXPIRED
};

struct PickRetractActionWaitResult
{
  PickRetractActionWaitTermination termination;
  bool result_ready;
};

inline PickRetractActionWaitResult wait_for_pick_retract_action_result(
  const std::function<std::future_status(std::chrono::milliseconds)> & wait_for_result,
  std::chrono::milliseconds planning_budget,
  const std::function<bool()> & interrupted,
  const std::function<void()> & cancel,
  std::chrono::milliseconds cancel_result_grace)
{
  constexpr auto kPollInterval = std::chrono::milliseconds(50);
  const auto planning_deadline = std::chrono::steady_clock::now() + planning_budget;
  auto termination = PickRetractActionWaitTermination::RESULT_READY;
  bool cancellation_requested = false;
  while (!cancellation_requested) {
    if (interrupted && interrupted()) {
      termination = PickRetractActionWaitTermination::MOTION_INTERRUPTED;
      cancellation_requested = true;
    } else if (std::chrono::steady_clock::now() >= planning_deadline) {
      termination = PickRetractActionWaitTermination::PLANNING_BUDGET_EXPIRED;
      cancellation_requested = true;
    }
    if (cancellation_requested) {
      cancel();
      break;
    }
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      planning_deadline - std::chrono::steady_clock::now());
    const auto wait_duration = std::max(
      std::chrono::milliseconds(1), std::min(kPollInterval, remaining));
    if (wait_for_result(wait_duration) == std::future_status::ready) {
      return {PickRetractActionWaitTermination::RESULT_READY, true};
    }
  }

  const auto cancel_deadline = std::chrono::steady_clock::now() + cancel_result_grace;
  while (std::chrono::steady_clock::now() < cancel_deadline) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      cancel_deadline - std::chrono::steady_clock::now());
    const auto wait_duration = std::max(
      std::chrono::milliseconds(1), std::min(kPollInterval, remaining));
    if (wait_for_result(wait_duration) == std::future_status::ready) {
      return {termination, true};
    }
  }
  return {termination, false};
}

}  // namespace arm_cell_motion
