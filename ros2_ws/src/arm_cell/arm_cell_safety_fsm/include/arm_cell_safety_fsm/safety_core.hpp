#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

#include <arm_cell_interfaces/msg/motion_status.hpp>
#include <arm_cell_interfaces/msg/safety_hardware_state.hpp>
#include <arm_cell_interfaces/msg/safety_state.hpp>
#include <arm_cell_interfaces/msg/stop_mode.hpp>

#include "arm_cell_safety_fsm/safety_input_tracker.hpp"

namespace arm_cell_safety_fsm
{

using MotionStatus = arm_cell_interfaces::msg::MotionStatus;
using SafetyHardwareState = arm_cell_interfaces::msg::SafetyHardwareState;
using SafetyState = arm_cell_interfaces::msg::SafetyState;
using MotionCapability = arm_cell_interfaces::msg::MotionCapability;
using SafetyCauseSet = arm_cell_interfaces::msg::SafetyCauseSet;
using StopMode = arm_cell_interfaces::msg::StopMode;

struct SafetyInputs
{
  SafetyHardwareState hardware;
  MotionStatus motion;
};

class SafetyCore
{
public:
  using Clock = SafetyInputTracker::Clock;

  explicit SafetyCore(
    std::chrono::milliseconds freshness_timeout = std::chrono::milliseconds(500));

  void update_hardware(
    const SafetyHardwareState & message, Clock::time_point receipt_time,
    bool source_timestamp_plausible = true,
    std::int64_t source_stamp_ns = 0,
    std::int64_t source_evaluation_ros_time_ns = 0,
    std::int64_t source_maximum_age_ns = 0);
  void update_motion(
    const MotionStatus & message, Clock::time_point receipt_time,
    bool source_timestamp_plausible = true,
    std::int64_t source_stamp_ns = 0,
    std::int64_t source_evaluation_ros_time_ns = 0,
    std::int64_t source_maximum_age_ns = 0);

  SafetyState evaluate(Clock::time_point now);
  std::array<SafetyInputDiagnostic, static_cast<std::size_t>(SafetyInput::COUNT)>
  input_diagnostics(Clock::time_point now) const;
  bool should_dispatch_stop() const;
  uint8_t selected_stop_mode() const;
  bool motion_inactive() const;
  bool reset(
    bool operator_clear, Clock::time_point now,
    std::string * diagnostic_detail = nullptr);

private:
  SafetyState evaluate_locked(Clock::time_point now);
  bool motion_inactive_locked() const;
  static bool is_recovery_eligible(uint32_t causes);
  static bool is_known_motion_state(uint8_t value);

  mutable std::mutex mutex_;
  SafetyInputTracker tracker_;
  SafetyInputs inputs_;
  uint32_t latched_causes_{SafetyCauseSet::NONE};
  bool recovery_stop_completion_required_{false};
  SafetyState last_state_;
  bool has_last_state_{false};
};

}  // namespace arm_cell_safety_fsm
