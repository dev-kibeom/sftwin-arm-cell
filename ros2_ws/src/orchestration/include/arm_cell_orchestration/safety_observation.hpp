#pragma once

#include <chrono>
#include <cstdint>

#include <arm_cell_interfaces/msg/motion_capability.hpp>
#include <arm_cell_interfaces/msg/safety_state.hpp>
#include <arm_cell_interfaces/msg/stop_mode.hpp>

namespace arm_cell_orchestration
{

struct SafetyObservation
{
  using Clock = std::chrono::steady_clock;
  using State = arm_cell_interfaces::msg::SafetyState;

  State state;
  bool received{false};
  bool delivery_valid{false};
  Clock::time_point receipt_time{};

  SafetyObservation()
  : state(rosidl_runtime_cpp::MessageInitialization::ALL) {}

  bool normal_motion_allowed() const
  {
    return delivery_valid && state.motion_capability.value ==
           arm_cell_interfaces::msg::MotionCapability::MOTION_NORMAL;
  }

  bool recovery_motion_allowed() const
  {
    return delivery_valid && state.motion_capability.value ==
           arm_cell_interfaces::msg::MotionCapability::MOTION_RECOVERY_ONLY;
  }
};

inline bool known_safety_state(uint8_t value)
{
  using State = arm_cell_interfaces::msg::SafetyState;
  return value <= State::SAFETY_STATE_EMERGENCY_STOPPED;
}

inline bool known_motion_capability(uint8_t value)
{
  using Capability = arm_cell_interfaces::msg::MotionCapability;
  return value <= Capability::MOTION_NORMAL;
}

inline bool known_stop_mode(uint8_t value)
{
  using StopMode = arm_cell_interfaces::msg::StopMode;
  return value <= StopMode::STOP_MODE_EMERGENCY;
}

inline SafetyObservation evaluate_safety_observation(
  const arm_cell_interfaces::msg::SafetyState & state, bool received,
  SafetyObservation::Clock::time_point receipt_time,
  SafetyObservation::Clock::time_point now,
  SafetyObservation::Clock::duration freshness_timeout)
{
  SafetyObservation result;
  result.state = state;
  result.received = received;
  result.receipt_time = receipt_time;
  result.delivery_valid = received && now >= receipt_time &&
    now - receipt_time < freshness_timeout && state.valid && state.required_inputs_fresh &&
    known_safety_state(state.safety_state) &&
    known_motion_capability(state.motion_capability.value) &&
    known_stop_mode(state.selected_stop_mode.value);
  return result;
}

inline SafetyObservation refresh_safety_observation(
  const SafetyObservation & observation,
  SafetyObservation::Clock::time_point now,
  SafetyObservation::Clock::duration freshness_timeout)
{
  return evaluate_safety_observation(
    observation.state, observation.received, observation.receipt_time, now, freshness_timeout);
}

}  // namespace arm_cell_orchestration
