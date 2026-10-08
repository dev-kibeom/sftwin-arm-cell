#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <vector>

#include <arm_cell_interfaces/msg/amr_docking_state.hpp>
#include <arm_cell_interfaces/msg/motion_status.hpp>
#include <arm_cell_interfaces/msg/pack_ml_state.hpp>
#include <arm_cell_interfaces/msg/safety_hardware_state.hpp>
#include <arm_cell_interfaces/msg/safety_state.hpp>
#include <arm_cell_interfaces/msg/stop_mode.hpp>

#include "arm_cell_safety_fsm/safety_input_tracker.hpp"

namespace arm_cell_safety_fsm
{

using AMRDockingState = arm_cell_interfaces::msg::AMRDockingState;
using MotionStatus = arm_cell_interfaces::msg::MotionStatus;
using PackMLState = arm_cell_interfaces::msg::PackMLState;
using SafetyHardwareState = arm_cell_interfaces::msg::SafetyHardwareState;
using SafetyState = arm_cell_interfaces::msg::SafetyState;
using MotionCapability = arm_cell_interfaces::msg::MotionCapability;
using SafetyCauseSet = arm_cell_interfaces::msg::SafetyCauseSet;
using StopMode = arm_cell_interfaces::msg::StopMode;

struct SafetyInputs
{
  AMRDockingState amr;
  PackMLState packml;
  SafetyHardwareState hardware;
  MotionStatus motion;
};

class SafetyCore
{
public:
  using Clock = SafetyInputTracker::Clock;

  explicit SafetyCore(
    std::chrono::milliseconds freshness_timeout = std::chrono::milliseconds(500),
    std::chrono::milliseconds degraded_freshness_threshold = std::chrono::milliseconds(0),
    float degraded_velocity_scale = 0.5F,
    float degraded_acceleration_scale = 0.5F,
    std::vector<SafetyInput> degraded_freshness_inputs = {});

  void update_amr(
    const AMRDockingState & message, Clock::time_point receipt_time,
    bool source_timestamp_plausible = true);
  void update_packml(
    const PackMLState & message, Clock::time_point receipt_time,
    bool source_timestamp_plausible = true);
  void update_hardware(
    const SafetyHardwareState & message, Clock::time_point receipt_time,
    bool source_timestamp_plausible = true);
  void update_motion(
    const MotionStatus & message, Clock::time_point receipt_time,
    bool source_timestamp_plausible = true);

  SafetyState evaluate(Clock::time_point now);
  bool should_dispatch_stop() const;
  uint8_t selected_stop_mode() const;
  bool motion_inactive() const;
  bool reset(bool operator_clear, Clock::time_point now);

private:
  SafetyState evaluate_locked(Clock::time_point now);
  bool motion_inactive_locked() const;
  static bool is_recovery_eligible(uint32_t causes);
  static bool is_known_amr_state(uint8_t value);
  static bool is_known_packml_state(uint8_t value);
  static bool is_known_motion_state(uint8_t value);

  mutable std::mutex mutex_;
  SafetyInputTracker tracker_;
  std::chrono::milliseconds degraded_freshness_threshold_;
  float degraded_velocity_scale_;
  float degraded_acceleration_scale_;
  std::vector<SafetyInput> degraded_freshness_inputs_;
  SafetyInputs inputs_;
  uint32_t latched_causes_{SafetyCauseSet::NONE};
  bool normal_supervision_established_{false};
  bool recovery_stop_completion_required_{false};
  SafetyState last_state_;
  bool has_last_state_{false};
};

}  // namespace arm_cell_safety_fsm
