#pragma once

#include <array>
#include <cstdint>
#include <tuple>

#include <arm_cell_interfaces/msg/motion_status.hpp>
#include "arm_cell_orchestration/safety_observation.hpp"

namespace arm_cell_orchestration
{

inline bool recovery_ready_for_immediate_return(
  const SafetyObservation & safety,
  const arm_cell_interfaces::msg::MotionStatus & motion)
{
  return safety.recovery_motion_allowed() &&
         motion.execution_state == arm_cell_interfaces::msg::MotionStatus::MOTION_STATE_STOPPED &&
         !motion.execution_active && motion.backend_inactivity_confirmed;
}

inline bool recovery_ready_for_batch_continuation(
  const SafetyObservation & safety,
  const arm_cell_interfaces::msg::MotionStatus & motion,
  bool motion_status_fresh = true)
{
  return safety.normal_motion_allowed() && motion_status_fresh &&
         motion.execution_state == arm_cell_interfaces::msg::MotionStatus::MOTION_STATE_STOPPED &&
         !motion.execution_active && !motion.has_active_execution &&
         motion.backend_inactivity_confirmed &&
         motion.holding_state == arm_cell_interfaces::msg::MotionStatus::HOLDING_RELEASED;
}

inline bool recovery_severity_escalation_ready_for_immediate_return(
  const SafetyObservation & safety)
{
  return safety.delivery_valid && safety.state.selected_stop_mode.value >=
         arm_cell_interfaces::msg::StopMode::STOP_MODE_IMMEDIATE;
}

using RecoveryObservationDiagnosticSignature = std::tuple<
  bool, uint8_t, uint8_t, uint8_t, bool, bool, bool, uint8_t, bool,
  std::array<uint8_t, 16>, bool, bool, bool, bool>;

inline RecoveryObservationDiagnosticSignature recovery_observation_diagnostic_signature(
  const SafetyObservation & safety,
  const arm_cell_interfaces::msg::MotionStatus & motion,
  bool motion_observed,
  bool motion_fresh)
{
  return {
    safety.delivery_valid,
    safety.state.motion_capability.value,
    safety.state.selected_stop_mode.value,
    motion.execution_state,
    motion.execution_active,
    motion.has_active_execution,
    motion.backend_inactivity_confirmed,
    motion.holding_state,
    motion.has_last_stop_request,
    motion.last_stop_request_id.uuid,
    motion_observed,
    motion_fresh,
    recovery_ready_for_immediate_return(safety, motion),
    recovery_severity_escalation_ready_for_immediate_return(safety)};
}

}  // namespace arm_cell_orchestration
