#pragma once

#include <arm_cell_interfaces/msg/motion_status.hpp>
#include <arm_cell_interfaces/msg/safety_state.hpp>

namespace arm_cell_orchestration_bt
{

inline bool recovery_ready_for_immediate_return(
  const arm_cell_interfaces::msg::SafetyState & safety,
  const arm_cell_interfaces::msg::MotionStatus & motion)
{
  return safety.valid && safety.required_inputs_fresh &&
         safety.motion_capability.value ==
         arm_cell_interfaces::msg::MotionCapability::MOTION_RECOVERY_ONLY &&
         motion.execution_state == arm_cell_interfaces::msg::MotionStatus::MOTION_STATE_STOPPED &&
         !motion.execution_active && motion.backend_inactivity_confirmed;
}

inline bool recovery_severity_escalation_ready_for_immediate_return(
  const arm_cell_interfaces::msg::SafetyState & safety)
{
  return safety.valid && safety.required_inputs_fresh &&
         safety.selected_stop_mode.value >=
         arm_cell_interfaces::msg::StopMode::STOP_MODE_IMMEDIATE;
}

}  // namespace arm_cell_orchestration_bt
