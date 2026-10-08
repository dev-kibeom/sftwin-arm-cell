#include "arm_cell_safety_fsm/safety_core.hpp"

#include <cmath>
#include <string>
#include <utility>

namespace arm_cell_safety_fsm
{

namespace
{
const char * amr_state_name(uint8_t state)
{
  switch (state) {
    case AMRDockingState::AMR_DOCKING_DOCKED: return "DOCKED";
    case AMRDockingState::AMR_DOCKING_UNDOCKED: return "UNDOCKED";
    default: return "UNKNOWN_OR_INVALID";
  }
}

const char * packml_state_name(uint8_t state)
{
  switch (state) {
    case PackMLState::PACKML_STATE_IDLE: return "IDLE";
    case PackMLState::PACKML_STATE_STARTING: return "STARTING";
    case PackMLState::PACKML_STATE_EXECUTE: return "EXECUTE";
    case PackMLState::PACKML_STATE_COMPLETE: return "COMPLETE";
    case PackMLState::PACKML_STATE_ABORTED: return "ABORTED";
    default: return "UNKNOWN_OR_INVALID";
  }
}

const char * motion_state_name(uint8_t state)
{
  switch (state) {
    case MotionStatus::MOTION_STATE_IDLE: return "IDLE";
    case MotionStatus::MOTION_STATE_EXECUTING: return "EXECUTING";
    case MotionStatus::MOTION_STATE_STOPPING: return "STOPPING";
    case MotionStatus::MOTION_STATE_STOPPED: return "STOPPED";
    case MotionStatus::MOTION_STATE_EMERGENCY_STOPPED: return "EMERGENCY_STOPPED";
    case MotionStatus::MOTION_STATE_FAULTED: return "FAULTED";
    default: return "UNKNOWN_OR_INVALID";
  }
}
}  // namespace

SafetyCore::SafetyCore(
  std::chrono::milliseconds freshness_timeout,
  std::chrono::milliseconds degraded_freshness_threshold,
  float degraded_velocity_scale,
  float degraded_acceleration_scale,
  std::vector<SafetyInput> degraded_freshness_inputs)
: tracker_(freshness_timeout),
  degraded_freshness_threshold_(degraded_freshness_threshold),
  degraded_velocity_scale_(degraded_velocity_scale),
  degraded_acceleration_scale_(degraded_acceleration_scale),
  degraded_freshness_inputs_(std::move(degraded_freshness_inputs))
{
  inputs_.amr.docking_state = AMRDockingState::AMR_DOCKING_UNKNOWN;
  inputs_.packml.state = PackMLState::PACKML_STATE_UNKNOWN;
  inputs_.motion.execution_state = MotionStatus::MOTION_STATE_UNKNOWN;
}

void SafetyCore::update_amr(
  const AMRDockingState & message, Clock::time_point receipt_time,
  bool source_timestamp_plausible)
{
  std::lock_guard<std::mutex> lock(mutex_);
  inputs_.amr = message;
  tracker_.record(
    SafetyInput::AMR, message.valid && is_known_amr_state(message.docking_state),
    source_timestamp_plausible, receipt_time);
}

void SafetyCore::update_packml(
  const PackMLState & message, Clock::time_point receipt_time,
  bool source_timestamp_plausible)
{
  std::lock_guard<std::mutex> lock(mutex_);
  inputs_.packml = message;
  tracker_.record(
    SafetyInput::PACKML, message.valid && is_known_packml_state(message.state),
    source_timestamp_plausible, receipt_time);
}

void SafetyCore::update_hardware(
  const SafetyHardwareState & message, Clock::time_point receipt_time,
  bool source_timestamp_plausible)
{
  std::lock_guard<std::mutex> lock(mutex_);
  inputs_.hardware = message;
  tracker_.record(SafetyInput::HARDWARE, message.valid, source_timestamp_plausible, receipt_time);
}

void SafetyCore::update_motion(
  const MotionStatus & message, Clock::time_point receipt_time,
  bool source_timestamp_plausible)
{
  std::lock_guard<std::mutex> lock(mutex_);
  inputs_.motion = message;
  const bool stop_status_consistent =
    message.execution_state != MotionStatus::MOTION_STATE_STOPPED ||
    (!message.execution_active && !message.has_active_execution &&
    message.backend_inactivity_confirmed);
  tracker_.record(
    SafetyInput::MOTION, is_known_motion_state(message.execution_state) && stop_status_consistent,
    source_timestamp_plausible, receipt_time);
}

SafetyState SafetyCore::evaluate(Clock::time_point now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return evaluate_locked(now);
}

bool SafetyCore::should_dispatch_stop() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_last_state_) {
    return false;
  }
  const bool motion_active =
    inputs_.motion.execution_active || inputs_.motion.has_active_execution ||
    inputs_.motion.execution_state == MotionStatus::MOTION_STATE_EXECUTING ||
    inputs_.motion.execution_state == MotionStatus::MOTION_STATE_STOPPING;
  if (!motion_active) {
    return false;
  }
  if (last_state_.active_causes.value != SafetyCauseSet::NONE) {
    return true;
  }
  if (!last_state_.motion_envelope_valid) {
    return false;
  }
  constexpr float kEnvelopeComparisonTolerance = 1.0e-6F;
  return !inputs_.motion.applied_envelope_valid ||
         !std::isfinite(inputs_.motion.applied_velocity_scale) ||
         !std::isfinite(inputs_.motion.applied_acceleration_scale) ||
         inputs_.motion.applied_velocity_scale >
         last_state_.max_velocity_scale + kEnvelopeComparisonTolerance ||
         inputs_.motion.applied_acceleration_scale >
         last_state_.max_acceleration_scale + kEnvelopeComparisonTolerance;
}

bool SafetyCore::motion_inactive() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return motion_inactive_locked();
}

bool SafetyCore::motion_inactive_locked() const
{
  if (inputs_.motion.execution_active || inputs_.motion.has_active_execution) {
    return false;
  }
  if (inputs_.motion.execution_state == MotionStatus::MOTION_STATE_IDLE) {
    return true;
  }
  return inputs_.motion.execution_state == MotionStatus::MOTION_STATE_STOPPED &&
         inputs_.motion.backend_inactivity_confirmed;
}

uint8_t SafetyCore::selected_stop_mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_last_state_) {
    return arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY;
  }
  return last_state_.selected_stop_mode.value;
}

SafetyState SafetyCore::evaluate_locked(Clock::time_point now)
{
  SafetyState state;
  const bool fresh = tracker_.all_required_fresh(now);
  state.required_inputs_fresh = fresh;
  const bool degraded = fresh && tracker_.any_in_degraded_band(
    now, degraded_freshness_threshold_, degraded_freshness_inputs_);
  const bool motion_execution_active =
    inputs_.motion.execution_active || inputs_.motion.has_active_execution ||
    inputs_.motion.execution_state == MotionStatus::MOTION_STATE_EXECUTING ||
    inputs_.motion.execution_state == MotionStatus::MOTION_STATE_STOPPING;

  uint32_t active_causes = SafetyCauseSet::NONE;
  if (inputs_.hardware.valid && inputs_.hardware.e_stop_active) {
    active_causes |= SafetyCauseSet::E_STOP;
  }
  if (inputs_.hardware.valid && inputs_.hardware.sto_active) {
    active_causes |= SafetyCauseSet::STO;
  }
  if (!fresh) {
    bool all_received = true;
    for (std::size_t i = 0; i < static_cast<std::size_t>(SafetyInput::COUNT); ++i) {
      all_received = all_received && tracker_.received(static_cast<SafetyInput>(i));
    }
    if ((all_received && tracker_.all_received_valid()) ||
      (motion_execution_active && !all_received))
    {
      active_causes |= SafetyCauseSet::COMMUNICATION_LOSS;
    }
    if (tracker_.any_received_invalid()) {
      active_causes |= SafetyCauseSet::REQUIRED_INPUT_INVALID;
    }
    state.safety_state = SafetyState::SAFETY_STATE_UNKNOWN;
    state.valid = false;
    state.motion_capability.value = MotionCapability::MOTION_NONE;
  } else {
    if (!inputs_.hardware.e_stop_active && !inputs_.hardware.sto_active) {
      normal_supervision_established_ = true;
    }
    if (
      inputs_.amr.docking_state == AMRDockingState::AMR_DOCKING_UNDOCKED &&
      motion_execution_active)
    {
      active_causes |= SafetyCauseSet::PREMATURE_UNDOCK;
    }
    if (
      inputs_.packml.state == PackMLState::PACKML_STATE_ABORTED &&
      motion_execution_active)
    {
      active_causes |= SafetyCauseSet::PACKML_ABORT;
    }

    state.valid = true;
    state.safety_state = SafetyState::SAFETY_STATE_SAFE;
    state.motion_capability.value =
      inputs_.packml.state == PackMLState::PACKML_STATE_ABORTED ?
      MotionCapability::MOTION_NONE : MotionCapability::MOTION_NORMAL;
  }

  const bool startup_input_fault_only = !normal_supervision_established_ &&
    (active_causes & ~(SafetyCauseSet::COMMUNICATION_LOSS |
    SafetyCauseSet::REQUIRED_INPUT_INVALID)) == 0;
  const uint32_t startup_transient_causes = normal_supervision_established_ ?
    SafetyCauseSet::NONE :
    active_causes & (SafetyCauseSet::COMMUNICATION_LOSS | SafetyCauseSet::REQUIRED_INPUT_INVALID);
  const uint32_t causes_to_latch = active_causes & ~startup_transient_causes;
  latched_causes_ |= causes_to_latch;
  if (causes_to_latch != SafetyCauseSet::NONE && motion_execution_active) {
    recovery_stop_completion_required_ = true;
  }
  state.active_causes.value = active_causes;
  state.latched_causes.value = latched_causes_;
  const auto selected_causes = active_causes | latched_causes_;
  if (selected_causes & (SafetyCauseSet::E_STOP | SafetyCauseSet::STO)) {
    state.selected_stop_mode.value = StopMode::STOP_MODE_EMERGENCY;
  } else if (selected_causes & (SafetyCauseSet::COMMUNICATION_LOSS |
    SafetyCauseSet::REQUIRED_INPUT_INVALID | SafetyCauseSet::COLLISION |
    SafetyCauseSet::PACKML_ABORT | SafetyCauseSet::OTHER))
  {
    state.selected_stop_mode.value = StopMode::STOP_MODE_IMMEDIATE;
  } else {
    state.selected_stop_mode.value = StopMode::STOP_MODE_CONTROLLED;
  }

  if (active_causes & (SafetyCauseSet::E_STOP | SafetyCauseSet::STO)) {
    state.safety_state = SafetyState::SAFETY_STATE_EMERGENCY_STOPPED;
    state.motion_capability.value = MotionCapability::MOTION_NONE;
  } else if (active_causes != SafetyCauseSet::NONE && !startup_input_fault_only) {
    state.safety_state = SafetyState::SAFETY_STATE_INTERLOCKED;
    state.motion_capability.value = MotionCapability::MOTION_NONE;
  } else if (latched_causes_ != SafetyCauseSet::NONE) {
    state.safety_state = SafetyState::SAFETY_STATE_RECOVERY_REQUIRED;
    state.motion_capability.value =
      motion_inactive_locked() && (!recovery_stop_completion_required_ ||
      (inputs_.motion.execution_state == MotionStatus::MOTION_STATE_STOPPED &&
      inputs_.motion.backend_inactivity_confirmed)) &&
      is_recovery_eligible(latched_causes_) ?
      MotionCapability::MOTION_RECOVERY_ONLY : MotionCapability::MOTION_NONE;
  }
  if (inputs_.motion.execution_state == MotionStatus::MOTION_STATE_EMERGENCY_STOPPED) {
    state.safety_state = SafetyState::SAFETY_STATE_EMERGENCY_STOPPED;
    state.motion_capability.value = MotionCapability::MOTION_NONE;
  } else if (inputs_.motion.execution_state == MotionStatus::MOTION_STATE_STOPPING) {
    state.safety_state = SafetyState::SAFETY_STATE_STOPPING;
    state.motion_capability.value = MotionCapability::MOTION_NONE;
  } else if (inputs_.motion.execution_state == MotionStatus::MOTION_STATE_FAULTED) {
    state.safety_state = SafetyState::SAFETY_STATE_INTERLOCKED;
    state.motion_capability.value = MotionCapability::MOTION_NONE;
  }
  if (state.valid && state.required_inputs_fresh &&
    state.motion_capability.value != MotionCapability::MOTION_NONE)
  {
    state.motion_envelope_valid = true;
    state.max_velocity_scale = degraded ? degraded_velocity_scale_ : 1.0F;
    state.max_acceleration_scale = degraded ? degraded_acceleration_scale_ : 1.0F;
  }
  if (state.motion_capability.value == MotionCapability::MOTION_NONE) {
    state.diagnostic_detail = std::string("permission_denied amr_state=") +
      amr_state_name(inputs_.amr.docking_state) +
      " amr_valid=" + (inputs_.amr.valid ? "true" : "false") +
      " packml_state=" + packml_state_name(inputs_.packml.state) +
      " packml_valid=" + (inputs_.packml.valid ? "true" : "false") +
      " motion_state=" + motion_state_name(inputs_.motion.execution_state) +
      " hardware_valid=" + (inputs_.hardware.valid ? "true" : "false") +
      " e_stop_active=" + (inputs_.hardware.e_stop_active ? "true" : "false") +
      " sto_active=" + (inputs_.hardware.sto_active ? "true" : "false");
    if (!fresh) {
      state.diagnostic_detail += " deny_reason=required_inputs_not_fresh";
    } else if (active_causes != SafetyCauseSet::NONE) {
      state.diagnostic_detail += " deny_reason=active_safety_cause";
    } else if (latched_causes_ != SafetyCauseSet::NONE) {
      state.diagnostic_detail += " deny_reason=recovery_not_eligible_or_motion_not_inactive";
    } else if (inputs_.motion.execution_state == MotionStatus::MOTION_STATE_FAULTED) {
      state.diagnostic_detail += " deny_reason=motion_faulted";
    } else {
      state.diagnostic_detail += " deny_reason=normal_capability_not_granted";
    }
  }
  last_state_ = state;
  has_last_state_ = true;
  return state;
}

bool SafetyCore::reset(bool operator_clear, Clock::time_point now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto state = evaluate_locked(now);
  if (
    !operator_clear || !state.required_inputs_fresh || state.active_causes.value != 0 ||
    inputs_.hardware.e_stop_active || inputs_.hardware.sto_active ||
    !motion_inactive_locked() || latched_causes_ == SafetyCauseSet::NONE ||
    (recovery_stop_completion_required_ &&
    (inputs_.motion.execution_state != MotionStatus::MOTION_STATE_STOPPED ||
    !inputs_.motion.backend_inactivity_confirmed)))
  {
    return false;
  }
  latched_causes_ = SafetyCauseSet::NONE;
  recovery_stop_completion_required_ = false;
  return true;
}

bool SafetyCore::is_recovery_eligible(uint32_t causes)
{
  constexpr uint32_t allowed =
    SafetyCauseSet::COMMUNICATION_LOSS | SafetyCauseSet::REQUIRED_INPUT_INVALID |
    SafetyCauseSet::PREMATURE_UNDOCK |
    SafetyCauseSet::PACKML_ABORT | SafetyCauseSet::GRIPPER_FAILURE;
  return causes != SafetyCauseSet::NONE && (causes & ~allowed) == 0;
}

bool SafetyCore::is_known_amr_state(uint8_t value)
{
  return value == AMRDockingState::AMR_DOCKING_DOCKED ||
         value == AMRDockingState::AMR_DOCKING_UNDOCKED;
}

bool SafetyCore::is_known_packml_state(uint8_t value)
{
  return value >= PackMLState::PACKML_STATE_IDLE && value <= PackMLState::PACKML_STATE_ABORTED;
}

bool SafetyCore::is_known_motion_state(uint8_t value)
{
  return value >= MotionStatus::MOTION_STATE_IDLE && value <= MotionStatus::MOTION_STATE_FAULTED;
}

}  // namespace arm_cell_safety_fsm
