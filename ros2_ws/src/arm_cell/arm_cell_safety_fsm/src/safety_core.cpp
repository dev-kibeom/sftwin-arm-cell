#include "arm_cell_safety_fsm/safety_core.hpp"

#include <cmath>
#include <string>

namespace arm_cell_safety_fsm
{

namespace
{
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

std::string format_input_status(
  const std::array<SafetyInputDiagnostic, static_cast<std::size_t>(SafetyInput::COUNT)> & inputs)
{
  std::string result;
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    if (i != 0) {
      result += ',';
    }
    const auto & diagnostic = inputs[i];
    result += safety_input_name(static_cast<SafetyInput>(i));
    result += "{received=";
    result += diagnostic.received ? "true" : "false";
    result += ",message_valid=";
    result += diagnostic.message_valid ? "true" : "false";
    result += ",source_stamp_plausible=";
    result += diagnostic.source_timestamp_plausible ? "true" : "false";
    result += ",reason=";
    result += safety_input_reason_name(diagnostic.reason);
    result += '}';
  }
  return result;
}
}  // namespace

SafetyCore::SafetyCore(std::chrono::milliseconds freshness_timeout)
: tracker_(freshness_timeout)
{
  inputs_.motion.execution_state = MotionStatus::MOTION_STATE_UNKNOWN;
}

void SafetyCore::update_hardware(
  const SafetyHardwareState & message, Clock::time_point receipt_time,
  bool source_timestamp_plausible, std::int64_t source_stamp_ns,
  std::int64_t source_evaluation_ros_time_ns, std::int64_t source_maximum_age_ns)
{
  std::lock_guard<std::mutex> lock(mutex_);
  inputs_.hardware = message;
  tracker_.record(
    SafetyInput::HARDWARE, message.valid, source_timestamp_plausible, receipt_time,
    source_stamp_ns, source_evaluation_ros_time_ns, source_maximum_age_ns);
}

void SafetyCore::update_motion(
  const MotionStatus & message, Clock::time_point receipt_time,
  bool source_timestamp_plausible, std::int64_t source_stamp_ns,
  std::int64_t source_evaluation_ros_time_ns, std::int64_t source_maximum_age_ns)
{
  std::lock_guard<std::mutex> lock(mutex_);
  inputs_.motion = message;
  const bool stop_status_consistent =
    message.execution_state != MotionStatus::MOTION_STATE_STOPPED ||
    (!message.execution_active && !message.has_active_execution &&
    message.backend_inactivity_confirmed);
  tracker_.record(
    SafetyInput::MOTION, is_known_motion_state(message.execution_state) && stop_status_consistent,
    source_timestamp_plausible, receipt_time, source_stamp_ns,
    source_evaluation_ros_time_ns, source_maximum_age_ns);
}

SafetyState SafetyCore::evaluate(Clock::time_point now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return evaluate_locked(now);
}

std::array<SafetyInputDiagnostic, static_cast<std::size_t>(SafetyInput::COUNT)>
SafetyCore::input_diagnostics(Clock::time_point now) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return tracker_.diagnostics(now);
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
  if (!last_state_.required_inputs_fresh) {
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
  // A STOPPING sample with no active control execution or Action ownership
  // permits an operator to clear the latch. evaluate_locked() continues to
  // hold capability at NONE until physical stop confirmation arrives.
  if (inputs_.motion.execution_state == MotionStatus::MOTION_STATE_STOPPING) {
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
    state.safety_state = SafetyState::SAFETY_STATE_UNKNOWN;
    state.valid = false;
    state.motion_capability.value = MotionCapability::MOTION_NONE;
  } else {
    state.valid = true;
    state.safety_state = SafetyState::SAFETY_STATE_SAFE;
    state.motion_capability.value = MotionCapability::MOTION_NORMAL;
  }

  const uint32_t causes_to_latch = active_causes;
  latched_causes_ |= causes_to_latch;
  if (causes_to_latch != SafetyCauseSet::NONE && motion_execution_active) {
    recovery_stop_completion_required_ = true;
  }
  if (!fresh && motion_execution_active) {
    recovery_stop_completion_required_ = true;
  }
  if (recovery_stop_completion_required_ &&
    inputs_.motion.execution_state == MotionStatus::MOTION_STATE_STOPPED &&
    !inputs_.motion.execution_active && !inputs_.motion.has_active_execution &&
    inputs_.motion.backend_inactivity_confirmed && fresh)
  {
    recovery_stop_completion_required_ = false;
  }
  state.active_causes.value = active_causes;
  state.latched_causes.value = latched_causes_;
  const auto selected_causes = active_causes | latched_causes_;
  if (selected_causes & (SafetyCauseSet::E_STOP | SafetyCauseSet::STO)) {
    state.selected_stop_mode.value = StopMode::STOP_MODE_EMERGENCY;
  } else if ((selected_causes & (SafetyCauseSet::COLLISION | SafetyCauseSet::OTHER)) ||
    (!fresh && motion_execution_active))
  {
    state.selected_stop_mode.value = StopMode::STOP_MODE_IMMEDIATE;
  } else {
    state.selected_stop_mode.value = StopMode::STOP_MODE_CONTROLLED;
  }

  if (active_causes & (SafetyCauseSet::E_STOP | SafetyCauseSet::STO)) {
    state.safety_state = SafetyState::SAFETY_STATE_EMERGENCY_STOPPED;
    state.motion_capability.value = MotionCapability::MOTION_NONE;
  } else if (active_causes != SafetyCauseSet::NONE ||
    (!fresh && motion_execution_active))
  {
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
  if (recovery_stop_completion_required_) {
    state.motion_capability.value = MotionCapability::MOTION_NONE;
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
    state.max_velocity_scale = 1.0F;
    state.max_acceleration_scale = 1.0F;
  }
  if (state.motion_capability.value == MotionCapability::MOTION_NONE) {
    state.diagnostic_detail = std::string("permission_denied motion_state=") +
      motion_state_name(inputs_.motion.execution_state) +
      " hardware_valid=" + (inputs_.hardware.valid ? "true" : "false") +
      " e_stop_active=" + (inputs_.hardware.e_stop_active ? "true" : "false") +
      " sto_active=" + (inputs_.hardware.sto_active ? "true" : "false");
    if (!fresh) {
      state.diagnostic_detail += " deny_reason=required_inputs_not_fresh";
    } else if (active_causes != SafetyCauseSet::NONE) {
      state.diagnostic_detail += " deny_reason=active_safety_cause";
    } else if (!fresh) {
      state.diagnostic_detail += " deny_reason=required_safety_evidence_unavailable";
    } else if (latched_causes_ != SafetyCauseSet::NONE) {
      state.diagnostic_detail += " deny_reason=recovery_not_eligible_or_motion_not_inactive";
    } else if (inputs_.motion.execution_state == MotionStatus::MOTION_STATE_FAULTED) {
      state.diagnostic_detail += " deny_reason=motion_faulted";
    } else {
      state.diagnostic_detail += " deny_reason=normal_capability_not_granted";
    }
  }
  if (!state.diagnostic_detail.empty()) {
    state.diagnostic_detail += ' ';
  }
  state.diagnostic_detail += "required_input_status=[" +
    format_input_status(tracker_.diagnostics(now)) + ']';
  last_state_ = state;
  has_last_state_ = true;
  return state;
}

bool SafetyCore::reset(
  bool operator_clear, Clock::time_point now, std::string * diagnostic_detail)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto state = evaluate_locked(now);
  if (!operator_clear) {
    if (diagnostic_detail) {*diagnostic_detail = "operator_acknowledgement_missing";}
    return false;
  }
  if (!state.required_inputs_fresh) {
    if (diagnostic_detail) {*diagnostic_detail = "required_inputs_not_fresh";}
    return false;
  }
  if (inputs_.hardware.e_stop_active) {
    if (diagnostic_detail) {*diagnostic_detail = "e_stop_active";}
    return false;
  }
  if (inputs_.hardware.sto_active) {
    if (diagnostic_detail) {*diagnostic_detail = "sto_active";}
    return false;
  }
  if (state.active_causes.value != 0) {
    if (diagnostic_detail) {*diagnostic_detail = "active_safety_causes_remain";}
    return false;
  }
  if (!motion_inactive_locked()) {
    if (diagnostic_detail) {*diagnostic_detail = "motion_not_inactive";}
    return false;
  }
  if (recovery_stop_completion_required_ &&
    (inputs_.motion.execution_active || inputs_.motion.has_active_execution ||
    (inputs_.motion.execution_state != MotionStatus::MOTION_STATE_STOPPING &&
    (inputs_.motion.execution_state != MotionStatus::MOTION_STATE_STOPPED ||
    !inputs_.motion.backend_inactivity_confirmed))))
  {
    if (diagnostic_detail) {*diagnostic_detail = "stop_completion_not_confirmed";}
    return false;
  }
  if (latched_causes_ == SafetyCauseSet::NONE) {
    if (diagnostic_detail) {*diagnostic_detail = "no_latched_safety_causes";}
    return false;
  }
  if (diagnostic_detail) {diagnostic_detail->clear();}
  latched_causes_ = SafetyCauseSet::NONE;
  recovery_stop_completion_required_ = false;
  return true;
}

bool SafetyCore::is_recovery_eligible(uint32_t causes)
{
  constexpr uint32_t allowed =
    SafetyCauseSet::GRIPPER_FAILURE;
  return causes != SafetyCauseSet::NONE && (causes & ~allowed) == 0;
}

bool SafetyCore::is_known_motion_state(uint8_t value)
{
  return value >= MotionStatus::MOTION_STATE_IDLE && value <= MotionStatus::MOTION_STATE_FAULTED;
}

}  // namespace arm_cell_safety_fsm
