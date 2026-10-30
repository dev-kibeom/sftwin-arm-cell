#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>

#include <arm_cell_interfaces/action/execute_task.hpp>
#include <arm_cell_interfaces/msg/mission_exit_reason.hpp>
#include <arm_cell_interfaces/msg/motion_status.hpp>
#include <arm_cell_interfaces/msg/motion_task_phase.hpp>
#include <arm_cell_interfaces/msg/motion_task_type.hpp>
#include <arm_cell_interfaces/msg/vision_diagnostic.hpp>

#include "arm_cell_orchestration/cycle_coordinator.hpp"

namespace arm_cell_orchestration::detail
{

constexpr auto kDiagnosticMotionStatusFreshnessTimeout = std::chrono::milliseconds(1000);

struct QueuedOrchestrationRecord
{
  CycleCoordinator::DiagnosticRecord decision;
  std::array<std::uint8_t, 16> execute_cycle_id{};
  std::array<std::uint8_t, 16> delivery_id{};
  std::array<char, 128> target_id{};
  std::uint8_t target_id_size{0};
  std::int64_t enqueue_steady_ns{0};
  std::uint64_t sequence{0};
  std::array<char, 64> submission_id{};
  std::array<char, 64> action_goal_uuid{};
  std::array<char, 32> action_event{};
  std::array<char, 24> action_kind{};
  std::array<char, 24> action_terminal{};
  int action_outcome{-1};
  int material_placed{-1};
};

inline std::optional<CycleCoordinator::DiagnosticRecord> release_attempt_diagnostic(
  std::uint8_t task_type, const arm_cell_interfaces::action::ExecuteTask::Feedback & feedback,
  std::string_view motion_execution_id)
{
  using Feedback = arm_cell_interfaces::action::ExecuteTask::Feedback;
  const bool is_initial_attempt = feedback.release_attempt == 1 &&
    feedback.release_last_failure == Feedback::RELEASE_FAILURE_NONE;
  const bool is_timeout_retry = feedback.release_attempt > 1 &&
    feedback.release_last_failure == Feedback::RELEASE_FAILURE_CONFIRMATION_TIMEOUT;
  if (task_type != arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE ||
    feedback.phase.value != arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_GRIPPER ||
    feedback.release_attempt == 0 || feedback.release_attempt > 4 ||
    feedback.release_retry_budget != 3 || (!is_initial_attempt && !is_timeout_retry) ||
    motion_execution_id.empty())
  {
    return std::nullopt;
  }

  CycleCoordinator::DiagnosticRecord record;
  record.event = CycleCoordinator::DiagnosticEvent::MOTION_GRIPPER_RELEASE_ATTEMPT;
  record.failure_domain = CycleCoordinator::DiagnosticFailureDomain::MOTION;
  record.reason = is_timeout_retry ? CycleCoordinator::DiagnosticReason::RELEASE_CONFIRMATION_TIMEOUT :
    CycleCoordinator::DiagnosticReason::NONE;
  record.observed_motion_phase = feedback.phase.value;
  record.motion_task_type = task_type;
  record.attempt = feedback.release_attempt;
  record.retry_budget = feedback.release_retry_budget;
  record.result_detail_size = static_cast<std::uint8_t>(std::min(
    feedback.diagnostic_detail.size(), record.result_detail.size()));
  std::copy_n(feedback.diagnostic_detail.begin(), record.result_detail_size, record.result_detail.begin());
  record.motion_execution_id_size = static_cast<std::uint8_t>(std::min(
    motion_execution_id.size(), record.motion_execution_id.size()));
  std::copy_n(motion_execution_id.begin(), record.motion_execution_id_size,
    record.motion_execution_id.begin());
  return record;
}

inline std::string json_escape(std::string_view value)
{
  std::ostringstream escaped;
  for (const unsigned char character : value) {
    switch (character) {
      case '"': escaped << "\\\""; break;
      case '\\': escaped << "\\\\"; break;
      case '\n': escaped << "\\n"; break;
      case '\r': escaped << "\\r"; break;
      case '\t': escaped << "\\t"; break;
      default:
        if (character < 0x20) {
          escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                  << static_cast<unsigned int>(character) << std::dec;
        } else {
          escaped << character;
        }
    }
  }
  return escaped.str();
}

inline const char * diagnostic_event_name(CycleCoordinator::DiagnosticEvent event)
{
  switch (event) {
    case CycleCoordinator::DiagnosticEvent::CANCELLATION: return "cancellation_propagation";
    case CycleCoordinator::DiagnosticEvent::RECOVERY: return "recovery_observation";
    case CycleCoordinator::DiagnosticEvent::RETRACT: return "retract_decision";
    case CycleCoordinator::DiagnosticEvent::RETRY: return "retry_decision";
    case CycleCoordinator::DiagnosticEvent::VISION_IDLE_WAIT: return "vision_idle_wait_outcome";
    case CycleCoordinator::DiagnosticEvent::MOTION_FEEDBACK_GAP: return "motion_feedback_gap";
    case CycleCoordinator::DiagnosticEvent::MOTION_FEEDBACK_RECOVERED: return "motion_feedback_recovered";
    case CycleCoordinator::DiagnosticEvent::MOTION_GOAL_ACCEPTED: return "motion_goal_accepted";
    case CycleCoordinator::DiagnosticEvent::MOTION_ACTION_RESULT: return "motion_action_result";
    case CycleCoordinator::DiagnosticEvent::MOTION_ACTION_COMMUNICATION: return
        "motion_action_communication_unknown";
    case CycleCoordinator::DiagnosticEvent::MOTION_GRIPPER_RELEASE_ATTEMPT: return
        "motion_gripper_release_attempt";
    case CycleCoordinator::DiagnosticEvent::MISSION_STARTED: return "mission_started";
    case CycleCoordinator::DiagnosticEvent::MISSION_OUTCOME: return "mission_outcome";
    case CycleCoordinator::DiagnosticEvent::BT_NODE_STATUS: return "behavior_tree_node_status";
    case CycleCoordinator::DiagnosticEvent::ACTION_LIFECYCLE: return "action_lifecycle";
    default: return "unknown";
  }
}

inline const char * diagnostic_phase_name(CycleCoordinator::DiagnosticPhase phase)
{
  switch (phase) {
    case CycleCoordinator::DiagnosticPhase::VISION: return "VISION";
    case CycleCoordinator::DiagnosticPhase::PICK: return "PICK";
    default: return "NONE";
  }
}

inline const char * diagnostic_decision_name(CycleCoordinator::DiagnosticDecision decision)
{
  switch (decision) {
    case CycleCoordinator::DiagnosticDecision::RETRY: return "retry";
    case CycleCoordinator::DiagnosticDecision::STOP: return "stop";
    case CycleCoordinator::DiagnosticDecision::ALLOWED: return "allowed";
    case CycleCoordinator::DiagnosticDecision::DENIED: return "denied";
    default: return "none";
  }
}

inline const char * diagnostic_reason_name(CycleCoordinator::DiagnosticReason reason)
{
  using Reason = CycleCoordinator::DiagnosticReason;
  switch (reason) {
    case Reason::MISSION_INTERRUPTION: return "mission_interruption";
    case Reason::HIGHER_STOP_SEVERITY: return "higher_stop_severity";
    case Reason::EMERGENCY_STOP: return "emergency_stop";
    case Reason::RECOVERY_CAPABILITY_REMOVED: return "recovery_capability_removed";
    case Reason::HOLDING_INVALID: return "holding_state_unknown_or_invalid";
    case Reason::BUDGET_AVAILABLE: return "budget_available";
    case Reason::RETRY_BUDGET_EXHAUSTED: return "retry_budget_exhausted";
    case Reason::NON_RETRYABLE: return "non_retryable";
    case Reason::RESULT_UNAVAILABLE: return "result_unavailable";
    case Reason::RETRY_ADMISSION: return "fresh_released_idle_safety_permitted";
    case Reason::MOTION_STATUS_UNAVAILABLE: return "motion_status_unavailable_or_stale";
    case Reason::MOTION_NOT_IDLE: return "motion_not_idle";
    case Reason::MOTION_ACTIVE: return "motion_execution_active";
    case Reason::HOLDING_NOT_RELEASED: return "holding_not_released";
    case Reason::RETRY_ADMISSION_FAILED: return "retry_admission_guard_failed";
    case Reason::RETRY_SAFETY_UNAVAILABLE: return "retry_safety_unavailable";
    case Reason::RETRY_SAFETY_NOT_NORMAL: return "retry_safety_not_normal";
    case Reason::RETRY_STOP_NOT_CONFIRMED: return "retry_stop_not_confirmed";
    case Reason::RETRY_CANCEL_REQUESTED: return "retry_canceled_by_caller";
    case Reason::RECOVERY_AUTHORIZED_AND_STOPPED: return "recovery_capability_and_motion_stopped";
    case Reason::MISSION_CANCELLED: return "mission_cancelled";
    case Reason::SAFETY_CAPABILITY_WITHDRAWN: return "safety_capability_withdrawn";
    case Reason::UNEXPECTED_BUSY: return "unexpected_busy";
    case Reason::FEEDBACK_MISSING: return "feedback_missing";
    case Reason::FEEDBACK_RECOVERED: return "feedback_recovered";
    case Reason::ACTION_COMMUNICATION_UNKNOWN: return "action_communication_unknown";
    case Reason::RELEASE_CONFIRMATION_TIMEOUT: return "release_confirmation_timeout";
    default: return "none";
  }
}

inline const char * failure_domain_name(CycleCoordinator::DiagnosticFailureDomain domain)
{
  switch (domain) {
    case CycleCoordinator::DiagnosticFailureDomain::VISION: return "VISION";
    case CycleCoordinator::DiagnosticFailureDomain::MOTION: return "MOTION";
    default: return "NONE";
  }
}

inline const char * failure_code_name(const CycleCoordinator::DiagnosticRecord & record)
{
  if (record.failure_domain == CycleCoordinator::DiagnosticFailureDomain::VISION) {
    using Code = arm_cell_interfaces::msg::DetectTargetResultCode;
    switch (record.failure_code) {
      case Code::DETECT_RESULT_TIMEOUT: return "DETECT_RESULT_TIMEOUT";
      case Code::DETECT_RESULT_TEMPORARY_INVALID_TARGET: return
          "DETECT_RESULT_TEMPORARY_INVALID_TARGET";
      case Code::DETECT_RESULT_SENSOR_ERROR: return "DETECT_RESULT_SENSOR_ERROR";
      case Code::DETECT_RESULT_TF_ERROR: return "DETECT_RESULT_TF_ERROR";
      case Code::DETECT_RESULT_GEOMETRY_ERROR: return "DETECT_RESULT_GEOMETRY_ERROR";
      case Code::DETECT_RESULT_INVALID_RESULT: return "DETECT_RESULT_INVALID_RESULT";
      default: return "UNKNOWN_VISION_RESULT";
    }
  }
  if (record.failure_domain == CycleCoordinator::DiagnosticFailureDomain::MOTION) {
    using Code = arm_cell_interfaces::msg::MotionTaskResultCode;
    switch (record.failure_code) {
      case Code::TASK_RESULT_SUCCESS: return "TASK_RESULT_SUCCESS";
      case Code::TASK_RESULT_INVALID_GOAL: return "TASK_RESULT_INVALID_GOAL";
      case Code::TASK_RESULT_IK_UNREACHABLE: return "TASK_RESULT_IK_UNREACHABLE";
      case Code::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED: return
          "TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED";
      case Code::TASK_RESULT_PATH_OBSTRUCTED: return "TASK_RESULT_PATH_OBSTRUCTED";
      case Code::TASK_RESULT_GRASP_FAILURE: return "TASK_RESULT_GRASP_FAILURE";
      case Code::TASK_RESULT_JOINT_PROGRESS_STALLED: return "TASK_RESULT_JOINT_PROGRESS_STALLED";
      case Code::TASK_RESULT_CANCELED: return "TASK_RESULT_CANCELED";
      case Code::TASK_RESULT_SAFETY_PREEMPTED: return "TASK_RESULT_SAFETY_PREEMPTED";
      case Code::TASK_RESULT_BACKEND_UNAVAILABLE: return "TASK_RESULT_BACKEND_UNAVAILABLE";
      case Code::TASK_RESULT_PERMISSION_DENIED: return "TASK_RESULT_PERMISSION_DENIED";
      case Code::TASK_RESULT_BACKEND_ERROR: return "TASK_RESULT_BACKEND_ERROR";
      case Code::TASK_RESULT_BUSY: return "TASK_RESULT_BUSY";
      default: return "UNKNOWN_MOTION_RESULT";
    }
  }
  return "NONE";
}

inline const char * motion_task_type_name_for_diagnostic(std::uint8_t value)
{
  using Type = arm_cell_interfaces::msg::MotionTaskType;
  switch (value) {
    case Type::TASK_TYPE_PICK: return "TASK_TYPE_PICK";
    case Type::TASK_TYPE_PLACE: return "TASK_TYPE_PLACE";
    case Type::TASK_TYPE_GO_HOME: return "TASK_TYPE_GO_HOME";
    case Type::TASK_TYPE_RETRACT: return "TASK_TYPE_RETRACT";
    default: return "TASK_TYPE_UNSPECIFIED";
  }
}

inline const char * motion_phase_name_for_diagnostic(std::uint8_t value)
{
  using Phase = arm_cell_interfaces::msg::MotionTaskPhase;
  switch (value) {
    case Phase::TASK_PHASE_VALIDATING: return "TASK_PHASE_VALIDATING";
    case Phase::TASK_PHASE_PLANNING: return "TASK_PHASE_PLANNING";
    case Phase::TASK_PHASE_APPROACHING: return "TASK_PHASE_APPROACHING";
    case Phase::TASK_PHASE_EXECUTING: return "TASK_PHASE_EXECUTING";
    case Phase::TASK_PHASE_GRIPPER: return "TASK_PHASE_GRIPPER";
    case Phase::TASK_PHASE_ATTACHING: return "TASK_PHASE_ATTACHING";
    case Phase::TASK_PHASE_DETACHING: return "TASK_PHASE_DETACHING";
    case Phase::TASK_PHASE_RETRACTING: return "TASK_PHASE_RETRACTING";
    case Phase::TASK_PHASE_COMPLETING: return "TASK_PHASE_COMPLETING";
    default: return "TASK_PHASE_UNKNOWN";
  }
}

inline const char * mission_exit_name(std::uint8_t value)
{
  using Reason = arm_cell_interfaces::msg::MissionExitReason;
  switch (value) {
    case Reason::MISSION_EXIT_VISION_ERROR: return "MISSION_EXIT_VISION_ERROR";
    case Reason::MISSION_EXIT_MOTION_ERROR: return "MISSION_EXIT_MOTION_ERROR";
    case Reason::MISSION_EXIT_CANCELED: return "MISSION_EXIT_CANCELED";
    case Reason::MISSION_EXIT_SAFETY_PREEMPTED: return "MISSION_EXIT_SAFETY_PREEMPTED";
    case Reason::MISSION_EXIT_QUANTITY_REACHED: return "MISSION_EXIT_QUANTITY_REACHED";
    default: return "PENDING_OR_OTHER";
  }
}

inline std::string serialize_diagnostic(const QueuedOrchestrationRecord & queued, std::uint64_t sequence)
{
  const auto & record = queued.decision;
  std::ostringstream out;
  out << "{\"schema_version\":1,\"local_sequence\":" << sequence
      << ",\"time_basis\":\"local_steady_enqueue\",\"component\":\"orchestration\",\"event\":\""
      << diagnostic_event_name(record.event) << "\",\"enqueue_steady_ns\":"
      << queued.enqueue_steady_ns << ",\"execute_cycle_id\":\"";
  out << std::hex << std::setfill('0');
  for (const auto byte : queued.execute_cycle_id) {
    out << std::setw(2) << static_cast<unsigned>(byte);
  }
  out << '"' << std::dec;
  if (queued.action_event[0] != '\0') {
    out << ",\"submission_id\":\"" << json_escape(queued.submission_id.data())
        << "\",\"action_goal_uuid\":\"" << json_escape(queued.action_goal_uuid.data())
        << "\",\"goal_uuid_observed\":" << (queued.action_goal_uuid[0] != '\0' ? "true" : "false")
        << ",\"action_event\":\"" << json_escape(queued.action_event.data())
        << "\",\"action_kind\":\"" << json_escape(queued.action_kind.data())
        << "\",\"action_terminal\":\"" << json_escape(queued.action_terminal.data())
        << "\",\"action_outcome\":" << queued.action_outcome
        << ",\"material_placed\":" << queued.material_placed;
  }
  out << ",\"delivery_id\":\"" << std::hex;
  for (const auto byte : queued.delivery_id) {
    out << std::setw(2) << static_cast<unsigned>(byte);
  }
  out << std::dec << "\",\"target_id\":\""
      << json_escape(std::string_view(queued.target_id.data(), queued.target_id_size))
      << "\",\"phase\":\"" << diagnostic_phase_name(record.phase)
      << "\",\"motion_task_type\":\""
      << motion_task_type_name_for_diagnostic(record.motion_task_type) << '"'
      << ",\"decision\":\"" << diagnostic_decision_name(record.decision)
      << "\",\"reason\":\"" << diagnostic_reason_name(record.reason)
      << "\",\"failure_domain\":\"" << failure_domain_name(record.failure_domain) << '\"'
      << ",\"failure_code\":";
  if (record.event == CycleCoordinator::DiagnosticEvent::MOTION_ACTION_COMMUNICATION ||
    record.event == CycleCoordinator::DiagnosticEvent::MISSION_STARTED ||
    (record.event == CycleCoordinator::DiagnosticEvent::MISSION_OUTCOME &&
    !record.result_available)) {
    out << "null,\"failure_name\":\"UNAVAILABLE\"";
  } else {
    out << static_cast<unsigned>(record.failure_code)
        << ",\"failure_name\":\"" << failure_code_name(record) << '\"';
  }
  out
      << ",\"result_available\":" << (record.result_available ? "true" : "false")
      << ",\"original_failure_code\":" << static_cast<unsigned>(record.original_failure_code)
      << ",\"attempt\":" << record.attempt << ",\"retry_budget\":" << record.retry_budget
      << ",\"requested_quantity\":" << record.requested_quantity
      << ",\"processed_count\":" << record.processed_count
      << ",\"retryable\":" << (record.retryable ? "true" : "false")
      << ",\"final_result\":" << static_cast<unsigned>(record.final_result)
      << ",\"final_result_name\":\"" << mission_exit_name(record.final_result) << '\"'
      << ",\"mission_exit_reason\":" << static_cast<unsigned>(record.mission_exit_reason)
      << ",\"mission_exit_reason_name\":\"" << mission_exit_name(record.mission_exit_reason) << '\"'
      << ",\"safety_valid\":" << (record.safety_valid ? "true" : "false")
      << ",\"capability\":" << static_cast<unsigned>(record.capability)
      << ",\"selected_stop_mode\":" << static_cast<unsigned>(record.selected_stop_mode)
      << ",\"has_motion_status\":" << (record.has_motion_status ? "true" : "false")
      << ",\"motion_state\":" << static_cast<unsigned>(record.motion_state)
      << ",\"motion_active\":" << (record.motion_active ? "true" : "false")
      << ",\"has_active_execution\":" << (record.has_active_execution ? "true" : "false")
      << ",\"backend_inactivity_confirmed\":" <<
    (record.backend_inactivity_confirmed ? "true" : "false")
      << ",\"holding_state\":" << static_cast<unsigned>(record.holding_state)
      << ",\"episode_retract_allowance\":" <<
    (record.episode_retract_allowance ? "true" : "false")
      << ",\"cancellation_requested\":" << (record.cancellation_requested ? "true" : "false")
      << ",\"motion_execution_id\":\""
      << json_escape(
    std::string_view(
      record.motion_execution_id.data(),
      record.motion_execution_id_size))
      << "\"";
  if (record.event == CycleCoordinator::DiagnosticEvent::MOTION_GOAL_ACCEPTED ||
    record.event == CycleCoordinator::DiagnosticEvent::MOTION_ACTION_RESULT ||
    record.event == CycleCoordinator::DiagnosticEvent::MOTION_GRIPPER_RELEASE_ATTEMPT ||
    record.event == CycleCoordinator::DiagnosticEvent::MOTION_ACTION_COMMUNICATION ||
    record.event == CycleCoordinator::DiagnosticEvent::MISSION_OUTCOME ||
    record.event == CycleCoordinator::DiagnosticEvent::MOTION_FEEDBACK_GAP ||
    record.event == CycleCoordinator::DiagnosticEvent::MOTION_FEEDBACK_RECOVERED ||
    (record.event == CycleCoordinator::DiagnosticEvent::CANCELLATION &&
    record.failure_domain == CycleCoordinator::DiagnosticFailureDomain::MOTION))
  {
    out << ",\"motion_result_detail\":\""
        << json_escape(std::string_view(record.result_detail.data(), record.result_detail_size))
        << '"';
    if (record.motion_action_terminal_size > 0U) {
      out << ",\"motion_action_terminal\":\""
          << json_escape(std::string_view(
        record.motion_action_terminal.data(), record.motion_action_terminal_size)) << '"';
    }
    if (record.motion_status_execution_id_size > 0U) {
      out << ",\"motion_status_execution_id\":\""
          << json_escape(std::string_view(record.motion_status_execution_id.data(),
            record.motion_status_execution_id_size)) << '"';
    }
    if (record.event == CycleCoordinator::DiagnosticEvent::MOTION_ACTION_COMMUNICATION) {
      out << ",\"communication_stage\":\""
          << json_escape(std::string_view(record.wait_reason.data(), record.wait_reason_size))
          << '"';
    }
  }
  if (record.event == CycleCoordinator::DiagnosticEvent::RETRY &&
    record.phase == CycleCoordinator::DiagnosticPhase::PICK &&
    record.has_motion_status)
  {
    out << ",\"motion_status_diagnostic_detail\":\""
        << json_escape(std::string_view(record.result_detail.data(), record.result_detail_size))
        << '"';
  }
  if (record.event == CycleCoordinator::DiagnosticEvent::BT_NODE_STATUS) {
    out << ",\"node\":\""
        << json_escape(std::string_view(record.tree_node.data(), record.tree_node_size))
        << "\",\"previous_status\":\""
        << json_escape(std::string_view(
      record.tree_previous_status.data(), record.tree_previous_status_size))
        << "\",\"current_status\":\""
        << json_escape(std::string_view(
      record.tree_current_status.data(), record.tree_current_status_size)) << '"';
  }
  if (record.event == CycleCoordinator::DiagnosticEvent::VISION_IDLE_WAIT)
  {
    out << ",\"wait_reason\":\""
        << json_escape(std::string_view(record.wait_reason.data(), record.wait_reason_size))
        << "\",\"wait_started\":" << (record.wait_started ? "true" : "false");
    if (record.wait_started) {
      out << ",\"wait_budget_ms\":30000"
          << ",\"wait_start_steady_ns\":" << record.wait_start_steady_ns
          << ",\"wait_deadline_steady_ns\":" << record.wait_deadline_steady_ns
          << ",\"wait_decision_steady_ns\":" << record.wait_decision_steady_ns
          << ",\"wait_elapsed_ns\":" << record.wait_elapsed_ns;
    }
    if (record.event == CycleCoordinator::DiagnosticEvent::VISION_IDLE_WAIT &&
      record.wait_started)
    {
      out << ",\"requested_vision_generation\":" << record.requested_vision_generation
          << ",\"observed_vision_generation\":" << record.observed_vision_generation
          << ",\"vision_state\":\""
          << (record.vision_state == arm_cell_interfaces::msg::VisionDiagnostic::IDLE ? "IDLE" :
    record.vision_state == arm_cell_interfaces::msg::VisionDiagnostic::DETECTING ?
    "DETECTING" : "UNKNOWN")
          << "\",\"vision_observation_received\":"
          << (record.vision_observation_received ? "true" : "false")
          << ",\"vision_observation_age_ns\":" << record.vision_observation_age_ns;
    }
  }
  if (record.event == CycleCoordinator::DiagnosticEvent::MOTION_ACTION_RESULT ||
    record.event == CycleCoordinator::DiagnosticEvent::MOTION_GRIPPER_RELEASE_ATTEMPT ||
    record.event == CycleCoordinator::DiagnosticEvent::MOTION_ACTION_COMMUNICATION ||
    record.event == CycleCoordinator::DiagnosticEvent::MOTION_FEEDBACK_GAP ||
    record.event == CycleCoordinator::DiagnosticEvent::MOTION_FEEDBACK_RECOVERED)
  {
    out << ",\"motion_phase\":\""
        << motion_phase_name_for_diagnostic(record.observed_motion_phase) << '"'
        << ",\"motion_observation_age_ns\":" << record.motion_observation_age_ns
      << ",\"motion_status_generation\":" << record.motion_status_generation
      << ",\"motion_status_stamp_ns\":" << record.motion_status_stamp_ns
        << ",\"motion_status_fresh\":" <<
      (record.motion_observation_age_ns >= 0 && record.motion_observation_age_ns <=
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      kDiagnosticMotionStatusFreshnessTimeout).count() ? "true" : "false");
    const auto motion_status_observation = !record.has_motion_status ? "missing" :
      record.motion_observation_age_ns > kDiagnosticMotionStatusFreshnessTimeout.count() * 1000000LL ?
      "stale" : record.motion_active || record.has_active_execution ? "active" :
      record.backend_inactivity_confirmed ? "inactive" : "uncertain";
    out << ",\"motion_status_observation\":\"" << motion_status_observation << '"';
    if (record.event != CycleCoordinator::DiagnosticEvent::MOTION_ACTION_RESULT) {
      out << ",\"feedback_gap_elapsed_ns\":" << record.wait_elapsed_ns;
    } else {
      out << ",\"motion_action_elapsed_ns\":";
      if (record.motion_action_elapsed_ns >= 0) {
        out << record.motion_action_elapsed_ns;
      } else {
        out << "null";
      }
    }
  }
  out << "}";
  return out.str();
}


}  // namespace arm_cell_orchestration::detail
