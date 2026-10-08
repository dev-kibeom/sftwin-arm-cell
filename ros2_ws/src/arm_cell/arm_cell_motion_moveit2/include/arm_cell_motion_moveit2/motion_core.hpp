#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <arm_cell_interfaces/action/execute_task.hpp>
#include <arm_cell_interfaces/msg/motion_status.hpp>
#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>
#include <arm_cell_interfaces/msg/safety_state.hpp>
#include <arm_cell_interfaces/msg/stop_mode.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include "arm_cell_motion_moveit2/motion_backend.hpp"
#include "arm_cell_motion_moveit2/motion_tuning.hpp"
#include "arm_cell_motion_moveit2/task_executor.hpp"

namespace arm_cell_motion_moveit2
{

using ExecuteTask = arm_cell_interfaces::action::ExecuteTask;
using MotionStatus = arm_cell_interfaces::msg::MotionStatus;
using MotionTaskType = arm_cell_interfaces::msg::MotionTaskType;
using MotionTaskResultCode = arm_cell_interfaces::msg::MotionTaskResultCode;
using SafetyState = arm_cell_interfaces::msg::SafetyState;
using UUID = unique_identifier_msgs::msg::UUID;

inline UUID make_uuid(uint8_t seed)
{
  UUID uuid;
  uuid.uuid.fill(seed);
  return uuid;
}

class MotionCore
{
public:
  using Clock = std::chrono::steady_clock;

  explicit MotionCore(
    std::shared_ptr<MotionBackend> backend,
    std::chrono::milliseconds safety_state_timeout = std::chrono::milliseconds(500),
    MotionGeometry geometry = {},
    std::chrono::milliseconds holding_confirmation_timeout = std::chrono::milliseconds(500));

  MotionStatus status() const;
  void update_safety_state(const SafetyState & state, Clock::time_point receipt_time);
  void update_object_state(MotionObjectState object_state);
  uint8_t validate_goal(
    const ExecuteTask::Goal & goal,
    Clock::time_point now = Clock::now()) const;
  uint8_t reserve_goal(const UUID & goal_uuid, const ExecuteTask::Goal & goal);
  uint8_t accept_goal(const UUID & goal_uuid, const ExecuteTask::Goal & goal);
  uint8_t accept_goal_with_diagnostic(
    const UUID & goal_uuid, const ExecuteTask::Goal & goal, std::string & diagnostic_detail);
  uint8_t accept_goal(
    const UUID & goal_uuid, const ExecuteTask::Goal & goal, Clock::time_point now);
  bool request_cancel();
  bool request_cancel(const UUID & goal_uuid);
  bool request_stop(const UUID & request_uuid);
  bool request_stop(const UUID & request_uuid, uint8_t stop_mode);
  void refresh_stop_state();
  bool set_runtime_tuning(
    const MotionTuning & tuning, const MotionGeometry & geometry,
    std::chrono::milliseconds holding_confirmation_timeout, bool idle_mutable = true);

private:
  static bool is_supported_task(uint8_t task_type);
  static bool is_known_safety_state(uint8_t safety_state);
  static bool is_known_capability(uint8_t capability);
  static bool capability_allows_task(uint8_t capability, uint8_t task_type);
  uint8_t accept_goal_impl(
    const UUID & goal_uuid, const ExecuteTask::Goal & goal,
    Clock::time_point acceptance_time, bool refresh_pre_submit_time,
    std::string * diagnostic_detail = nullptr);
  uint8_t validate_goal_locked(
    const ExecuteTask::Goal & goal, Clock::time_point now,
    std::string * diagnostic_detail = nullptr) const;
  uint8_t interruption_result_locked(uint8_t execution_result) const;
  uint8_t current_interruption_result() const;
  void clear_execution_interruption_locked();

  std::shared_ptr<MotionBackend> backend_;
  TaskExecutor task_executor_;
  std::chrono::milliseconds safety_state_timeout_;
  mutable std::mutex mutex_;
  MotionStatus status_;
  SafetyState safety_state_;
  Clock::time_point safety_state_receipt_time_{};
  bool has_safety_state_{false};
  bool goal_submission_pending_{false};
  bool task_execution_in_progress_{false};
  UUID pending_goal_uuid_{};
  bool stop_requested_{false};
  uint8_t requested_stop_mode_{0};
  enum class InterruptionReason : uint8_t {NONE, CALLER_CANCELED, SAFETY_PREEMPTED};
  InterruptionReason interruption_reason_{InterruptionReason::NONE};
  MotionObjectState object_state_{MotionObjectState::NO_OBJECT};
  MotionTuning runtime_tuning_{};
  MotionGeometry runtime_geometry_{};
  std::chrono::milliseconds holding_confirmation_timeout_{500};
  MotionTuning active_tuning_snapshot_{};
  MotionGeometry active_geometry_snapshot_{};
  std::chrono::milliseconds active_holding_confirmation_timeout_{500};
};

}  // namespace arm_cell_motion_moveit2
