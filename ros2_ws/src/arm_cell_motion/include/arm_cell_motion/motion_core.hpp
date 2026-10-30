#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <string>
#include <vector>

#include <arm_cell_interfaces/action/execute_task.hpp>
#include <arm_cell_interfaces/msg/motion_status.hpp>
#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>
#include <arm_cell_interfaces/msg/safety_state.hpp>
#include <arm_cell_interfaces/msg/stop_mode.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include "arm_cell_motion/motion_backend.hpp"
#include "arm_cell_motion/motion_execution_support.hpp"
#include "arm_cell_motion/motion_port.hpp"
#include "arm_cell_motion/motion_tuning.hpp"

namespace arm_cell_motion
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

class MotionCore : public MotionPort
{
public:
  using Clock = std::chrono::steady_clock;
  using PhaseCallback =
    std::function<void (std::uint8_t, std::uint8_t, std::uint8_t, std::uint8_t)>;

  struct GripperOpenResult
  {
    bool accepted{false};
    bool command_issued{false};
    bool released_confirmed{false};
    std::string diagnostic_detail;
  };

  explicit MotionCore(
    std::shared_ptr<MotionBackend> backend,
    std::chrono::milliseconds safety_state_timeout = std::chrono::milliseconds(500),
    MotionGeometry geometry = {},
    std::chrono::milliseconds holding_confirmation_timeout = std::chrono::milliseconds(500),
    std::function<Clock::time_point()> now = []() {return Clock::now();});

  MotionStatus status() const;
  void update_safety_state(const SafetyState & state, Clock::time_point receipt_time);
  bool request_cancel(const UUID & goal_uuid);
  bool request_stop(const UUID & request_uuid);
  bool request_stop(const UUID & request_uuid, uint8_t stop_mode);
  void refresh_stop_state();
  bool set_runtime_tuning(
    const MotionTuning & tuning, const MotionGeometry & geometry,
    std::chrono::milliseconds holding_confirmation_timeout, bool idle_mutable = true);
  GripperOpenResult request_operator_gripper_open(
    bool object_supported_or_restrained, bool release_area_clear);
  MotionPortResult planAndExecuteMotion(const MotionPortRequest & request) override;
  MotionPlanningResult planMotion(const MotionPortRequest & request) override;
  MotionPortResult executePlannedMotion(const MotionPlan & plan) override;
  MotionDisturbResult executeDisturb(const MotionDisturbRequest & request) override;
  MotionDisturbanceParameters disturbanceParameters() const override;
  MotionCancelResult cancelMotion(
    const UUID & execution_id, const std::string & movement_id) override;
  MotionObservation observeMotionState() const override;
  bool measuredTcpPose(geometry_msgs::msg::PoseStamped & pose) const override;
  bool generatePlaceWaypointApproach(const std::string & path, std::string & detail);
  std::shared_ptr<GripperPort> gripper_port() const {return backend_->gripper_port();}
  void beginPickSearch(const std::string & execution_id) override;
  void beginPickCandidateSearch() override;
  void recordMaterialCandidateSearch(
    const std::vector<MaterialCandidateSearchAttempt> & attempts,
    const std::string & selected_candidate_id) override;
  std::string lastPickCandidateEvaluation() const override;
  void resetPickGraspInsertionStarted() override;
  bool pickGraspInsertionStarted() const override;
  std::string lastPlaceFailureStage() const override;
  bool advanceToNextPlanningCandidate() override;
  bool retryPlaceRetreat() override;
  void finalizeMotionPlanningDiagnostic(
    std::uint8_t result_code, double elapsed_s, const std::string & failure_stage,
    const std::string & failure_detail) override;
  void beginPickGraspRelation() override;
  void stageSelectedPickGraspRelation(const geometry_msgs::msg::Transform & transform) override;
  bool stagePickCaptureReference(
    const geometry_msgs::msg::PoseStamped & pose, double height, std::uint8_t symmetry,
    const std::string & source) override;
  void discardPendingGraspRelation() override;
  bool commitPendingGraspRelation() override;
  void clearGraspRelationAfterRelease() override;
  std::uint8_t executeLegacyMovement(
    std::uint8_t task_type, const std::string & execution_id,
    const std::string & movement_id) override;
  std::uint8_t executePlaceRecoveryJointTarget(
    const UUID & execution_id, const std::string & movement_id) override;

private:
  static bool is_supported_task(uint8_t task_type);
  static bool is_known_safety_state(uint8_t safety_state);
  static bool is_known_capability(uint8_t capability);
  static bool capability_allows_task(uint8_t capability, uint8_t task_type);
  uint8_t accept_goal_impl(
    const UUID & goal_uuid, uint8_t task_type,
    Clock::time_point acceptance_time, bool refresh_pre_submit_time,
    bool sample_initial_time,
    std::string * diagnostic_detail = nullptr,
    const MotionPortRequest * motion_request = nullptr,
    const ExecuteTask::Goal * named_task = nullptr, bool joint_target = false,
    const std::string & movement_id = {},
    NormalizedTrajectoryPlan * planning_output = nullptr,
    bool retain_planned_operation = false,
    std::uint64_t * retained_plan_id_output = nullptr);
  MotionPortResult finish_planned_operation(
    const MotionPlan & plan, const NormalizedTrajectoryPlan & backend_plan,
    std::uint8_t execution_result, double elapsed_s = 0.0,
    bool finalize_diagnostic = true);
  uint8_t validate_goal_locked(
    uint8_t task_type, Clock::time_point now,
    std::string * diagnostic_detail = nullptr) const;
  uint8_t interruption_result_locked(uint8_t execution_result) const;
  uint8_t current_interruption_result() const;
  void clear_execution_interruption_locked();

  std::shared_ptr<MotionBackend> backend_;
  std::shared_ptr<MotionExecutionSupport> motion_execution_support_;
  std::chrono::milliseconds safety_state_timeout_;
  std::function<Clock::time_point()> now_;
  mutable std::mutex mutex_;
  std::mutex stop_dispatch_mutex_;
  MotionStatus status_;
  SafetyState safety_state_;
  Clock::time_point safety_state_receipt_time_{};
  bool has_safety_state_{false};
  bool goal_submission_pending_{false};
  bool motion_execution_in_progress_{false};
  bool operator_gripper_open_in_progress_{false};
  bool place_waypoint_generation_in_progress_{false};
  std::string active_movement_id_;
  UUID pending_goal_uuid_{};
  bool stop_requested_{false};
  uint8_t requested_stop_mode_{0};
  enum class InterruptionReason : uint8_t {NONE, CALLER_CANCELED, SAFETY_PREEMPTED};
  InterruptionReason interruption_reason_{InterruptionReason::NONE};
  MotionTuning runtime_tuning_{};
  MotionGeometry runtime_geometry_{};
  std::chrono::milliseconds holding_confirmation_timeout_{500};
  MotionTuning active_tuning_snapshot_{};
  MotionGeometry active_geometry_snapshot_{};
  std::chrono::milliseconds active_holding_confirmation_timeout_{500};
  std::unordered_map<std::uint64_t, NormalizedTrajectoryPlan> retained_planned_operations_;
  std::uint64_t next_plan_id_{1U};
};

}  // namespace arm_cell_motion
