#pragma once

#include <chrono>
#include <atomic>
#include <array>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <arm_cell_interfaces/action/execute_task.hpp>
#include <arm_cell_interfaces/action/execute_pnp.hpp>
#include <arm_cell_interfaces/action/execute_disturb.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <arm_cell_interfaces/msg/mission_exit_reason.hpp>
#include <arm_cell_interfaces/msg/mission_phase.hpp>
#include <arm_cell_interfaces/msg/motion_status.hpp>
#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>
#include <arm_cell_interfaces/srv/detect_target.hpp>
#include "arm_cell_orchestration/safety_observation.hpp"

namespace arm_cell_orchestration
{

enum class PlaceOrientationConstraint : std::uint8_t
{
  FIXED,
  BOUNDED,
  FREE
};

enum class UnobservedYawPolicy : std::uint8_t
{
  ALLOWED,
  FORBIDDEN
};

enum class PickApproachAxis : std::uint8_t
{
  TCP_INSERTION_AXIS,
  PLANNING_FRAME_VERTICAL
};

class CancellationRegistry
{
public:
  using CancelFn = std::function<void ()>;

  class ActionSubmission
  {
public:
    bool set_submission_id(std::string submission_id);
    std::string submission_id() const;
    void set_cancel_observer(CancelFn observer);
    void request_cancel();
    void set_cancel_fn(CancelFn cancel_action);
    bool set_goal_uuid(std::string goal_uuid);
    std::string goal_uuid() const;

private:
    mutable std::mutex mutex_;
    CancelFn cancel_action_;
    CancelFn cancel_observer_;
    bool cancel_observed_{false};
    bool cancel_requested_{false};
    bool cancel_sent_{false};
    std::string goal_uuid_;
    std::string submission_id_;
  };

  void register_execution(const std::string & key, CancelFn cancel_execution);
  bool register_action_submission(const std::string & key, const std::shared_ptr<ActionSubmission> & submission);
  void request_caller_cancel(const std::string & key);
  void clear_active_action(const std::string & key, const std::shared_ptr<ActionSubmission> & submission);
  void erase_execution(const std::string & key);

private:
  struct Entry
  {
    CancelFn cancel_execution;
    std::shared_ptr<ActionSubmission> active_submission;
    std::uint64_t next_action_sequence{1};
  };

  std::mutex mutex_;
  std::unordered_map<std::string, Entry> entries_;
  std::unordered_set<std::string> pending_caller_cancels_;
};


class CycleCoordinator
{
public:
  using DetectTarget = arm_cell_interfaces::srv::DetectTarget;
  using ExecuteTask = arm_cell_interfaces::action::ExecuteTask;
  using ExecutePnp = arm_cell_interfaces::action::ExecutePnp;
  using ExecuteDisturb = arm_cell_interfaces::action::ExecuteDisturb;
  using MissionExitReason = arm_cell_interfaces::msg::MissionExitReason;
  using MissionPhase = arm_cell_interfaces::msg::MissionPhase;
  using MotionTaskResultCode = arm_cell_interfaces::msg::MotionTaskResultCode;
  using ObjectDisposition = ExecuteTask::Result;
  using MotionStatus = arm_cell_interfaces::msg::MotionStatus;
  using StopMode = arm_cell_interfaces::msg::StopMode;

  struct MotionResult
  {
    MotionTaskResultCode code;
    std::string diagnostic_detail;
    uint8_t task_type{0};
    std::string motion_execution_id;
    bool result_code_available{true};
    std::string action_terminal{};
    std::uint8_t object_disposition{ObjectDisposition::OBJECT_DISPOSITION_UNKNOWN};
    bool pick_grasp_insertion_started{false};
    bool pick_grasp_insertion_started_available{false};
  };

  struct MotionFailureTrace
  {
    uint8_t task_type{0};
    MotionTaskResultCode code;
    std::string diagnostic_detail;
    std::string motion_execution_id;
    bool result_code_available{true};
    std::string action_terminal{};
    std::uint8_t object_disposition{ObjectDisposition::OBJECT_DISPOSITION_UNKNOWN};
  };

  struct PnpResult
  {
    std::uint8_t outcome{ExecutePnp::Result::PNP_OUTCOME_FAILED};
    bool material_placed{false};
    MotionTaskResultCode detail_code;
    std::string diagnostic_detail;
    std::string goal_uuid;
    std::string action_terminal;
    bool result_code_available{false};
    bool goal_rejected{false};
    bool safety_preempted{false};
    bool safety_handoff{false};
  };

  struct DisturbResult
  {
    std::uint8_t outcome{arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_FAILED};
    std::string diagnostic_detail;
    std::string action_terminal;
    bool goal_rejected{false};
    bool safety_preempted{false};
    bool safety_handoff{false};
  };

  struct Result
  {
    MissionExitReason exit_reason;
    std::string diagnostic_detail;
    std::optional<MotionFailureTrace> motion_failure;
    uint64_t processed_count{0};
  };

  struct RecoveryObservation
  {
    SafetyObservation safety;
    MotionStatus motion;
    bool motion_status_fresh{true};
  };

  struct MissionRecipe
  {
    std::string vision_algorithm{"cuboid_top_face"};
    std::string recipe_id;
    std::string target_id;
    uint32_t schema_version{0};
    float grasp_width_min_mm{0.0F};
    float grasp_width_max_mm{0.0F};
    bool has_grasp_width_policy{false};
    double grasp_yaw_rad{0.0};
    bool has_grasp_yaw{false};
    UnobservedYawPolicy unobserved_yaw_policy{UnobservedYawPolicy::FORBIDDEN};
    bool has_unobserved_yaw_policy{false};
    PickApproachAxis pick_approach_axis{PickApproachAxis::TCP_INSERTION_AXIS};
    double pick_approach_distance_m{0.0};
    double pick_grasp_position_tolerance_m{0.0};
    double pick_grasp_orientation_tolerance_rad{0.0};
    geometry_msgs::msg::PoseStamped place_pose;
    bool has_place_pose{false};
    geometry_msgs::msg::Quaternion place_tool_orientation_preference;
    bool has_place_tool_orientation_preference{false};
    double place_position_tolerance_m{0.0};
    double place_orientation_tolerance_rad{0.0};
    PlaceOrientationConstraint place_orientation_constraint{PlaceOrientationConstraint::FIXED};
    geometry_msgs::msg::Vector3 place_approach_direction_object;
    double place_approach_distance_m{0.0};
    double retract_distance_m{0.0};
  };

  using Submission = std::shared_ptr<CancellationRegistry::ActionSubmission>;
  using VisionFn = std::function<DetectTarget::Response(const DetectTarget::Request &)>;
  using MotionFn = std::function<MotionResult(const ExecuteTask::Goal &)>;
  using ReadSafetyFn = std::function<SafetyObservation()>;
  using ReadMotionStatusFn = std::function<std::optional<MotionStatus>(bool)>;
  using MissionRecipeFn = std::function<std::optional<MissionRecipe>(const std::string &)>;
  using PublishPhaseFn = std::function<void (uint8_t)>;
  using PublishProgressFn = std::function<void (uint64_t)>;
  using PublishRetryCountFn = std::function<void (uint8_t)>;
  using CancelPredicateFn = std::function<bool ()>;
  using WaitForRecoveryFn = std::function<RecoveryObservation()>;
  using WaitForRecoveryCancellableFn = std::function<bool (
        const CancelPredicateFn &, RecoveryObservation &)>;
  using WaitForBatchContinuationFn = std::function<bool (const CancelPredicateFn &)>;
  using ReadRecoveryEntryFn = std::function<std::optional<SafetyObservation>()>;
  using RecordRecoveryResultFn = std::function<void (const MotionResult &)>;
  using MotionFeedbackFn = std::function<void (uint8_t)>;
  using TreeStatusFn = std::function<void (
      const std::string &, const std::string &, const std::string &)>;
  using MotionWithFeedbackFn = std::function<MotionResult(
        const ExecuteTask::Goal &, const MotionFeedbackFn &)>;
  using PnpFeedbackFn = std::function<void (std::uint8_t)>;
  using ExecutePnpFn = std::function<PnpResult(const ExecutePnp::Goal &)>;
  using ExecutePnpWithFeedbackFn = std::function<PnpResult(
        const ExecutePnp::Goal &, const PnpFeedbackFn &)>;
  using ExecuteDisturbFn = std::function<DisturbResult(const ExecuteDisturb::Goal &)>;
  using VisionCancellableFn = std::function<DetectTarget::Response(
        const DetectTarget::Request &, const CancelPredicateFn &)>;
  using ReadVisionDiagnosticGenerationFn = std::function<uint64_t()>;
  using WaitForVisionIdleFn = std::function<bool (
        uint64_t, const CancelPredicateFn &)>;
  struct TargetAcquisition
  {
    std::optional<DetectTarget::Response> response;
  };

  using RecordVisionResultFn = std::function<void (std::uint8_t)>;
  enum class DiagnosticEvent : std::uint8_t {NONE, CANCELLATION, RECOVERY, RETRACT, RETRY,
    VISION_IDLE_WAIT, MOTION_FEEDBACK_GAP, MOTION_FEEDBACK_RECOVERED,
    MOTION_GOAL_ACCEPTED, MOTION_ACTION_RESULT, MOTION_ACTION_COMMUNICATION,
    MOTION_GRIPPER_RELEASE_ATTEMPT,
    MISSION_STARTED, MISSION_OUTCOME,
    BT_NODE_STATUS, ACTION_LIFECYCLE};
  enum class DiagnosticPhase : std::uint8_t {NONE, VISION, PICK};
  enum class DiagnosticDecision : std::uint8_t {NONE, RETRY, STOP, ALLOWED, DENIED};
  enum class DiagnosticFailureDomain : std::uint8_t {NONE, VISION, MOTION};
  enum class DiagnosticReason : std::uint8_t
  {
    NONE, MISSION_INTERRUPTION, HIGHER_STOP_SEVERITY, EMERGENCY_STOP,
    RECOVERY_CAPABILITY_REMOVED, HOLDING_INVALID, BUDGET_AVAILABLE,
    RETRY_BUDGET_EXHAUSTED, NON_RETRYABLE, RESULT_UNAVAILABLE,
    RETRY_ADMISSION, MOTION_STATUS_UNAVAILABLE, MOTION_NOT_IDLE,
    MOTION_ACTIVE, HOLDING_NOT_RELEASED, RETRY_ADMISSION_FAILED,
    RETRY_SAFETY_UNAVAILABLE, RETRY_SAFETY_NOT_NORMAL,
    RETRY_STOP_NOT_CONFIRMED, RETRY_CANCEL_REQUESTED,
    RECOVERY_AUTHORIZED_AND_STOPPED, MISSION_CANCELLED, SAFETY_CAPABILITY_WITHDRAWN,
    UNEXPECTED_BUSY, FEEDBACK_MISSING, FEEDBACK_RECOVERED,
    ACTION_COMMUNICATION_UNKNOWN, RELEASE_CONFIRMATION_TIMEOUT
  };
  struct DiagnosticRecord
  {
    DiagnosticEvent event{DiagnosticEvent::NONE};
    DiagnosticPhase phase{DiagnosticPhase::NONE};
    DiagnosticDecision decision{DiagnosticDecision::NONE};
    DiagnosticReason reason{DiagnosticReason::NONE};
    DiagnosticFailureDomain failure_domain{DiagnosticFailureDomain::NONE};
    std::uint8_t failure_code{0};
    std::uint8_t original_failure_code{0};
    std::uint8_t final_result{0};
    std::uint8_t mission_exit_reason{0};
    std::uint8_t capability{0};
    std::uint8_t selected_stop_mode{0};
    std::uint8_t motion_state{0};
    std::uint8_t observed_motion_phase{0};
    std::uint8_t motion_task_type{0};
    std::uint8_t holding_state{0};
    std::uint32_t attempt{0};
    std::uint32_t retry_budget{0};
    std::uint32_t requested_quantity{0};
    std::uint64_t processed_count{0};
    bool result_available{true};
    bool retryable{false};
    bool safety_valid{false};
    bool has_motion_status{false};
    bool motion_active{false};
    bool has_active_execution{false};
    bool backend_inactivity_confirmed{false};
    bool episode_retract_allowance{false};
    bool cancellation_requested{false};
    bool wait_started{false};
    std::array<char, 64> motion_execution_id{};
    std::uint8_t motion_execution_id_size{0};
    std::array<char, 64> motion_status_execution_id{};
    std::uint8_t motion_status_execution_id_size{0};
    std::array<char, 16> motion_action_terminal{};
    std::uint8_t motion_action_terminal_size{0};
    std::array<char, 128> result_detail{};
    std::uint8_t result_detail_size{0};
    std::array<char, 64> tree_node{};
    std::uint8_t tree_node_size{0};
    std::array<char, 16> tree_previous_status{};
    std::uint8_t tree_previous_status_size{0};
    std::array<char, 16> tree_current_status{};
    std::uint8_t tree_current_status_size{0};
    std::array<char, 64> wait_reason{};
    std::uint8_t wait_reason_size{0};
    std::int64_t wait_start_steady_ns{0};
    std::int64_t wait_deadline_steady_ns{0};
    std::int64_t wait_decision_steady_ns{0};
    std::int64_t wait_elapsed_ns{0};
    std::int64_t motion_action_elapsed_ns{-1};
    std::int64_t vision_observation_age_ns{-1};
    std::int64_t motion_observation_age_ns{-1};
    std::int64_t motion_status_stamp_ns{-1};
    std::uint64_t motion_status_generation{0};
    std::uint64_t requested_vision_generation{0};
    std::uint64_t observed_vision_generation{0};
    std::uint64_t requested_motion_status_generation{0};
    std::uint64_t observed_motion_status_generation{0};
    std::uint8_t vision_state{0};
    bool vision_observation_received{false};
  };
  using RecordDiagnosticFn = std::function<void (const std::string &)>;
  using RecordStructuredDiagnosticFn = std::function<void (const DiagnosticRecord &)>;

  struct Callbacks
  {
    VisionFn detect_target;
    VisionCancellableFn detect_target_cancellable;
    ReadVisionDiagnosticGenerationFn read_vision_diagnostic_generation;
    WaitForVisionIdleFn wait_for_vision_idle;
    RecordVisionResultFn record_vision_result;
    RecordDiagnosticFn record_diagnostic;
    RecordStructuredDiagnosticFn record_structured_diagnostic;
    MotionFn execute_task;
    ReadSafetyFn read_safety;
    ReadMotionStatusFn read_motion_status;
    MissionRecipeFn load_recipe;
    PublishPhaseFn publish_phase;
    PublishProgressFn publish_processed_count;
    PublishRetryCountFn publish_retry_count;
    WaitForRecoveryFn wait_for_recovery;
    WaitForRecoveryCancellableFn wait_for_recovery_cancellable;
    WaitForBatchContinuationFn wait_for_batch_continuation;
    ReadRecoveryEntryFn read_recovery_entry;
    RecordRecoveryResultFn record_recovery_result;
    MotionWithFeedbackFn execute_task_with_feedback;
    ExecutePnpFn execute_pnp;
    ExecutePnpWithFeedbackFn execute_pnp_with_feedback;
    ExecuteDisturbFn execute_disturb;
    std::function<MotionResult(const ExecuteTask::Goal &, const MotionFeedbackFn &, const Submission &)> execute_task_scoped;
    std::function<PnpResult(const ExecutePnp::Goal &, const PnpFeedbackFn &, const Submission &)> execute_pnp_scoped;
    std::function<DisturbResult(const ExecuteDisturb::Goal &, const Submission &)> execute_disturb_scoped;
    TreeStatusFn observe_tree_status;
  };
  explicit CycleCoordinator(Callbacks callbacks);

  Result execute(const std::string & target_id, uint32_t quantity = 1U);
  void set_cancel_requested(bool requested);

private:
  SafetyObservation read_safety() const;
  bool normal_motion_allowed() const;
  bool normal_motion_allowed(const SafetyObservation & observation) const;
  Result interrupted(
    MissionExitReason reason, const std::string & detail,
    std::optional<MotionFailureTrace> motion_failure = std::nullopt);
  Result commit_non_safety_terminal(
    MissionExitReason reason, const std::string & detail,
    std::optional<MotionFailureTrace> motion_failure = std::nullopt);
  Result safety_preempted(
    const SafetyObservation & entry_safety,
    std::optional<MotionFailureTrace> motion_failure = std::nullopt);
  void coordinate_recovery(const SafetyObservation & entry_safety);
  MotionResult execute_motion(
    const ExecuteTask::Goal & goal, uint8_t phase,
    const MotionFeedbackFn & feedback = MotionFeedbackFn{}, const Submission & submission = {});
  PnpResult execute_pnp(
    const ExecutePnp::Goal & goal, uint8_t phase,
    const PnpFeedbackFn & feedback = PnpFeedbackFn{}, const Submission & submission = {});
  DisturbResult execute_disturb(const ExecuteDisturb::Goal & goal, const Submission & submission = {});
  TargetAcquisition acquire_target(
    const std::string & target_id, const std::string & algorithm,
    const CancelPredicateFn & extra_cancel = CancelPredicateFn{});
  static MotionFailureTrace motion_failure_trace(const MotionResult & result);
  MissionExitReason motion_failure_exit_reason(const MotionResult & result) const;
  static ExecuteTask::Goal motion_goal(uint8_t task_type);
  static ExecutePnp::Goal pnp_goal(
    const DetectTarget::Response & response, const MissionRecipe & recipe);
  static bool successful(const MotionResult & result);
  void record_diagnostic(const DiagnosticRecord & record) const noexcept;
  bool diagnostics_observed() const noexcept;

  Callbacks callbacks_;
  std::atomic_bool cancel_requested_{false};
  std::optional<SafetyObservation> recovery_entry_;
  std::optional<MotionResult> recovery_held_return_result_;
  std::uint32_t episode_retry_count_{0U};
  uint64_t processed_count_{0};
};


}  // namespace arm_cell_orchestration
