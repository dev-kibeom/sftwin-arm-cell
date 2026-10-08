#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <arm_cell_interfaces/action/execute_task.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <arm_cell_interfaces/msg/mission_exit_reason.hpp>
#include <arm_cell_interfaces/msg/mission_phase.hpp>
#include <arm_cell_interfaces/msg/motion_status.hpp>
#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>
#include <arm_cell_interfaces/msg/safety_state.hpp>
#include <arm_cell_interfaces/srv/detect_target.hpp>

namespace arm_cell_orchestration_bt
{

enum class PlaceOrientationConstraint : std::uint8_t
{
  FIXED,
  BOUNDED,
  FREE
};

class CycleCoordinator
{
public:
  using DetectTarget = arm_cell_interfaces::srv::DetectTarget;
  using ExecuteTask = arm_cell_interfaces::action::ExecuteTask;
  using SafetyState = arm_cell_interfaces::msg::SafetyState;
  using MissionExitReason = arm_cell_interfaces::msg::MissionExitReason;
  using MissionPhase = arm_cell_interfaces::msg::MissionPhase;
  using MotionTaskResultCode = arm_cell_interfaces::msg::MotionTaskResultCode;
  using MotionStatus = arm_cell_interfaces::msg::MotionStatus;
  using StopMode = arm_cell_interfaces::msg::StopMode;

  struct MotionResult
  {
    MotionTaskResultCode code;
    std::string diagnostic_detail;
    uint8_t task_type{0};
    std::string motion_execution_id;
  };

  struct MotionFailureTrace
  {
    uint8_t task_type{0};
    MotionTaskResultCode code;
    std::string diagnostic_detail;
    std::string motion_execution_id;
  };

  struct Result
  {
    MissionExitReason exit_reason;
    std::string diagnostic_detail;
    std::optional<MotionFailureTrace> motion_failure;
  };

  struct RecoveryObservation
  {
    SafetyState safety;
    MotionStatus motion;
  };

  struct MissionRecipe
  {
    std::string recipe_id;
    std::string target_id;
    uint32_t schema_version{0};
    float grasp_width_mm{0.0F};
    bool has_grasp_width{false};
    double grasp_yaw_rad{0.0};
    bool has_grasp_yaw{false};
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

  using VisionFn = std::function<DetectTarget::Response(const DetectTarget::Request &)>;
  using MotionFn = std::function<MotionResult(const ExecuteTask::Goal &)>;
  using ReadSafetyFn = std::function<SafetyState()>;
  using MissionRecipeFn = std::function<std::optional<MissionRecipe>(const std::string &)>;
  using CancelMotionFn = std::function<void ()>;
  using PublishPhaseFn = std::function<void (uint8_t)>;
  using WaitForRecoveryFn = std::function<RecoveryObservation()>;
  using ReadRecoveryEntryFn = std::function<std::optional<SafetyState>()>;
  using RecordRecoveryResultFn = std::function<void (const MotionResult &)>;

  struct Callbacks
  {
    VisionFn detect_target;
    MotionFn execute_task;
    ReadSafetyFn read_safety;
    MissionRecipeFn load_recipe;
    CancelMotionFn cancel_motion;
    PublishPhaseFn publish_phase;
    WaitForRecoveryFn wait_for_recovery;
    ReadRecoveryEntryFn read_recovery_entry;
    RecordRecoveryResultFn record_recovery_result;
  };

  explicit CycleCoordinator(Callbacks callbacks);

  Result execute(const std::string & target_id);
  void set_cancel_requested(bool requested);

private:
  bool normal_motion_allowed() const;
  bool normal_motion_allowed(const SafetyState & state) const;
  Result interrupted(MissionExitReason reason, const std::string & detail);
  Result commit_non_safety_terminal(
    MissionExitReason reason, const std::string & detail,
    std::optional<MotionFailureTrace> motion_failure = std::nullopt);
  Result safety_preempted(
    const SafetyState & entry_safety,
    std::optional<MotionFailureTrace> motion_failure = std::nullopt);
  void coordinate_recovery(const SafetyState & entry_safety);
  MotionResult execute_motion(const ExecuteTask::Goal & goal, uint8_t phase);
  static MotionFailureTrace motion_failure_trace(const MotionResult & result);
  MissionExitReason motion_failure_exit_reason(const MotionResult & result) const;
  static ExecuteTask::Goal motion_goal(uint8_t task_type);
  static ExecuteTask::Goal pick_goal(
    const DetectTarget::Response & response, const MissionRecipe & recipe);
  static ExecuteTask::Goal place_goal(const MissionRecipe & recipe);
  static bool successful(const MotionResult & result);

  Callbacks callbacks_;
  std::atomic_bool cancel_requested_{false};
  mutable std::mutex cancel_mutex_;
  bool cancellation_sent_{false};
};

}  // namespace arm_cell_orchestration_bt

namespace arm_cell_orchestration_bt
{

class CancellationRegistry
{
public:
  using CancelFn = std::function<void ()>;

  void register_execution(const std::string & key, CancelFn cancel_execution);
  void register_active_motion(const std::string & key, CancelFn cancel_motion);
  void request_caller_cancel(const std::string & key);
  void request_secondary_motion_cancel(const std::string & key);
  void clear_active_motion(const std::string & key);
  void erase_execution(const std::string & key);

private:
  struct Entry
  {
    CancelFn cancel_execution;
    CancelFn cancel_motion;
  };

  std::mutex mutex_;
  std::unordered_map<std::string, Entry> entries_;
  std::unordered_set<std::string> pending_caller_cancels_;
  std::unordered_set<std::string> pending_motion_cancels_;
};

}  // namespace arm_cell_orchestration_bt
