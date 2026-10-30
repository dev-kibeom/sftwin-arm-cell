#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <arm_cell_interfaces/msg/collision_primitive.hpp>
#include <arm_cell_interfaces/msg/pick_candidate.hpp>
#include <arm_cell_interfaces/msg/disturbance_candidate.hpp>
#include <arm_cell_interfaces/msg/disturbance_failure_reason.hpp>
#include <arm_cell_interfaces/msg/disturbance_outcome.hpp>
#include <arm_cell_interfaces/msg/motion_status.hpp>
#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>
#include <arm_cell_interfaces/msg/safety_cause_set.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include "arm_cell_motion/pick_approach_axis.hpp"
#include "arm_cell_motion/place_orientation_constraint.hpp"
#include "arm_cell_motion/motion_port_support_types.hpp"

namespace arm_cell_motion
{

inline constexpr char kMotionDisturbMovementId[] = "disturb";

enum class MotionSegmentKind : std::uint8_t
{
  PICK_APPROACH,
  PICK_DESCENT,
  PICK_RETRACT,
  PLACE_APPROACH,
  PLACE_DESCENT,
  PLACE_CLEARANCE,
  PLACE_RETRACT,
  DISTURB_APPROACH,
  DISTURB_PUSH,
  DISTURB_RETRACT
};

enum class MotionFailureKind : std::uint8_t
{
  NONE,
  PLANNING,
  EXECUTION,
  CANCELED,
  SAFETY_PREEMPTED,
  OTHER
};

enum class MotionOrientationSymmetry : std::uint8_t
{
  UNOBSERVABLE = 0,
  UNIQUE = 1,
  TWOFOLD = 2,
  FOURFOLD = 4
};

struct MotionConstraints
{
  double approach_distance_m{0.0};
  double retract_distance_m{0.0};
  double position_tolerance_m{0.0};
  double orientation_tolerance_rad{0.0};
  geometry_msgs::msg::Vector3 approach_direction_object;
  bool has_estimated_object_height{false};
  float estimated_object_height_m{0.0F};
  bool has_observed_rectangle{false};
  float rectangle_long_side_m{0.0F};
  float rectangle_short_side_m{0.0F};
  float grasp_width_min_mm{0.0F};
  float grasp_width_max_mm{0.0F};
  MotionOrientationSymmetry orientation_symmetry_order{MotionOrientationSymmetry::UNIQUE};
  PickApproachAxis pick_approach_axis{PickApproachAxis::TCP_INSERTION_AXIS};
  PlaceOrientationConstraint place_orientation_constraint{PlaceOrientationConstraint::FIXED};
  double motion_speed_fraction{1.0};
};

// One target/segment request. execution_id links it to its owning operation;
// movement_id identifies this single Motion operation within that execution.
struct MotionPortRequest
{
  unique_identifier_msgs::msg::UUID execution_id;
  std::string movement_id;
  MotionSegmentKind segment{MotionSegmentKind::PICK_DESCENT};
  geometry_msgs::msg::PoseStamped target_pose;
  MotionConstraints constraints;
  std::string vision_acquisition_id;
  std::string vision_candidate_id;
  std::vector<arm_cell_interfaces::msg::PickCandidate> material_candidates;
  bool has_non_target_occupancy{false};
  std::vector<arm_cell_interfaces::msg::CollisionPrimitive> non_target_occupancy;
  bool has_unobserved_yaw_fallback{false};
  double unobserved_yaw_fallback_rad{0.0};
  bool has_place_tool_orientation_preference{false};
  geometry_msgs::msg::Quaternion place_tool_orientation_preference;
  // Optional planning start for chaining segments. An empty value keeps the
  // existing measured-current-state behavior.
  bool has_start_joint_state{false};
  sensor_msgs::msg::JointState start_joint_state;
  bool target_is_tcp_pose{false};
  bool place_waypoint_approach{false};
  double place_waypoint_clearance_y_margin_m{0.05};
};

// A planning result belongs to one reserved Motion operation. Its identity
// and plan_id are required by executePlannedMotion; the returned trajectory is
// already time-parameterized and can be inspected or passed to the executor.
struct MotionPlan
{
  std::uint64_t plan_id{0U};
  unique_identifier_msgs::msg::UUID execution_id;
  std::string movement_id;
  trajectory_msgs::msg::JointTrajectory trajectory;
  sensor_msgs::msg::JointState start_state;
  sensor_msgs::msg::JointState end_state;
};

struct MotionPlanningResult
{
  std::uint8_t result_code{
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR};
  MotionFailureKind failure_kind{MotionFailureKind::OTHER};
  std::string failure_stage;
  std::string diagnostic_detail;
  arm_cell_interfaces::msg::MotionStatus motion_status;
  MotionPlan plan;
};

struct MotionPortResult
{
  unique_identifier_msgs::msg::UUID execution_id;
  std::string movement_id;
  std::uint8_t result_code{
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR};
  MotionFailureKind failure_kind{MotionFailureKind::OTHER};
  std::string failure_stage;
  std::string diagnostic_detail;
  // The movement execution returned SUCCESS under its task-specific contract.
  // PICK/PLACE trajectory completion does not attest joint accuracy or stop;
  // stop/inactivity evidence remains separate in MotionStatus.
  bool motion_execution_completed{false};
  arm_cell_interfaces::msg::MotionStatus motion_status;
};

// One composite disturbance operation. execution_id is the owning Action UUID;
// acquisition identity and the complete source candidate list remain intact.
struct MotionDisturbRequest
{
  unique_identifier_msgs::msg::UUID execution_id;
  // Use this same ID when forwarding cancellation through cancelMotion().
  std::string movement_id{kMotionDisturbMovementId};
  std::string acquisition_id;
  std::uint64_t scene_generation{0};
  std::vector<arm_cell_interfaces::msg::DisturbanceCandidate> disturbance_candidates;
  std::function<std::uint8_t()> pre_submit_permission_check;
};

struct MotionDisturbResult
{
  unique_identifier_msgs::msg::UUID execution_id;
  std::string movement_id{kMotionDisturbMovementId};
  std::string acquisition_id;
  arm_cell_interfaces::msg::DisturbanceOutcome outcome;
  arm_cell_interfaces::msg::DisturbanceFailureReason failure_reason;
  arm_cell_interfaces::msg::SafetyCauseSet safety_cause;
  bool backend_inactivity_confirmed{false};
  std::string diagnostic_detail{"Motion disturbance execution is not implemented"};

  MotionDisturbResult()
  {
    outcome.value = arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_FAILED;
    failure_reason.value =
      arm_cell_interfaces::msg::DisturbanceFailureReason::FAILURE_REASON_NOT_IMPLEMENTED;
  }
};

struct MotionDisturbanceParameters
{
  double push_distance_m{0.02};
  double approach_clearance_m{0.01};
  double speed_fraction{0.2};
};

struct MotionCancelResult
{
  bool accepted{false};
  bool backend_inactivity_confirmed{false};
  arm_cell_interfaces::msg::MotionStatus motion_status;
};

struct MotionObservation
{
  arm_cell_interfaces::msg::MotionStatus motion_status;
  bool measured_joint_state_available{false};
  sensor_msgs::msg::JointState measured_joint_state;
};

class MotionPort
{
public:
  virtual ~MotionPort() = default;
  // Planning reserves the Motion operation until its plan is executed or the
  // same execution identity is canceled. No robot command is issued here.
  virtual MotionPlanningResult planMotion(const MotionPortRequest & request)
  {
    MotionPlanningResult result;
    result.plan.execution_id = request.execution_id;
    result.plan.movement_id = request.movement_id;
    return result;
  }
  virtual MotionPortResult executePlannedMotion(const MotionPlan & plan)
  {
    MotionPortResult result;
    result.execution_id = plan.execution_id;
    result.movement_id = plan.movement_id;
    return result;
  }
  virtual MotionPortResult planAndExecuteMotion(const MotionPortRequest & request) = 0;
  // Composite disturbance operation. The default is an explicit unavailable
  // result so existing MotionPort consumers remain source compatible.
  virtual MotionDisturbResult executeDisturb(const MotionDisturbRequest & request)
  {
    MotionDisturbResult result;
    result.execution_id = request.execution_id;
    result.movement_id = request.movement_id;
    result.acquisition_id = request.acquisition_id;
    return result;
  }
  virtual MotionDisturbanceParameters disturbanceParameters() const {return {};}
  virtual MotionCancelResult cancelMotion(
    const unique_identifier_msgs::msg::UUID & execution_id,
    const std::string & movement_id) = 0;
  virtual MotionObservation observeMotionState() const = 0;

  // Current measured TCP pose in base_link for PLACE WAYPOINT clearance.
  virtual bool measuredTcpPose(geometry_msgs::msg::PoseStamped &) const {return false;}
  // Motion planner state and measured planning evidence. These operations do
  // not rank or transition Material Candidates; Task Execution supplies the
  // selected candidate and owns that process decision.
  virtual void beginPickSearch(const std::string & execution_id)
  {(void)execution_id;}
  virtual void beginPickCandidateSearch() {}
  virtual void recordMaterialCandidateSearch(
    const std::vector<MaterialCandidateSearchAttempt> & attempts,
    const std::string & selected_candidate_id)
  {(void)attempts; (void)selected_candidate_id;}
  virtual std::string lastPickCandidateEvaluation() const {return {};}
  virtual void resetPickGraspInsertionStarted() {}
  virtual bool pickGraspInsertionStarted() const {return false;}
  virtual std::string lastPlaceFailureStage() const {return {};}
  virtual bool advanceToNextPlanningCandidate() {return false;}
  virtual bool retryPlaceRetreat() {return false;}
  virtual void finalizeMotionPlanningDiagnostic(
    std::uint8_t result_code, double elapsed_s, const std::string & failure_stage,
    const std::string & failure_detail)
  {(void)result_code; (void)elapsed_s; (void)failure_stage; (void)failure_detail;}
  virtual void beginPickGraspRelation() {}
  virtual void stageSelectedPickGraspRelation(const geometry_msgs::msg::Transform & transform)
  {(void)transform;}
  virtual bool stagePickCaptureReference(
    const geometry_msgs::msg::PoseStamped & pose, double height, std::uint8_t symmetry,
    const std::string & source)
  {(void)pose; (void)height; (void)symmetry; (void)source; return false;}
  virtual void discardPendingGraspRelation() {}
  // Task Execution owns the fresh Holding decision and calls these only after
  // that process gate succeeds. Motion only commits/clears its scene relation.
  virtual bool commitPendingGraspRelation() {return false;}
  virtual void clearGraspRelationAfterRelease() {}
  // Compatibility movement boundary for legacy standalone HOME/RETRACT goals.
  // PnP PICK/PLACE movements use planAndExecuteMotion one movement at a time.
  virtual std::uint8_t executeLegacyMovement(
    std::uint8_t task_type,
    const std::string & execution_id, const std::string & movement_id)
  {
    (void)task_type; (void)execution_id; (void)movement_id;
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  }
  virtual std::uint8_t executePlaceRecoveryJointTarget(
    const unique_identifier_msgs::msg::UUID & execution_id, const std::string & movement_id)
  {
    (void)execution_id; (void)movement_id;
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  }
};

}  // namespace arm_cell_motion
