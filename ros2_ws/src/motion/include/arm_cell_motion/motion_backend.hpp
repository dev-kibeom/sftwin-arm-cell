#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include "arm_cell_motion/pick_approach_axis.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <string>
#include <vector>
#include <functional>

#include <arm_cell_interfaces/action/execute_task.hpp>

#include "arm_cell_motion/place_orientation_constraint.hpp"
#include "arm_cell_motion/gripper_port.hpp"
#include "arm_cell_motion/motion_tuning.hpp"
#include "arm_cell_motion/motion_port.hpp"
#include "arm_cell_motion/motion_port_support_types.hpp"
#include <arm_cell_interfaces/msg/motion_task_phase.hpp>
#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>
#include <arm_cell_interfaces/msg/motion_task_type.hpp>
#include <arm_cell_interfaces/msg/stop_mode.hpp>

namespace arm_cell_motion
{

struct NormalizedMotionRequest
{
  arm_cell_interfaces::msg::MotionTaskType task_type;
  std::string observation_reference;
  uint8_t orientation_symmetry_order{0};
  bool has_estimated_object_height{false};
  bool has_observed_rectangle{false};
  float rectangle_long_side_m{0.0F};
  float rectangle_short_side_m{0.0F};
  float grasp_width_min_mm{0.0F};
  float grasp_width_max_mm{0.0F};
  bool has_unobserved_yaw_fallback{false};
  double unobserved_yaw_fallback_rad{0.0};
  std::string vision_acquisition_id;
  std::string vision_candidate_id;
  std::vector<arm_cell_interfaces::msg::PickCandidate> material_candidates;
  bool has_non_target_occupancy{false};
  std::vector<arm_cell_interfaces::msg::CollisionPrimitive> non_target_occupancy;
  std::string motion_goal_uuid;
  std::string movement_id;
  // The owning Action's latched interruption, including after Safety returns SAFE.
  std::function<uint8_t()> interruption_result;
  geometry_msgs::msg::PoseStamped observation_pose;
  geometry_msgs::msg::PoseStamped object_pose;
  geometry_msgs::msg::PoseStamped tcp_target;
  geometry_msgs::msg::PoseStamped target_pose;
  geometry_msgs::msg::Transform object_to_grasp_tcp;
  geometry_msgs::msg::PoseStamped approach_target;
  geometry_msgs::msg::PoseStamped retract_target;
  geometry_msgs::msg::Vector3 insertion_axis_tcp;
  PickApproachAxis pick_approach_axis{PickApproachAxis::TCP_INSERTION_AXIS};
  double pick_grasp_position_tolerance_m{0.0};
  double pick_grasp_orientation_tolerance_rad{0.0};
  geometry_msgs::msg::Vector3 place_approach_direction_object;
  double approach_distance_m{0.0};
  double retract_distance_m{0.0};
  double estimated_object_height_m{0.0};
  double motion_speed_fraction{1.0};
  bool is_disturbance{false};
  double place_position_tolerance_m{0.0};
  double place_orientation_tolerance_rad{0.0};
  PlaceOrientationConstraint place_orientation_constraint{PlaceOrientationConstraint::FIXED};
  bool has_place_tool_orientation_preference{false};
  geometry_msgs::msg::Quaternion place_tool_orientation_preference;
  uint8_t phase{arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_UNKNOWN};
  bool has_start_joint_state{false};
  sensor_msgs::msg::JointState start_joint_state;
  bool target_is_tcp_pose{false};
  bool place_waypoint_approach{false};
  MotionSegmentKind segment{MotionSegmentKind::PICK_DESCENT};
  double place_waypoint_clearance_y_margin_m{0.05};
};

struct MotionEnvelope
{
  float max_velocity_scale{1.0F};
  float max_acceleration_scale{1.0F};
  bool valid{false};
};

struct MotionPlanningScales
{
  double max_velocity{1.0};
  double max_acceleration{1.0};
};

// Backend-private execution context accompanying the public MotionPlan. The
// normalized request and execution policy are retained from planning so the
// stored trajectory is executed without rebuilding or replanning it.
struct NormalizedTrajectoryPlan
{
  NormalizedMotionRequest request;
  trajectory_msgs::msg::JointTrajectory trajectory;
  sensor_msgs::msg::JointState start_state;
  sensor_msgs::msg::JointState end_state;
  MotionPlanningScales planning_scales;
  bool cartesian_primitive{false};
  bool place_recovery_waypoint{false};
};

inline MotionPlanningScales apply_motion_envelope_to_planning_scales(
  double calibrated_velocity_scale, double calibrated_acceleration_scale,
  const MotionEnvelope & envelope)
{
  return {
    calibrated_velocity_scale * envelope.max_velocity_scale,
    calibrated_acceleration_scale * envelope.max_acceleration_scale};
}

class MotionBackend
{
public:
  virtual ~MotionBackend() = default;

  virtual bool apply_runtime_tuning(const MotionTuning &) {return true;}
  virtual uint8_t generate_place_waypoint_approach(
    const NormalizedMotionRequest &, const std::string &)
  {return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;}

  virtual bool available() const = 0;
  virtual void begin_pick_search() {}
  virtual void begin_pick_search(const std::string & execution_id)
  { (void)execution_id; begin_pick_search(); }
  virtual void begin_pick_candidate_search() {}
  virtual void record_material_candidate_search(
    const std::vector<MaterialCandidateSearchAttempt> &, const std::string &) {}
  virtual std::shared_ptr<GripperPort> gripper_port() const {return nullptr;}
  virtual std::string availability_diagnostic() const {return "backend_readiness=unknown";}
  virtual bool set_motion_envelope(const MotionEnvelope & envelope)
  {
    return envelope.valid && envelope.max_velocity_scale == 1.0F &&
           envelope.max_acceleration_scale == 1.0F;
  }
  virtual MotionEnvelope applied_motion_envelope() const
  {
    return MotionEnvelope{1.0F, 1.0F, true};
  }
  virtual bool submit(const arm_cell_interfaces::action::ExecuteTask::Goal & goal) = 0;
  virtual MotionDisturbanceParameters disturbance_parameters() const {return {};}
  virtual uint8_t execute_task(const arm_cell_interfaces::action::ExecuteTask::Goal & goal)
  {
    return submit(goal) ?
           arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS :
           arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  }
  virtual uint8_t execute_task(
    const arm_cell_interfaces::action::ExecuteTask::Goal & goal,
    const std::string & motion_goal_uuid)
  {
    (void)motion_goal_uuid;
    return execute_task(goal);
  }
  virtual uint8_t execute_joint_target()
  {
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  }
  // SUCCESS means the existing trajectory completion oracle accepted measured
  // execution; it is distinct from backend inactivity/stop confirmation.
  virtual uint8_t execute_normalized(const NormalizedMotionRequest & request)
  {
    (void)request;
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  }
  virtual bool supports_split_planning() const {return false;}
  virtual uint8_t plan_normalized(
    const NormalizedMotionRequest &, NormalizedTrajectoryPlan &)
  {return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;}
  virtual uint8_t execute_planned_normalized(const NormalizedTrajectoryPlan &)
  {return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;}
  virtual std::string last_pick_candidate_evaluation() const {return "";}
  virtual void reset_pick_grasp_insertion_started() {}
  virtual bool pick_grasp_insertion_started() const {return false;}
  // Detailed terminal stage of the current PLACE planning strategy, when available.
  virtual std::string last_place_failure_stage() const {return {};}
  // True only when the preceding failure occurred during planning, before a
  // trajectory was submitted, and another posture candidate is available.
  virtual bool advance_to_next_planning_candidate() {return false;}
  virtual bool retry_place_retreat() {return false;}
  virtual void finalize_motion_planning_diagnostic(uint8_t) {}
  virtual void finalize_motion_planning_diagnostic(uint8_t code, double)
  {finalize_motion_planning_diagnostic(code);}
  virtual void finalize_motion_planning_diagnostic(
    uint8_t code, double elapsed_s, const std::string & failure_stage,
    const std::string & failure_detail)
  {
    (void)failure_stage;
    (void)failure_detail;
    finalize_motion_planning_diagnostic(code, elapsed_s);
  }
  virtual uint8_t normalized_failure_result() const
  {return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;}
  virtual uint8_t command_gripper(float)
  {
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  }
  virtual void begin_pick_grasp_relation() {}
  virtual void stage_selected_pick_grasp_relation(const geometry_msgs::msg::Transform &) {}
  virtual bool stage_pick_capture_reference(
    const geometry_msgs::msg::PoseStamped &, double, uint8_t, const std::string &) {return false;}
  virtual void discard_pending_grasp_relation() {}
  virtual bool commit_pending_grasp_relation() {return true;}
  virtual void clear_grasp_relation_after_release() {}
  virtual bool gripper_active() const {return false;}
  virtual void stop_gripper() {}
  virtual void cancel() = 0;
  virtual void safety_preempt(uint8_t stop_mode)
  {
    switch (stop_mode) {
      case arm_cell_interfaces::msg::StopMode::STOP_MODE_CONTROLLED:
        hold();
        break;
      case arm_cell_interfaces::msg::StopMode::STOP_MODE_IMMEDIATE:
        cancel();
        hold();
        break;
      case arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY:
        stop();
        break;
      default:
        break;
    }
  }
  virtual void hold() = 0;
  virtual void stop() = 0;
  virtual bool execution_active() const = 0;
  virtual bool backend_inactivity_confirmed() const = 0;
  virtual std::string inactivity_diagnostic() const {return {};}
  virtual bool measured_joint_state(sensor_msgs::msg::JointState &) const {return false;}
  virtual bool measured_tcp_pose(geometry_msgs::msg::PoseStamped &) const {return false;}
};

}  // namespace arm_cell_motion
