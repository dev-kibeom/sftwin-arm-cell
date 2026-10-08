#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <string>

#include <arm_cell_interfaces/action/execute_task.hpp>

#include "arm_cell_motion_moveit2/place_orientation_constraint.hpp"
#include "arm_cell_motion_moveit2/gripper_port.hpp"
#include "arm_cell_motion_moveit2/motion_tuning.hpp"
#include <arm_cell_interfaces/msg/motion_task_phase.hpp>
#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>
#include <arm_cell_interfaces/msg/motion_task_type.hpp>
#include <arm_cell_interfaces/msg/stop_mode.hpp>

namespace arm_cell_motion_moveit2
{

struct NormalizedMotionRequest
{
  arm_cell_interfaces::msg::MotionTaskType task_type;
  std::string observation_reference;
  geometry_msgs::msg::PoseStamped observation_pose;
  geometry_msgs::msg::PoseStamped object_pose;
  geometry_msgs::msg::PoseStamped tcp_target;
  geometry_msgs::msg::PoseStamped target_pose;
  geometry_msgs::msg::Transform object_to_grasp_tcp;
  geometry_msgs::msg::PoseStamped approach_target;
  geometry_msgs::msg::PoseStamped retract_target;
  geometry_msgs::msg::Vector3 insertion_axis_tcp;
  geometry_msgs::msg::Vector3 place_approach_direction_object;
  double approach_distance_m{0.0};
  double retract_distance_m{0.0};
  double target_roll_tolerance_rad{0.0};
  double target_pitch_tolerance_rad{0.0};
  double target_roll_sample_step_rad{0.0};
  double target_pitch_sample_step_rad{0.0};
  double place_position_tolerance_m{0.0};
  double place_orientation_tolerance_rad{0.0};
  PlaceOrientationConstraint place_orientation_constraint{PlaceOrientationConstraint::FIXED};
  bool has_place_tool_orientation_preference{false};
  geometry_msgs::msg::Quaternion place_tool_orientation_preference;
  float grasp_width_mm{0.0F};
  uint8_t phase{arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_UNKNOWN};
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

  virtual bool available() const = 0;
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
  virtual uint8_t execute_task(const arm_cell_interfaces::action::ExecuteTask::Goal & goal)
  {
    return submit(goal) ?
           arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS :
           arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  }
  virtual uint8_t execute_normalized(const NormalizedMotionRequest & request)
  {
    (void)request;
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  }
  virtual uint8_t command_gripper(float)
  {
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  }
  virtual void begin_pick_grasp_relation() {}
  virtual void stage_selected_pick_grasp_relation(const geometry_msgs::msg::Transform &) {}
  virtual void discard_pending_grasp_relation() {}
  virtual bool confirm_fresh_held_grasp_relation() {return true;}
  virtual void confirm_fresh_released_grasp_relation() {}
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
};

}  // namespace arm_cell_motion_moveit2
