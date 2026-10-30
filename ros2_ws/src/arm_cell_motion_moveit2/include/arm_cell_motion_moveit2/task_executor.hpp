#pragma once

#include <cstdint>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <arm_cell_interfaces/action/execute_task.hpp>
#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>
#include <arm_cell_interfaces/msg/motion_task_type.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "arm_cell_motion_moveit2/gripper_port.hpp"
#include "arm_cell_motion_moveit2/motion_backend.hpp"
#include "arm_cell_motion_moveit2/object_state_tracker.hpp"

namespace arm_cell_motion_moveit2
{

using ExecuteTask = arm_cell_interfaces::action::ExecuteTask;
using MotionTaskResultCode = arm_cell_interfaces::msg::MotionTaskResultCode;
using MotionTaskType = arm_cell_interfaces::msg::MotionTaskType;

enum class HoldingWaitResult : uint8_t
{
  CONFIRMED,
  TIMEOUT,
  UNKNOWN,
  CANCELED,
  SAFETY_PREEMPTED,
  BACKEND_UNAVAILABLE
};

struct MotionGeometry
{
  std::string observation_reference;
  geometry_msgs::msg::Transform observation_to_object;
  geometry_msgs::msg::Transform object_to_grasp_tcp;
  geometry_msgs::msg::Vector3 insertion_axis_tcp;
  double target_roll_tolerance_rad{0.0};
  double target_pitch_tolerance_rad{0.0};
  double target_roll_sample_step_rad{0.0};
  double target_pitch_sample_step_rad{0.0};
  double pick_approach_distance_m{0.0};
  double pick_retract_distance_m{0.0};
  bool configured{false};

  bool valid() const;
};

class TaskExecutor
{
public:
  explicit TaskExecutor(
    std::shared_ptr<MotionBackend> backend,
    MotionGeometry geometry = {},
    std::chrono::milliseconds holding_confirmation_timeout = std::chrono::milliseconds(500));
  using InterruptionChecker = std::function<uint8_t()>;

  TaskExecutor(
    std::shared_ptr<MotionBackend> backend,
    std::shared_ptr<GripperPort> gripper_port,
    MotionGeometry geometry = {},
    InterruptionChecker interruption_checker = {},
    std::chrono::milliseconds holding_confirmation_timeout = std::chrono::milliseconds(500));

  void set_interruption_checker(InterruptionChecker interruption_checker);
  void set_runtime_geometry(
    MotionGeometry geometry, std::chrono::milliseconds holding_confirmation_timeout);

  uint8_t execute(const ExecuteTask::Goal & goal);
  uint8_t execute(
    const ExecuteTask::Goal & goal, MotionObjectState object_state,
    const std::string & motion_execution_id = {});
  void set_object_state(MotionObjectState object_state);
  MotionObjectState object_state() const;

private:
  uint8_t execute_pick(const ExecuteTask::Goal & goal);
  uint8_t execute_place(const ExecuteTask::Goal & goal);
  uint8_t execute_motion(const ExecuteTask::Goal & goal, uint8_t phase);
  uint8_t execute_gripper_close(float width_mm, std::uint64_t & baseline_sequence);
  uint8_t execute_gripper_open(std::uint64_t & baseline_sequence);
  std::uint64_t holding_sequence() const;
  HoldingWaitResult fresh_holding(
    HoldingState expected, std::uint64_t baseline_sequence) const;
  static uint8_t map_object_operation_result(uint8_t result);
  NormalizedMotionRequest normalized_request(
    const ExecuteTask::Goal & goal, uint8_t phase) const;

  std::shared_ptr<MotionBackend> backend_;
  std::shared_ptr<GripperPort> gripper_port_;
  MotionGeometry geometry_;
  std::chrono::milliseconds holding_confirmation_timeout_;
  InterruptionChecker interruption_checker_;
  ObjectStateTracker object_state_tracker_;
  std::string motion_execution_id_;
};

}  // namespace arm_cell_motion_moveit2
