#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <arm_cell_interfaces/action/execute_task.hpp>
#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>

#include "arm_cell_motion/gripper_port.hpp"
#include "arm_cell_motion/motion_geometry.hpp"
#include "arm_cell_motion/motion_port.hpp"
#include "arm_cell_task_execution/object_state_tracker.hpp"
#include "arm_cell_task_execution/task_execution_types.hpp"

namespace arm_cell_task_execution
{
using namespace arm_cell_motion;

class TaskExecutionSupport final
{
public:
  using InterruptionChecker = std::function<std::uint8_t()>;
  using PhaseCallback = std::function<void(std::uint8_t, std::uint8_t, std::uint8_t, std::uint8_t)>;

  TaskExecutionSupport(
    std::shared_ptr<MotionPort> motion_port, std::shared_ptr<GripperPort> gripper_port,
    MotionGeometry geometry,
    std::chrono::milliseconds holding_confirmation_timeout = std::chrono::milliseconds(500));

  void set_interruption_checker(InterruptionChecker checker);
  void set_phase_callback(PhaseCallback callback);
  void set_runtime_geometry(
    MotionGeometry geometry, std::chrono::milliseconds holding_confirmation_timeout);
  void set_execution_identity(std::string execution_id) {execution_id_ = std::move(execution_id);}
  const std::string & execution_identity() const {return execution_id_;}

  std::shared_ptr<MotionPort> motion_port() const {return motion_port_;}
  std::shared_ptr<GripperPort> gripper() const {return gripper_port_;}
  const MotionGeometry & geometry() const {return geometry_;}
  ObjectStateTracker & object_state_tracker() {return object_state_tracker_;}
  const ObjectStateTracker & object_state_tracker() const {return object_state_tracker_;}
  const InterruptionChecker & interruption_checker() const {return interruption_checker_;}
  const PhaseCallback & phase_callback() const {return phase_callback_;}

  MotionPortRequest make_motion_request(
    const ExecuteTask::Goal & goal, std::uint8_t phase, const std::string & execution_id,
    const std::string & movement_id = {});
  MotionPlanningResult plan_motion_request(
    const MotionPortRequest & request,
    std::uint8_t phase = arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_PLANNING);
  MotionPortResult execute_planned_motion(
    const MotionPlan & plan, bool completes_operation = true);
  std::uint8_t execute_legacy_movement(std::uint8_t task_type);
  std::uint8_t execute_place_recovery_joint_target();
  MotionCancelResult cancel_active_motion(const unique_identifier_msgs::msg::UUID & execution_id);
  std::uint8_t execute_gripper_close(std::uint64_t & baseline_sequence);
  std::uint8_t execute_gripper_open(
    std::uint64_t & baseline_sequence, std::uint8_t release_attempt = 0,
    std::uint8_t retry_budget = 0, std::uint8_t last_failure = 0);
  std::uint8_t wait_interruptibly(std::chrono::milliseconds duration) const;
  std::uint64_t holding_sequence() const;
  HoldingWaitResult fresh_holding(HoldingState expected, std::uint64_t baseline_sequence) const;

private:
  std::string next_movement_id(const char * kind);
  static std::uint8_t map_object_operation_result(std::uint8_t result);

  std::shared_ptr<MotionPort> motion_port_;
  std::shared_ptr<GripperPort> gripper_port_;
  MotionGeometry geometry_;
  std::chrono::milliseconds holding_confirmation_timeout_;
  InterruptionChecker interruption_checker_;
  PhaseCallback phase_callback_;
  ObjectStateTracker object_state_tracker_;
  std::string execution_id_;
  mutable std::mutex movement_mutex_;
  unique_identifier_msgs::msg::UUID active_execution_id_{};
  std::string active_movement_id_;
  std::uint64_t movement_sequence_{0};
};

}  // namespace arm_cell_task_execution
