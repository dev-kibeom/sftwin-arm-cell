#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>

#include <arm_cell_interfaces/action/execute_task.hpp>
#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>
#include <arm_cell_interfaces/msg/motion_task_type.hpp>

#include "arm_cell_motion/motion_geometry.hpp"
#include "arm_cell_motion/motion_backend.hpp"
#include "arm_cell_motion/motion_port.hpp"

namespace arm_cell_motion
{

using ExecuteTask = arm_cell_interfaces::action::ExecuteTask;
using MotionTaskResultCode = arm_cell_interfaces::msg::MotionTaskResultCode;
using MotionTaskType = arm_cell_interfaces::msg::MotionTaskType;

// Motion-owned adapter for movement normalization, execution, and diagnostics.
// Task-level Gripper and Holding process behavior lives in Task Execution.
class MotionExecutionSupport final
{
public:
  using InterruptionChecker = std::function<std::uint8_t()>;
  MotionExecutionSupport(std::shared_ptr<MotionBackend> backend, MotionGeometry geometry);

  void set_interruption_checker(InterruptionChecker checker);
  void set_runtime_geometry(MotionGeometry geometry);

  std::shared_ptr<MotionBackend> backend() const {return backend_;}
  const MotionGeometry & geometry() const {return geometry_;}
  const InterruptionChecker & interruption_checker() const {return interruption_checker_;}
  std::uint8_t execute_motion(const MotionPortRequest & request);
  std::uint8_t plan_motion(
    const MotionPortRequest & request, NormalizedTrajectoryPlan & plan);
  std::uint8_t execute_planned_motion(
    const NormalizedTrajectoryPlan & plan, bool finalize_diagnostics = true);
  std::uint8_t execute_task(
    const ExecuteTask::Goal & goal, const std::string & execution_id);
  std::uint8_t execute_joint_target();
  NormalizedMotionRequest normalized_request(
    const ExecuteTask::Goal & goal, std::uint8_t phase, const std::string & execution_id) const;
  NormalizedMotionRequest normalized_request(const MotionPortRequest & request) const;

private:
  std::shared_ptr<MotionBackend> backend_;
  MotionGeometry geometry_;
  InterruptionChecker interruption_checker_;
};

}  // namespace arm_cell_motion
