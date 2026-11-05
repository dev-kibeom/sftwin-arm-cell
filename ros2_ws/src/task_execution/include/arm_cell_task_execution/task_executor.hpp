#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <arm_cell_interfaces/action/execute_task.hpp>
#include <arm_cell_interfaces/action/execute_pnp.hpp>
#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>
#include <arm_cell_interfaces/msg/motion_task_type.hpp>

#include "arm_cell_motion/candidate_occupancy.hpp"
#include "arm_cell_task_execution/task_execution_support.hpp"
#include "arm_cell_motion/motion_geometry.hpp"
#include "arm_cell_motion/motion_port.hpp"
#include "arm_cell_task_execution/task_execution_report.hpp"
#include "arm_cell_task_execution/task_execution_strategy.hpp"

namespace arm_cell_task_execution
{
using namespace arm_cell_motion;

class MotionBtExecutor;

// Task-level PICK/PLACE process execution. MotionPort owns each movement.
class TaskExecutor
{
public:
  using ExecutePnp = arm_cell_interfaces::action::ExecutePnp;
  using InterruptionChecker = TaskExecutionSupport::InterruptionChecker;
  using PhaseCallback = TaskExecutionSupport::PhaseCallback;

  TaskExecutor(
    std::shared_ptr<MotionPort> motion_port,
    std::shared_ptr<GripperPort> gripper_port,
    MotionGeometry geometry = {},
    InterruptionChecker interruption_checker = {},
    std::chrono::milliseconds holding_confirmation_timeout = std::chrono::milliseconds(500),
    PickPlaceStrategies strategies = {});
  explicit TaskExecutor(
    std::shared_ptr<TaskExecutionSupport> support,
    PickPlaceStrategies strategies = {});
  ~TaskExecutor();

  void set_interruption_checker(InterruptionChecker interruption_checker);
  void set_phase_callback(PhaseCallback phase_callback);
  void set_runtime_geometry(
    MotionGeometry geometry, std::chrono::milliseconds holding_confirmation_timeout);

  std::uint8_t execute(const ExecuteTask::Goal & goal);
  std::uint8_t execute(
    const ExecuteTask::Goal & goal, MotionObjectState object_state,
    const std::string & motion_execution_id = {});
  ExecutePnp::Result execute_pnp(
    const ExecutePnp::Goal & goal, const std::string & motion_execution_id = {},
    PhaseCallback phase_callback = {});
  MotionDisturbResult execute_disturb(const MotionDisturbRequest & request);
  std::shared_ptr<std::mutex> execution_mutex() const {return execution_mutex_;}
  void set_object_state(MotionObjectState object_state);
  MotionObjectState object_state() const;
  bool pick_grasp_insertion_started() const;
  MotionCancelResult cancel_active_motion(const unique_identifier_msgs::msg::UUID & execution_id);
  std::uint8_t object_disposition() const {return last_report_.object_disposition;}
  std::string last_failure_stage() const {return last_report_.failure_stage;}
  const std::vector<std::string> & attempted_material_candidate_ids() const
  {return attempted_material_candidate_ids_;}
  const std::string & outcome_material_candidate_id() const
  {return outcome_material_candidate_id_;}
  void reset_candidate_search_result();

private:
  std::shared_ptr<TaskExecutionSupport> support_;
  PickPlaceStrategies strategies_;
  std::unique_ptr<MotionBtExecutor> bt_executor_;
  TaskExecutionReport last_report_;
  std::vector<std::string> attempted_material_candidate_ids_;
  std::string outcome_material_candidate_id_;
  std::shared_ptr<std::mutex> execution_mutex_{std::make_shared<std::mutex>()};
};

}  // namespace arm_cell_task_execution
