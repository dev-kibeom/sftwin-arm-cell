#include "arm_cell_task_execution/task_execution_modules.hpp"

#include <chrono>

#include "arm_cell_task_execution/bt_callback_node.hpp"
#include "arm_cell_task_execution/release_wait_action.hpp"

namespace arm_cell_task_execution
{
using namespace arm_cell_motion;

std::uint8_t HeldObjectReturn::prepare()
{
  const auto gripper = support_.gripper();
  report_->failure_stage = stage_prefix_ + ":fresh_held_required";
  if (!gripper) {
    report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
    report_->result_code = MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
    return report_->result_code;
  }
  const auto held = gripper->holding();
  if (!held.fresh() || held.state != HoldingState::HELD) {
    report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
    report_->result_code = MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
    return report_->result_code;
  }

  return report_->result_code = MotionTaskResultCode::TASK_RESULT_SUCCESS;
}

std::uint8_t HeldObjectReturn::open()
{
  holding_baseline_ = 0;
  holding_result_ = HoldingWaitResult::TIMEOUT;
  report_->failure_stage = stage_prefix_ + ":gripper_open";
  report_->result_code = support_.execute_gripper_open(holding_baseline_);
  if (report_->result_code != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
    return report_->result_code;
  }

  return report_->result_code;
}

std::uint8_t HeldObjectReturn::finalize()
{
  if (support_.interruption_checker()) {
    const auto result = support_.interruption_checker()();
    if (result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      return report_->result_code = result;
    }
  }
  report_->failure_stage = stage_prefix_ + ":release_confirmation";
  const auto released = holding_result_;
  if (released != HoldingWaitResult::CONFIRMED) {
    report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
    report_->result_code = released == HoldingWaitResult::CANCELED ?
      MotionTaskResultCode::TASK_RESULT_CANCELED :
      released == HoldingWaitResult::SAFETY_PREEMPTED ?
      MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED :
      released == HoldingWaitResult::BACKEND_UNAVAILABLE ?
      MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE :
      MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE;
    return report_->result_code;
  }

  if (support_.prepared_grasp() && support_.prepared_grasp()->identity.valid()) {
    // Configured supply owns physical receiving confirmation and inventory.
    // Preserve the measured release fact if its scene handoff subsequently fails.
    report_->released_at_configured_destination = true;
    support_.object_state_tracker().mark_detached();
    report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED;
  }
  support_.motion_port()->clearGraspRelationAfterRelease();
  support_.object_state_tracker().mark_detached();
  report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED;
  report_->failure_stage = stage_prefix_ + ":completed";
  report_->result_code = MotionTaskResultCode::TASK_RESULT_SUCCESS;
  return report_->result_code;
}

void HeldObjectReturn::register_nodes(
  BT::BehaviorTreeFactory & factory, const ExecuteTask::Goal & goal,
  TaskExecutionReport & report)
{
  stage_prefix_ = goal.task_type.value == MotionTaskType::TASK_TYPE_PLACE ?
    "place_return" : "held_return";
  report_ = &report;
  register_callback_action(
    factory, "PrepareHeldTrayReturn", [this]() {
      return prepare() == MotionTaskResultCode::TASK_RESULT_SUCCESS ?
      BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  register_callback_action(
    factory, "OpenReturnedGripper", [this]() {
      return open() == MotionTaskResultCode::TASK_RESULT_SUCCESS ?
      BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  register_callback_action(
    factory, "FinalizeHeldTrayReturn", [this]() {
      return finalize() == MotionTaskResultCode::TASK_RESULT_SUCCESS ?
      BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  register_release_wait_action(
    factory, "WaitForReturnedRelease", support_, holding_baseline_,
    holding_result_, report, [this]() {
      report_->result_code = MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE;
      report_->failure_stage = stage_prefix_ + ":release_confirmation";
    });
}

}  // namespace arm_cell_task_execution
