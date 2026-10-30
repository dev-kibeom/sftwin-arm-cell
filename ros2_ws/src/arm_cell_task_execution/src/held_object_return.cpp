#include "arm_cell_task_execution/task_execution_modules.hpp"

#include <chrono>

#include "arm_cell_task_execution/bt_callback_node.hpp"

namespace arm_cell_task_execution
{
using namespace arm_cell_motion;

std::uint8_t HeldObjectReturn::execute()
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

  report_->failure_stage = stage_prefix_ + ":tray_joint_target";
  report_->result_code = support_.execute_place_recovery_joint_target();
  if (report_->result_code != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
    return report_->result_code;
  }

  std::uint64_t holding_baseline = 0;
  report_->failure_stage = stage_prefix_ + ":gripper_open";
  report_->result_code = support_.execute_gripper_open(holding_baseline);
  if (report_->result_code != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
    return report_->result_code;
  }

  report_->failure_stage = stage_prefix_ + ":gripper_settle_wait";
  report_->result_code = support_.wait_interruptibly(std::chrono::seconds(3));
  if (report_->result_code != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
    return report_->result_code;
  }

  report_->failure_stage = stage_prefix_ + ":release_confirmation";
  const auto released = support_.fresh_holding(HoldingState::RELEASED, holding_baseline);
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
  register_callback_action(factory, "ReturnHeldToTray", [this]() {
      return execute() == MotionTaskResultCode::TASK_RESULT_SUCCESS ?
             BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
}

}  // namespace arm_cell_task_execution
