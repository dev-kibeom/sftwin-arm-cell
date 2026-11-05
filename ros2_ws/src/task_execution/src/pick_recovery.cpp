#include "arm_cell_task_execution/task_execution_modules.hpp"

#include "arm_cell_task_execution/bt_callback_node.hpp"

namespace arm_cell_task_execution
{
using namespace arm_cell_motion;

void PickRecovery::prepare()
{
  auto & recovery = state_;
  recovery.recovery_needed = false;
  recovery.open_required = false;
  recovery.already_returned = false;
  if (report_->failure_stage == "retract_held_confirmation" ||
    report_->result_code == MotionTaskResultCode::TASK_RESULT_SUCCESS ||
    report_->result_code == MotionTaskResultCode::TASK_RESULT_CANCELED ||
    report_->result_code ==
    MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED ||
    report_->result_code == MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR ||
    report_->result_code ==
    MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE)
  {
    if (report_->object_disposition == ExecuteTask::Result::OBJECT_DISPOSITION_HELD &&
      support_.interruption_checker() &&
      support_.interruption_checker()() != MotionTaskResultCode::TASK_RESULT_SUCCESS)
    {
      report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
    }
    return;
  }

  const auto observed = support_.gripper()->holding();
  if (observed.fresh() && observed.state == HoldingState::RELEASED) {
    support_.motion_port()->clearGraspRelationAfterRelease();
    support_.object_state_tracker().mark_detached();

    report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED;
    recovery.already_returned = true;
    recovery.recovery_needed = true;
  } else if (observed.fresh() && observed.state == HoldingState::HELD) {
    recovery.open_required = true;
    recovery.recovery_needed = true;
  } else {
    report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
  }
}

std::uint8_t PickRecovery::open_return()
{
  auto & recovery = state_;
  report_->failure_stage = "retract";
  const auto result = support_.execute_gripper_open(recovery.holding_baseline);
  if (result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    report_->object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
    report_->result_code = result;
  }
  return result;
}

std::uint8_t PickRecovery::confirm_return()
{
  const auto released = support_.fresh_holding(
    HoldingState::RELEASED, state_.holding_baseline);
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
  return report_->result_code;
}

void PickRecovery::register_nodes(
  BT::BehaviorTreeFactory & factory, TaskExecutionReport & report)
{
  report_ = &report;
  register_callback_action(
    factory, "PreparePickRetractRecovery", [this]() {
      prepare();
      return BT::NodeStatus::SUCCESS;
    });
  register_callback_action(
    factory, "PickRetractRecoveryNeeded", [this]() {
      return state_.recovery_needed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  register_callback_action(
    factory, "PickRetractRecoveryNotNeeded", [this]() {
      return !state_.recovery_needed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  register_callback_action(
    factory, "PickRetractAlreadyReturned", [this]() {
      return state_.already_returned ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  register_callback_action(
    factory, "PickRetractOpenRequired", [this]() {
      return state_.open_required ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  register_callback_action(
    factory, "OpenPickRetractReturn", [this]() {
      const auto result = open_return();
      if (result == MotionTaskResultCode::TASK_RESULT_SUCCESS) {return BT::NodeStatus::SUCCESS;}
      return BT::NodeStatus::FAILURE;
    });
  register_callback_action(
    factory, "ConfirmPickRetractReturn", [this]() {
      return confirm_return() == MotionTaskResultCode::TASK_RESULT_SUCCESS ||
      report_->object_disposition == ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED ?
      BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
}

}  // namespace arm_cell_task_execution
