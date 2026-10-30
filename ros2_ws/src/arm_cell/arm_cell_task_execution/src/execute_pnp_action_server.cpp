#include "arm_cell_task_execution/execute_pnp_action_server.hpp"

#include <iomanip>
#include <atomic>
#include <exception>
#include <sstream>
#include <thread>

#include <arm_cell_interfaces/msg/motion_task_phase.hpp>

namespace arm_cell_task_execution
{
namespace
{
std::string uuid_string(const rclcpp_action::GoalUUID & uuid)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : uuid) {
    stream << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return stream.str();
}

unique_identifier_msgs::msg::UUID message_uuid(const rclcpp_action::GoalUUID & uuid)
{
  unique_identifier_msgs::msg::UUID result;
  result.uuid = uuid;
  return result;
}
}  // namespace

ExecutePnpActionServer::ExecutePnpActionServer(
  rclcpp::Node * node, std::shared_ptr<MotionPort> motion_port,
  std::shared_ptr<TaskExecutor> task_executor)
: node_(node), motion_port_(std::move(motion_port)), task_executor_(std::move(task_executor)),
  execution_mutex_(task_executor_ ? task_executor_->execution_mutex() : nullptr)
{
  server_ = rclcpp_action::create_server<Action>(
    node_, "/task_execution/execute_pnp",
    std::bind(
      &ExecutePnpActionServer::handle_goal, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(&ExecutePnpActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&ExecutePnpActionServer::handle_accepted, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse ExecutePnpActionServer::handle_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal)
{
  const bool processable = node_ && task_executor_ && execution_mutex_ && goal &&
    rclcpp::ok(node_->get_node_base_interface()->get_context());
  return processable ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
         rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse ExecutePnpActionServer::handle_cancel(
  const std::shared_ptr<GoalHandle> & goal_handle)
{
  {
    std::lock_guard<std::mutex> lock(cancellation_state_->mutex);
    cancellation_state_->before_execution.insert(uuid_string(goal_handle->get_goal_id()));
  }
  task_executor_->cancel_active_motion(message_uuid(goal_handle->get_goal_id()));
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ExecutePnpActionServer::handle_accepted(const std::shared_ptr<GoalHandle> & goal_handle)
{
  std::thread{&ExecutePnpActionServer::execute, goal_handle, motion_port_, task_executor_,
    execution_mutex_, cancellation_state_}.detach();
}

void ExecutePnpActionServer::execute(
  const std::shared_ptr<GoalHandle> & goal_handle,
  const std::shared_ptr<MotionPort> & motion_port,
  const std::shared_ptr<TaskExecutor> & task_executor,
  const std::shared_ptr<std::mutex> & execution_mutex,
  const std::shared_ptr<CancellationState> & cancellation_state)
{
  const auto uuid = goal_handle->get_goal_id();
  const auto id = uuid_string(uuid);
  struct CancellationCleanup
  {
    std::shared_ptr<CancellationState> state;
    std::string goal_id;
    ~CancellationCleanup()
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->before_execution.erase(goal_id);
    }
  } cancellation_cleanup{cancellation_state, id};
  auto canceled_result = std::make_shared<Action::Result>();
  bool canceled_before_execution = false;
  {
    std::lock_guard<std::mutex> lock(cancellation_state->mutex);
    canceled_before_execution = cancellation_state->before_execution.erase(id) != 0U;
  }
  if (canceled_before_execution) {
    canceled_result->pnp_outcome = Action::Result::PNP_OUTCOME_FAILED;
    canceled_result->result_code.value =
      arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_CANCELED;
    canceled_result->diagnostic_detail = "caller cancellation before PnP execution";
    goal_handle->canceled(canceled_result);
    return;
  }

  std::unique_lock<std::mutex> execution_lock(*execution_mutex, std::try_to_lock);
  if (!execution_lock.owns_lock()) {
    auto result = std::make_shared<Action::Result>();
    result->pnp_outcome = Action::Result::PNP_OUTCOME_FAILED;
    result->result_code.value = arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BUSY;
    result->diagnostic_detail = "Task Execution already owns an active task";
    goal_handle->abort(result);
    return;
  }

  auto publish = [goal_handle](std::uint8_t phase, std::uint8_t attempt,
      std::uint8_t budget, std::uint8_t failure) {
      auto feedback = std::make_shared<Action::Feedback>();
      feedback->phase.value = phase;
      feedback->release_attempt = attempt;
      feedback->release_retry_budget = budget;
      feedback->release_last_failure = failure;
      feedback->next_acquisition_hint = Action::Feedback::NEXT_ACQUISITION_HINT_NONE;
      try {
        goal_handle->publish_feedback(feedback);
      } catch (...) {
      }
    };
  std::atomic_bool caller_canceled{false};
  const auto interruption_checker =
    [&]() {
      {
        std::lock_guard<std::mutex> lock(cancellation_state->mutex);
        caller_canceled.store(
          cancellation_state->before_execution.erase(id) != 0U || caller_canceled.load());
      }
      if (caller_canceled.load()) {
        return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_CANCELED;
      }
      const auto status = motion_port->observeMotionState().motion_status;
      // Stop observations belong to an execution, not to every future Action.
      // New movement admission remains with MotionCore.
      if (status.active_execution_id.uuid == uuid &&
        (status.execution_state == arm_cell_interfaces::msg::MotionStatus::MOTION_STATE_STOPPING ||
        status.execution_state == arm_cell_interfaces::msg::MotionStatus::MOTION_STATE_STOPPED))
      {
        return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED;
      }
      return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS;
    };
  struct CallbackCleanup
  {
    std::shared_ptr<TaskExecutor> task_executor;
    ~CallbackCleanup()
    {
      task_executor->set_interruption_checker({});
      task_executor->set_phase_callback({});
    }
  } callback_cleanup{task_executor};
  task_executor->set_interruption_checker(interruption_checker);

  auto result = std::make_shared<Action::Result>();
  const auto map_execution_exception = [
      &result, &goal_handle, &motion_port, &task_executor, &caller_canceled](
      const std::string & detail) {
      result->result_code.value =
        arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
      result->pnp_outcome = Action::Result::PNP_OUTCOME_FAILED;
      result->object_disposition = task_executor->object_disposition();
      result->material_placed = result->object_disposition ==
        arm_cell_interfaces::action::ExecuteTask::Result::OBJECT_DISPOSITION_PLACED;
      result->attempted_candidate_ids = task_executor->attempted_material_candidate_ids();
      result->selected_candidate_id = task_executor->outcome_material_candidate_id();
      const auto failure_stage = task_executor->last_failure_stage();
      if (!failure_stage.empty()) {result->diagnostic_detail = "failure_stage=" + failure_stage;}
      if (!result->diagnostic_detail.empty()) {result->diagnostic_detail += "; ";}
      result->diagnostic_detail += detail;

      if (caller_canceled.load() || goal_handle->is_canceling()) {
        result->result_code.value =
          arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_CANCELED;
      } else {
        try {
          const auto status = motion_port->observeMotionState().motion_status;
          if (status.active_execution_id.uuid == goal_handle->get_goal_id() &&
            (status.execution_state ==
            arm_cell_interfaces::msg::MotionStatus::MOTION_STATE_STOPPING ||
            status.execution_state ==
            arm_cell_interfaces::msg::MotionStatus::MOTION_STATE_STOPPED))
          {
            result->result_code.value =
              arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED;
            result->pnp_outcome = Action::Result::PNP_OUTCOME_SAFETY_PREEMPTED;
          }
        } catch (...) {
          // Preserve the original execution exception if Motion observation also fails.
        }
      }
    };
  try {
    *result = task_executor->execute_pnp(
      *goal_handle->get_goal(), id,
      [publish](std::uint8_t phase, std::uint8_t attempt, std::uint8_t budget,
      std::uint8_t failure) {publish(phase, attempt, budget, failure);});
  } catch (const std::exception & error) {
    map_execution_exception(std::string("Task Execution raised an exception: ") + error.what());
  } catch (...) {
    map_execution_exception("Task Execution raised an unknown exception");
  }
  if (result->result_code.value ==
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_CANCELED ||
    goal_handle->is_canceling())
  {
    goal_handle->canceled(result);
  } else if (result->pnp_outcome == Action::Result::PNP_OUTCOME_COMPLETED ||
    result->pnp_outcome == Action::Result::PNP_OUTCOME_RETRYABLE ||
    result->pnp_outcome == Action::Result::PNP_OUTCOME_CANDIDATES_EXHAUSTED)
  {
    goal_handle->succeed(result);
  } else {
    goal_handle->abort(result);
  }
}

}  // namespace arm_cell_task_execution
