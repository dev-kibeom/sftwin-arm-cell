#include "arm_cell_task_execution/legacy_execute_task_action_server.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <functional>
#include <iomanip>
#include <sstream>
#include <thread>

#include <arm_cell_interfaces/msg/motion_task_phase.hpp>
#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>

namespace arm_cell_task_execution
{
namespace
{
using Code = arm_cell_interfaces::msg::MotionTaskResultCode;
using Action = arm_cell_interfaces::action::ExecuteTask;
using UUID = unique_identifier_msgs::msg::UUID;

std::string uuid_string(const rclcpp_action::GoalUUID & uuid)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : uuid) {
    stream << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return stream.str();
}

UUID message_uuid(const rclcpp_action::GoalUUID & uuid)
{
  UUID result;
  result.uuid = uuid;
  return result;
}

std::uint8_t pick_failure_stage_code(std::uint8_t result, const std::string & stage)
{
  using Stage = Action::Result;
  if (result == Code::TASK_RESULT_SUCCESS) {return Stage::PICK_FAILURE_STAGE_NONE;}
  if (result == Code::TASK_RESULT_CANCELED) {return Stage::PICK_FAILURE_STAGE_CANCELLATION;}
  if (result == Code::TASK_RESULT_SAFETY_PREEMPTED) {return Stage::PICK_FAILURE_STAGE_SAFETY;}
  if (stage.find("validation:") == 0U) {return Stage::PICK_FAILURE_STAGE_VALIDATION;}
  if (stage.find("occupancy_") == 0U || stage.find("scene") != std::string::npos) {
    return Stage::PICK_FAILURE_STAGE_SCENE_INTEGRITY;
  }
  if (stage.find("approach:") == 0U) {
    if (result == Code::TASK_RESULT_IK_UNREACHABLE) {return Stage::PICK_FAILURE_STAGE_APPROACH_IK_FK;}
    if (result == Code::TASK_RESULT_PATH_OBSTRUCTED ||
      result == Code::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED)
    {
      return Stage::PICK_FAILURE_STAGE_APPROACH_PLANNING;
    }
    return Stage::PICK_FAILURE_STAGE_APPROACH_EXECUTION;
  }
  if (stage.find("grasp_motion:") == 0U) {
    if (result == Code::TASK_RESULT_IK_UNREACHABLE || result == Code::TASK_RESULT_PATH_OBSTRUCTED ||
      result == Code::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED)
    {
      return Stage::PICK_FAILURE_STAGE_GRASP_CARTESIAN_PLANNING;
    }
    return Stage::PICK_FAILURE_STAGE_GRASP_INSERTION_EXECUTION;
  }
  if (stage.find("gripper_") == 0U || stage.find("released_confirmation:") == 0U ||
    stage.find("held_confirmation:") == 0U || stage.find("grasp_relation_confirmation:") == 0U)
  {
    return Stage::PICK_FAILURE_STAGE_GRIPPER_CONFIRMATION;
  }
  if (stage.find("retract:") == 0U) {return Stage::PICK_FAILURE_STAGE_RETRACT;}
  if (stage.find("backend") != std::string::npos) {return Stage::PICK_FAILURE_STAGE_BACKEND;}
  return Stage::PICK_FAILURE_STAGE_UNKNOWN;
}

class FeedbackHeartbeat final
{
public:
  using GoalHandle = LegacyExecuteTaskActionServer::GoalHandle;

  explicit FeedbackHeartbeat(std::shared_ptr<GoalHandle> goal_handle)
  : goal_handle_(std::move(goal_handle)) {}

  ~FeedbackHeartbeat() {stop();}

  void start(std::uint8_t phase)
  {
    publish(phase, 0, 0, 0, "Reported Motion execution phase");
    thread_ = std::thread([this]() {
      std::unique_lock<std::mutex> lock(mutex_);
      while (running_) {
        if (condition_.wait_for(lock, std::chrono::milliseconds(500), [this]() {return !running_;})) {
          break;
        }
        const auto phase = phase_;
        lock.unlock();
        publish(phase, attempt_, retry_budget_, last_failure_, "Reported Motion execution phase");
        lock.lock();
      }
    });
  }

  void event(std::uint8_t phase, std::uint8_t attempt, std::uint8_t retry_budget,
    std::uint8_t last_failure)
  {
    const auto failure = last_failure == Action::Feedback::RELEASE_FAILURE_CONFIRMATION_TIMEOUT ?
      "release_confirmation_timeout" : "none";
    const auto detail = "PLACE release OPEN attempt " + std::to_string(attempt) + "/" +
      std::to_string(retry_budget + 1) + " after " + failure;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      phase_ = phase;
      attempt_ = attempt;
      retry_budget_ = retry_budget;
      last_failure_ = last_failure;
    }
    publish(phase, attempt, retry_budget, last_failure, detail);
  }

  void phase(std::uint8_t phase)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      phase_ = phase;
      attempt_ = retry_budget_ = last_failure_ = 0;
    }
    publish(phase, 0, 0, 0, "Reported Motion execution phase");
  }

  void stop()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      running_ = false;
    }
    condition_.notify_all();
    if (thread_.joinable()) {thread_.join();}
  }

private:
  void publish(
    std::uint8_t phase, std::uint8_t attempt, std::uint8_t budget,
    std::uint8_t failure, const std::string & detail)
  {
    auto feedback = std::make_shared<Action::Feedback>();
    feedback->phase.value = phase;
    feedback->diagnostic_detail = detail;
    feedback->release_attempt = attempt;
    feedback->release_retry_budget = budget;
    feedback->release_last_failure = failure;
    try {goal_handle_->publish_feedback(feedback);} catch (...) {}
  }

  std::shared_ptr<GoalHandle> goal_handle_;
  std::mutex mutex_;
  std::condition_variable condition_;
  bool running_{true};
  std::uint8_t phase_{arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_VALIDATING};
  std::uint8_t attempt_{0};
  std::uint8_t retry_budget_{0};
  std::uint8_t last_failure_{0};
  std::thread thread_;
};

}  // namespace

LegacyExecuteTaskActionServer::LegacyExecuteTaskActionServer(
  rclcpp::Node * node, std::shared_ptr<MotionPort> motion_port,
  std::shared_ptr<TaskExecutor> task_executor)
: node_(node), motion_port_(std::move(motion_port)),
  task_executor_(std::move(task_executor)),
  execution_mutex_(task_executor_ ? task_executor_->execution_mutex() : nullptr)
{
  server_ = rclcpp_action::create_server<Action>(
    node_, "/motion/execute_task",
    std::bind(&LegacyExecuteTaskActionServer::handle_goal, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(&LegacyExecuteTaskActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&LegacyExecuteTaskActionServer::handle_accepted, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse LegacyExecuteTaskActionServer::handle_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal)
{
  const bool processable = node_ && motion_port_ && task_executor_ && goal &&
    rclcpp::ok(node_->get_node_base_interface()->get_context());
  return processable ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
         rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse LegacyExecuteTaskActionServer::handle_cancel(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  {
    std::lock_guard<std::mutex> lock(cancellation_state_->mutex);
    cancellation_state_->before_execution.insert(uuid_string(goal_handle->get_goal_id()));
  }
  task_executor_->cancel_active_motion(message_uuid(goal_handle->get_goal_id()));
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LegacyExecuteTaskActionServer::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread{&LegacyExecuteTaskActionServer::execute, goal_handle, motion_port_, task_executor_,
    cancellation_state_, execution_mutex_}.detach();
}

void LegacyExecuteTaskActionServer::execute(
  const std::shared_ptr<GoalHandle> goal_handle,
  const std::shared_ptr<MotionPort> motion_port,
  const std::shared_ptr<TaskExecutor> task_executor,
  const std::shared_ptr<CancellationState> cancellation_state,
  const std::shared_ptr<std::mutex> execution_mutex)
{
  const auto id = uuid_string(goal_handle->get_goal_id());
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
  auto result = std::make_shared<Action::Result>();
  bool canceled_before_execution = false;
  {
    std::lock_guard<std::mutex> lock(cancellation_state->mutex);
    canceled_before_execution = cancellation_state->before_execution.erase(id) != 0U;
  }
  if (canceled_before_execution) {
    result->result_code.value = Code::TASK_RESULT_CANCELED;
    result->diagnostic_detail = "caller cancellation before task execution";
    goal_handle->canceled(result);
    return;
  }

  std::unique_lock<std::mutex> execution_lock(*execution_mutex, std::try_to_lock);
  if (!execution_lock.owns_lock()) {
    result->result_code.value = Code::TASK_RESULT_BUSY;
    result->diagnostic_detail = "Task Execution already owns an active task";
    goal_handle->abort(result);
    return;
  }

  FeedbackHeartbeat heartbeat(goal_handle);
  std::atomic_bool caller_canceled{false};
  struct CallbackCleanup
  {
    std::shared_ptr<TaskExecutor> task_executor;
    ~CallbackCleanup()
    {
      task_executor->set_interruption_checker({});
      task_executor->set_phase_callback({});
    }
  } callback_cleanup{task_executor};
  task_executor->set_interruption_checker([&]() {
      {
        std::lock_guard<std::mutex> lock(cancellation_state->mutex);
        caller_canceled.store(
          cancellation_state->before_execution.erase(id) != 0U || caller_canceled.load());
      }
      if (caller_canceled.load()) {return Code::TASK_RESULT_CANCELED;}
      const auto status = motion_port->observeMotionState().motion_status;
      // A stopped previous execution does not preempt this new Action. MotionCore
      // still owns admission using current capability and backend inactivity.
      if (status.active_execution_id.uuid == goal_handle->get_goal_id() &&
        (status.execution_state == arm_cell_interfaces::msg::MotionStatus::MOTION_STATE_STOPPING ||
        status.execution_state == arm_cell_interfaces::msg::MotionStatus::MOTION_STATE_STOPPED))
      {
        return Code::TASK_RESULT_SAFETY_PREEMPTED;
      }
      return Code::TASK_RESULT_SUCCESS;
    });
  task_executor->set_phase_callback([&heartbeat](
      std::uint8_t phase, std::uint8_t attempt, std::uint8_t budget, std::uint8_t failure) {
      if (attempt > 0U) {heartbeat.event(phase, attempt, budget, failure);}
      else {heartbeat.phase(phase);}
    });

  std::uint8_t result_code = Code::TASK_RESULT_BACKEND_ERROR;
  std::string exception_detail;
  heartbeat.start(arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_VALIDATING);
  try {
    result_code = task_executor->execute(*goal_handle->get_goal(),
      task_executor->object_state(), id);
  } catch (const std::exception & error) {
    exception_detail = std::string("Task Execution raised an exception: ") + error.what();
  } catch (...) {
    exception_detail = "Task Execution raised an unknown exception";
  }
  heartbeat.stop();

  result->result_code.value = result_code;
  result->pick_grasp_insertion_started =
    goal_handle->get_goal()->task_type.value == MotionTaskType::TASK_TYPE_PICK &&
    task_executor->pick_grasp_insertion_started();
  result->object_disposition = task_executor->object_disposition();
  result->attempted_candidate_ids = task_executor->attempted_material_candidate_ids();
  result->selected_candidate_id = task_executor->outcome_material_candidate_id();
  const auto failure_stage = task_executor->last_failure_stage();
  result->pick_failure_stage = goal_handle->get_goal()->task_type.value == MotionTaskType::TASK_TYPE_PICK ?
    pick_failure_stage_code(result_code, failure_stage) : Action::Result::PICK_FAILURE_STAGE_NONE;
  if (!failure_stage.empty()) {result->diagnostic_detail = "failure_stage=" + failure_stage;}
  if (!exception_detail.empty()) {
    if (!result->diagnostic_detail.empty()) {result->diagnostic_detail += "; ";}
    result->diagnostic_detail += exception_detail;
  }
  if (result_code == Code::TASK_RESULT_SUCCESS) {
    const auto status = motion_port->observeMotionState().motion_status;
    if (goal_handle->is_canceling()) {
      result->result_code.value = Code::TASK_RESULT_CANCELED;
    } else if (status.active_execution_id.uuid == goal_handle->get_goal_id() &&
      (status.execution_state == arm_cell_interfaces::msg::MotionStatus::MOTION_STATE_STOPPING ||
      status.execution_state == arm_cell_interfaces::msg::MotionStatus::MOTION_STATE_STOPPED))
    {
      result->result_code.value = Code::TASK_RESULT_SAFETY_PREEMPTED;
    }
  }
  if (result->result_code.value == Code::TASK_RESULT_CANCELED) {
    goal_handle->canceled(result);
  } else if (result->result_code.value == Code::TASK_RESULT_SUCCESS) {
    goal_handle->succeed(result);
  } else {
    if (result->diagnostic_detail.empty()) {
      result->diagnostic_detail = exception_detail.empty() ?
        "task execution failed" : exception_detail;
    }
    goal_handle->abort(result);
  }
}

}  // namespace arm_cell_task_execution
