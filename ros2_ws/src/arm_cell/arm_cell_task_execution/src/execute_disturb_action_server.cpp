#include "arm_cell_task_execution/execute_disturb_action_server.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <iomanip>
#include <sstream>
#include <thread>
#include <utility>

#include <arm_cell_interfaces/msg/disturbance_phase.hpp>

namespace arm_cell_task_execution
{
namespace
{
enum class CancelTransitionWait
{
  CANCELING,
  INACTIVE,
  STATE_ERROR,
  NOT_OBSERVED
};

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

void publish_feedback(
  const std::shared_ptr<ExecuteDisturbActionServer::GoalHandle> & goal_handle,
  std::uint8_t phase)
{
  auto feedback = std::make_shared<ExecuteDisturbActionServer::Action::Feedback>();
  feedback->phase.value = phase;
  try {
    goal_handle->publish_feedback(feedback);
  } catch (...) {
    // Feedback delivery does not change the operation outcome.
  }
}

CancelTransitionWait wait_for_cancel_transition(
  const std::shared_ptr<ExecuteDisturbActionServer::GoalHandle> & goal_handle,
  const rclcpp::Context::SharedPtr & context)
{
  // Humble exposes no notification for the state change performed after the
  // cancel callback returns. Wait without spinning, and stop if ROS shuts down
  // or the goal reaches a terminal state before CANCELING becomes observable.
  // The bound only ends observation; it never proves cancellation or backend
  // inactivity. Callers still use Motion's result or report a transition fault.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  std::mutex mutex;
  std::condition_variable changed;
  std::unique_lock<std::mutex> lock(mutex);
  while (true) {
    try {
      if (!context || !context->is_valid()) {return CancelTransitionWait::INACTIVE;}
      if (goal_handle->is_canceling()) {return CancelTransitionWait::CANCELING;}
      if (!goal_handle->is_active()) {return CancelTransitionWait::INACTIVE;}
    } catch (...) {
      return CancelTransitionWait::STATE_ERROR;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {return CancelTransitionWait::NOT_OBSERVED;}
    changed.wait_until(lock, std::min(deadline, now + std::chrono::milliseconds(2)));
  }
}

bool is_canceling_safely(
  const std::shared_ptr<ExecuteDisturbActionServer::GoalHandle> & goal_handle)
{
  try {
    return goal_handle->is_canceling();
  } catch (...) {
    return false;
  }
}

void finish_prestart_cancel(
  const std::shared_ptr<ExecuteDisturbActionServer::GoalHandle> & goal_handle,
  const rclcpp::Context::SharedPtr & context,
  const std::shared_ptr<ExecuteDisturbActionServer::Action::Result> & result)
{
  result->outcome.value = arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_CANCELED;
  result->diagnostic_detail = "request canceled before Motion operation started";
  const auto transition = wait_for_cancel_transition(goal_handle, context);
  if (transition == CancelTransitionWait::CANCELING) {
    try {goal_handle->canceled(result);} catch (...) {}
  } else if (transition == CancelTransitionWait::NOT_OBSERVED ||
    transition == CancelTransitionWait::STATE_ERROR)
  {
    result->outcome.value = arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_FAILED;
    result->failure_reason.value =
      arm_cell_interfaces::msg::DisturbanceFailureReason::FAILURE_REASON_EXECUTION_FAILURE;
    result->diagnostic_detail =
      "ROS Action did not enter CANCELING after accepted pre-start Cancel";
    try {goal_handle->abort(result);} catch (...) {}
  }
}
}  // namespace

ExecuteDisturbActionServer::ExecuteDisturbActionServer(
  rclcpp::Node * node, std::shared_ptr<arm_cell_motion::MotionPort> motion_port,
  std::shared_ptr<std::mutex> execution_mutex,
  std::function<void(MotionStartPhase)> motion_start_hook,
  DisturbExecutor disturb_executor)
: node_(node), motion_port_(std::move(motion_port)), execution_mutex_(std::move(execution_mutex)),
  motion_start_hook_(std::move(motion_start_hook)),
  disturb_executor_(std::move(disturb_executor))
{
  server_ = rclcpp_action::create_server<Action>(
    node_, "/task_execution/execute_disturb",
    std::bind(
      &ExecuteDisturbActionServer::handle_goal, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(&ExecuteDisturbActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&ExecuteDisturbActionServer::handle_accepted, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse ExecuteDisturbActionServer::handle_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal)
{
  const bool processable = node_ && motion_port_ && execution_mutex_ && goal &&
    rclcpp::ok(node_->get_node_base_interface()->get_context());
  return processable ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
         rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse ExecuteDisturbActionServer::handle_cancel(
  const std::shared_ptr<GoalHandle> & goal_handle)
{
  const auto uuid = goal_handle->get_goal_id();
  const auto key = uuid_string(uuid);
  bool dispatch_to_motion = false;
  {
    std::lock_guard<std::mutex> lock(cancellation_state_->mutex);
    const auto found = cancellation_state_->executions.find(key);
    if (found == cancellation_state_->executions.end() || found->second.terminal_claimed ||
      found->second.cancel_requested)
    {
      return rclcpp_action::CancelResponse::REJECT;
    }
    auto & entry = found->second;
    // Claiming the start boundary assigns cancellation delivery to MotionPort.
    // If MotionPort has not activated the operation yet, its rejection is
    // returned to ROS and this provisional state is rolled back below.
    entry.cancel_requested = true;
    dispatch_to_motion = entry.motion_started;
    entry.cancel_dispatching = dispatch_to_motion;
  }

  if (!dispatch_to_motion) {
    std::lock_guard<std::mutex> lock(cancellation_state_->mutex);
    const auto found = cancellation_state_->executions.find(key);
    if (found != cancellation_state_->executions.end()) {found->second.cancel_accepted = true;}
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  bool accepted = false;
  try {
    accepted = motion_port_->cancelMotion(
      message_uuid(uuid), arm_cell_motion::kMotionDisturbMovementId).accepted;
  } catch (...) {
    accepted = false;
  }
  {
    std::lock_guard<std::mutex> lock(cancellation_state_->mutex);
    const auto found = cancellation_state_->executions.find(key);
    if (found != cancellation_state_->executions.end()) {
      found->second.cancel_dispatching = false;
      if (accepted) {
        found->second.cancel_accepted = true;
      } else {
        found->second.cancel_accepted = false;
        found->second.cancel_requested = false;
      }
    }
    cancellation_state_->changed.notify_all();
  }
  if (!accepted) {return rclcpp_action::CancelResponse::REJECT;}
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ExecuteDisturbActionServer::handle_accepted(const std::shared_ptr<GoalHandle> & goal_handle)
{
  {
    std::lock_guard<std::mutex> lock(cancellation_state_->mutex);
    cancellation_state_->executions.emplace(
      uuid_string(goal_handle->get_goal_id()), CancellationState::Entry());
  }
  std::thread{&ExecuteDisturbActionServer::execute, goal_handle, motion_port_, execution_mutex_,
    cancellation_state_, node_->get_node_base_interface()->get_context(), motion_start_hook_,
    disturb_executor_}.
    detach();
}

void ExecuteDisturbActionServer::execute(
  const std::shared_ptr<GoalHandle> & goal_handle,
  const std::shared_ptr<arm_cell_motion::MotionPort> & motion_port,
  const std::shared_ptr<std::mutex> & execution_mutex,
  const std::shared_ptr<CancellationState> & cancellation_state,
  const rclcpp::Context::SharedPtr & context,
  const std::function<void(MotionStartPhase)> & motion_start_hook,
  const DisturbExecutor & disturb_executor)
{
  const auto uuid = goal_handle->get_goal_id();
  const auto id = uuid_string(uuid);
  auto result = std::make_shared<Action::Result>();
  struct CancellationCleanup
  {
    std::shared_ptr<CancellationState> state;
    std::string goal_id;
    ~CancellationCleanup()
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->executions.erase(goal_id);
    }
  } cancellation_cleanup{cancellation_state, id};

  std::unique_lock<std::mutex> execution_lock(*execution_mutex, std::try_to_lock);
  bool canceled_before_start = false;
  bool busy = false;
  {
    std::lock_guard<std::mutex> lock(cancellation_state->mutex);
    auto & entry = cancellation_state->executions.at(id);
    canceled_before_start = entry.cancel_requested;
    busy = !canceled_before_start && !execution_lock.owns_lock();
    if (canceled_before_start || busy) {
      entry.terminal_claimed = true;
    }
  }
  if (canceled_before_start) {
    finish_prestart_cancel(goal_handle, context, result);
    return;
  }
  if (busy) {
    result->outcome.value = arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_FAILED;
    result->failure_reason.value =
      arm_cell_interfaces::msg::DisturbanceFailureReason::FAILURE_REASON_EXECUTION_FAILURE;
    result->diagnostic_detail = "Task Execution already owns an active task";
    try {goal_handle->abort(result);} catch (...) {}
    return;
  }

  publish_feedback(goal_handle, arm_cell_interfaces::msg::DisturbancePhase::PHASE_VALIDATING);
  publish_feedback(goal_handle, arm_cell_interfaces::msg::DisturbancePhase::PHASE_PLANNING);

  arm_cell_motion::MotionDisturbResult motion_result;
  arm_cell_motion::MotionDisturbRequest request;
  request.execution_id = message_uuid(uuid);
  request.acquisition_id = goal_handle->get_goal()->acquisition_id;
  request.scene_generation = goal_handle->get_goal()->scene_generation;
  request.disturbance_candidates = goal_handle->get_goal()->disturbance_candidates;
  if (motion_start_hook) {motion_start_hook(MotionStartPhase::BEFORE_START_CLAIM);}

  {
    std::lock_guard<std::mutex> lock(cancellation_state->mutex);
    auto & entry = cancellation_state->executions.at(id);
    canceled_before_start = entry.cancel_requested;
    if (canceled_before_start) {
      entry.terminal_claimed = true;
    } else {
      // This is the start/cancel linearization point immediately before the
      // Motion call. Earlier Cancel requests do not call cancelMotion; later
      // ones are forwarded using the same execution identity.
      entry.motion_started = true;
    }
  }
  if (canceled_before_start) {
    finish_prestart_cancel(goal_handle, context, result);
    return;
  }

  if (motion_start_hook) {motion_start_hook(MotionStartPhase::AFTER_START_CLAIM);}

  try {
    motion_result = disturb_executor ? disturb_executor(request) : motion_port->executeDisturb(request);
  } catch (const std::exception & error) {
    result->outcome.value = arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_FAILED;
    result->failure_reason.value =
      arm_cell_interfaces::msg::DisturbanceFailureReason::FAILURE_REASON_EXECUTION_FAILURE;
    result->diagnostic_detail = std::string("MotionPort.executeDisturb raised an exception: ") +
      error.what();
  } catch (...) {
    result->outcome.value = arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_FAILED;
    result->failure_reason.value =
      arm_cell_interfaces::msg::DisturbanceFailureReason::FAILURE_REASON_EXECUTION_FAILURE;
    result->diagnostic_detail = "MotionPort.executeDisturb raised an unknown exception";
  }

  if (result->diagnostic_detail.empty()) {
    result->outcome = motion_result.outcome;
    result->failure_reason = motion_result.failure_reason;
    result->safety_cause = motion_result.safety_cause;
    result->backend_inactivity_confirmed = motion_result.backend_inactivity_confirmed;
    result->diagnostic_detail = motion_result.diagnostic_detail;
  }
  if (result->outcome.value >
    arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_SAFETY_PREEMPTED)
  {
    result->outcome.value = arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_FAILED;
    result->failure_reason.value =
      arm_cell_interfaces::msg::DisturbanceFailureReason::FAILURE_REASON_EXECUTION_FAILURE;
    if (!result->diagnostic_detail.empty()) {result->diagnostic_detail += "; ";}
    result->diagnostic_detail += "MotionPort returned an unknown disturbance outcome";
  }
  bool cancel_accepted = false;
  {
    std::unique_lock<std::mutex> lock(cancellation_state->mutex);
    cancellation_state->changed.wait(lock, [&]() {
        return !cancellation_state->executions.at(id).cancel_dispatching;
      });
    cancel_accepted = cancellation_state->executions.at(id).cancel_accepted;
    cancellation_state->executions.at(id).terminal_claimed = true;
  }
  // Let the ROS Action server apply the accepted cancel transition before any
  // worker terminal call. COMPLETED/FAILED/Safety results still keep Motion's
  // classification and are legal terminal outcomes from CANCELING in Humble.
  if (cancel_accepted && wait_for_cancel_transition(goal_handle, context) ==
    CancelTransitionWait::INACTIVE)
  {
    return;
  }
  publish_feedback(goal_handle, arm_cell_interfaces::msg::DisturbancePhase::PHASE_COMPLETING);

  // Prefer Motion's terminal classification when Cancel and Safety race. The
  // Motion operation owns arbitration and carries the authoritative evidence.
  if (result->outcome.value == arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_CANCELED) {
    if (is_canceling_safely(goal_handle)) {
      try {goal_handle->canceled(result);} catch (...) {}
    } else {
      // Motion may report cancellation independently of a caller Action Cancel.
      // Preserve that operation outcome without fabricating a cancel transition.
      try {goal_handle->abort(result);} catch (...) {}
    }
  } else if (result->outcome.value == arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_COMPLETED) {
    try {goal_handle->succeed(result);} catch (...) {}
  } else {
    try {goal_handle->abort(result);} catch (...) {}
  }
}

}  // namespace arm_cell_task_execution
