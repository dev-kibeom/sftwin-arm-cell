#include "arm_cell_task_execution/execute_configured_grasp_action_server.hpp"
#include <iomanip>
#include <sstream>
#include <thread>

namespace arm_cell_task_execution
{
namespace
{
std::string id_string(const rclcpp_action::GoalUUID & uuid)
{
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (auto byte : uuid) {
    out << std::setw(2) << static_cast<unsigned>(byte);
  }
  return out.str();
}
}
ExecuteConfiguredGraspActionServer::ExecuteConfiguredGraspActionServer(
  rclcpp::Node * node, std::shared_ptr<MotionPort> motion, std::shared_ptr<TaskExecutor> task,
  std::map<std::string, PreparedGraspCycle> catalog)
{
  const auto state = state_;
  supply_subscription_ = node->create_subscription<arm_cell_interfaces::msg::ConfiguredSupplyState>(
    "/cell/configured_supply/state", 10,
    [state, task](const arm_cell_interfaces::msg::ConfiguredSupplyState::ConstSharedPtr message) {
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (message->valid && !message->supply_epoch.empty() &&
        (state->observed_supply != message->supply_id ||
        state->observed_epoch != message->supply_epoch))
        {
          state->consumed.clear();
          state->observed_supply = message->supply_id;
          state->observed_epoch = message->supply_epoch;
        }
        state->lease.update(*message, ConfiguredSupplyLease::Clock::now());
      }
      fence_leases(state, task);
    });
  lease_watchdog_ = node->create_wall_timer(
    std::chrono::milliseconds(50),
    [state, task]() {fence_leases(state, task);});
  server_ = rclcpp_action::create_server<Action>(
    node, "/task_execution/execute_configured_grasp",
    [catalog, state](const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Action::Goal> goal) {
      const auto found = catalog.find(goal->slot_id);
      if (found == catalog.end() || found->second.identity.supply_id != goal->supply_id ||
      goal->supply_epoch.empty() || goal->reservation_id.empty())
      {return rclcpp_action::GoalResponse::REJECT;}
      auto identity = found->second.identity;
      identity.execution_id = id_string(uuid);
      identity.supply_epoch = goal->supply_epoch;
      identity.reservation_id = goal->reservation_id;
      std::lock_guard<std::mutex> lock(state->mutex);
      if (!state->lease.matches(identity, ConfiguredSupplyLease::Clock::now()) ||
      state->consumed.count(
        State::LeaseKey{identity.supply_id, identity.supply_epoch,
          identity.reservation_id}))
      {return rclcpp_action::GoalResponse::REJECT;}
      State::Active active; active.identity = identity; active.uuid.uuid = uuid;
      state->active.emplace(identity.execution_id, std::move(active));
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [state, task](const std::shared_ptr<GoalHandle> handle) {
      {std::lock_guard<std::mutex> lock(state->mutex);
        state->canceled.insert(id_string(handle->get_goal_id()));}
      unique_identifier_msgs::msg::UUID uuid; uuid.uuid = handle->get_goal_id();
      task->cancel_active_motion(uuid);
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [state, task, motion, catalog](const std::shared_ptr<GoalHandle> handle) {
      auto cycle = catalog.at(handle->get_goal()->slot_id);
      const auto & goal = *handle->get_goal();
      cycle.identity.execution_id = id_string(handle->get_goal_id());
      cycle.identity.supply_epoch = goal.supply_epoch;
      cycle.identity.reservation_id = goal.reservation_id;
      std::thread(
        &ExecuteConfiguredGraspActionServer::execute, handle, motion, task, state,
        cycle).detach();
    });
}

void ExecuteConfiguredGraspActionServer::fence_leases(
  const std::shared_ptr<State> & state, const std::shared_ptr<TaskExecutor> & task)
{
  std::vector<unique_identifier_msgs::msg::UUID> stops;
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    const auto now = ConfiguredSupplyLease::Clock::now();
    for (const auto & active : state->active) {
      if (!state->lease.matches(active.second.identity, now) &&
        state->invalidated.insert(active.first).second) {stops.push_back(active.second.uuid);}
    }
  }
  for (const auto & uuid : stops) {
    task->fence_configured_grasp(uuid);
  }
}

void ExecuteConfiguredGraspActionServer::execute(
  const std::shared_ptr<GoalHandle> & handle,
  std::shared_ptr<MotionPort> motion, std::shared_ptr<TaskExecutor> task,
  std::shared_ptr<State> state, PreparedGraspCycle cycle)
{
  using Code = arm_cell_interfaces::msg::MotionTaskResultCode;
  auto result = std::make_shared<Action::Result>();
  const auto id = cycle.identity.execution_id;
  struct Cleanup
  {
    std::shared_ptr<State> state; std::string id;
    ~Cleanup()
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->canceled.erase(id); state->active.erase(id); state->invalidated.erase(id);
    }
  } cleanup{state, id};
  auto canceled = [&]() {
      std::lock_guard<std::mutex> lock(state->mutex);
      return state->canceled.count(id) != 0 || handle->is_canceling();
    };
  if (canceled()) {
    result->result_code = Code::TASK_RESULT_CANCELED; handle->canceled(result); return;
  }
  std::unique_lock<std::mutex> lock(*task->execution_mutex(), std::try_to_lock);
  if (!lock.owns_lock()) {
    result->result_code = Code::TASK_RESULT_BUSY; handle->abort(result); return;
  }
  auto interruption = [&]() -> std::uint8_t {
      if (canceled()) {return Code::TASK_RESULT_CANCELED;}
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (state->invalidated.count(id) ||
          !state->lease.matches(cycle.identity, ConfiguredSupplyLease::Clock::now()))
        {
          state->invalidated.insert(id);
          return Code::TASK_RESULT_PERMISSION_DENIED;
        }
      }
      const auto status = motion->observeMotionState().motion_status;
      if (status.active_execution_id.uuid == handle->get_goal_id() &&
        (status.execution_state == arm_cell_interfaces::msg::MotionStatus::MOTION_STATE_STOPPING ||
        status.execution_state == arm_cell_interfaces::msg::MotionStatus::MOTION_STATE_STOPPED))
      {return Code::TASK_RESULT_SAFETY_PREEMPTED;}
      return Code::TASK_RESULT_SUCCESS;
    };
  struct CallbackCleanup
  {
    std::shared_ptr<TaskExecutor> task;
    ~CallbackCleanup() {task->set_interruption_checker({}); task->set_phase_callback({});}
  } callbacks{task};
  task->set_interruption_checker(interruption);
  task->set_phase_callback(
    [handle](std::uint8_t phase, std::uint8_t, std::uint8_t, std::uint8_t) {
      auto feedback = std::make_shared<Action::Feedback>();
      static const char * names[] = {"unknown", "validating", "planning", "approaching", "executing",
        "gripper", "attaching", "detaching", "retracting", "completing"};
      feedback->phase = phase < 10 ? names[phase] : "unknown";
      try {
        handle->publish_feedback(feedback);
      } catch (const std::exception &) {
      }
    });
  // Claim only after the shared execution mutex and current lease admission
  // succeeded. Busy/rejected requests never consume a Cell reservation.
  const auto admitted = interruption();
  if (admitted != Code::TASK_RESULT_SUCCESS) {
    result->result_code = admitted;
    result->failure_stage = "configured_supply:lease_admission";
    handle->abort(result); return;
  }
  {
    std::lock_guard<std::mutex> state_lock(state->mutex);
    const State::LeaseKey key{cycle.identity.supply_id, cycle.identity.supply_epoch,
      cycle.identity.reservation_id};
    if (!state->consumed.emplace(key, id).second) {
      result->result_code = Code::TASK_RESULT_INVALID_GOAL;
      result->failure_stage = "configured_supply:lease_already_used";
      handle->abort(result); return;
    }
  }
  try {
    result->result_code = task->execute_grasp_cycle(cycle, id);
  } catch (const std::exception & error) {
    result->result_code = Code::TASK_RESULT_BACKEND_ERROR;
    result->diagnostic_detail = error.what();
  } catch (...) {
    result->result_code = Code::TASK_RESULT_BACKEND_ERROR;
    result->diagnostic_detail = "unknown configured grasp exception";
  }
  result->material_placed = task->object_disposition() ==
    arm_cell_interfaces::action::ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED;
  result->failure_stage = task->last_failure_stage();
  {
    try {
      const auto cause = interruption();
      if (cause != Code::TASK_RESULT_SUCCESS) {result->result_code = cause;}
    } catch (const std::exception &) {
      // Keep the original execution failure if status observation is unavailable.
    }
  }
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    if (state->invalidated.count(id)) {
      result->diagnostic_detail += ";configured_supply_lease_invalid_or_stale";
    }
  }
  if (canceled()) {
    result->result_code = Code::TASK_RESULT_CANCELED;
    handle->canceled(result);
  } else if (result->result_code == Code::TASK_RESULT_SUCCESS) {handle->succeed(result);} else {
    handle->abort(result);
  }
}
}
