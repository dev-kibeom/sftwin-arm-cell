#pragma once

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <geometry_msgs/msg/transform.hpp>
#include <rclcpp/rclcpp.hpp>
#include <arm_cell_interfaces/action/execute_disturb.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "arm_cell_motion/motion_port.hpp"

namespace arm_cell_task_execution
{

class ExecuteDisturbActionServer final
{
public:
  using Action = arm_cell_interfaces::action::ExecuteDisturb;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  enum class MotionStartPhase
  {
    BEFORE_START_CLAIM,
    AFTER_START_CLAIM
  };
  using DisturbExecutor = std::function<arm_cell_motion::MotionDisturbResult(
      const arm_cell_motion::MotionDisturbRequest &)>;

  ExecuteDisturbActionServer(
    rclcpp::Node * node, std::shared_ptr<arm_cell_motion::MotionPort> motion_port,
    std::shared_ptr<std::mutex> execution_mutex,
    std::function<void(MotionStartPhase)> motion_start_hook = {},
    DisturbExecutor disturb_executor = {});

private:
  struct CancellationState
  {
    struct Entry
    {
      bool cancel_requested{false};
      bool cancel_accepted{false};
      bool motion_started{false};
      bool cancel_dispatching{false};
      bool terminal_claimed{false};
    };

    std::mutex mutex;
    std::condition_variable changed;
    std::unordered_map<std::string, Entry> executions;
  };

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Action::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> & goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> & goal_handle);
  static void execute(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::shared_ptr<arm_cell_motion::MotionPort> & motion_port,
    const std::shared_ptr<std::mutex> & execution_mutex,
    const std::shared_ptr<CancellationState> & cancellation_state,
    const rclcpp::Context::SharedPtr & context,
    const std::function<void(MotionStartPhase)> & motion_start_hook,
    const DisturbExecutor & disturb_executor);

  rclcpp::Node * node_;
  std::shared_ptr<arm_cell_motion::MotionPort> motion_port_;
  std::shared_ptr<std::mutex> execution_mutex_;
  std::function<void(MotionStartPhase)> motion_start_hook_;
  DisturbExecutor disturb_executor_;
  std::shared_ptr<CancellationState> cancellation_state_{std::make_shared<CancellationState>()};
  rclcpp_action::Server<Action>::SharedPtr server_;
};

}  // namespace arm_cell_task_execution
