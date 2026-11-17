#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <unordered_set>
#include <tuple>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <arm_cell_interfaces/action/execute_configured_grasp.hpp>
#include "arm_cell_task_execution/task_executor.hpp"
#include "arm_cell_task_execution/configured_supply_lease.hpp"

namespace arm_cell_task_execution
{
class ExecuteConfiguredGraspActionServer
{
public:
  using Action = arm_cell_interfaces::action::ExecuteConfiguredGrasp;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  ExecuteConfiguredGraspActionServer(
    rclcpp::Node *, std::shared_ptr<MotionPort>,
    std::shared_ptr<TaskExecutor>, std::map<std::string, PreparedGraspCycle>);

private:
  struct State
  {
    std::mutex mutex;
    std::unordered_set<std::string> canceled;
    std::unordered_set<std::string> invalidated;
    struct Active
    {
      arm_cell_motion::GraspExecutionIdentity identity;
      unique_identifier_msgs::msg::UUID uuid;
    };
    std::map<std::string, Active> active;
    ConfiguredSupplyLease lease;
    using LeaseKey = std::tuple<std::string, std::string, std::string>;
    std::map<LeaseKey, std::string> consumed;
    std::string observed_supply, observed_epoch;
  };
  static void fence_leases(const std::shared_ptr<State> &, const std::shared_ptr<TaskExecutor> &);
  static void execute(
    const std::shared_ptr<GoalHandle> &, std::shared_ptr<MotionPort>,
    std::shared_ptr<TaskExecutor>, std::shared_ptr<State>, PreparedGraspCycle);
  std::shared_ptr<State> state_{std::make_shared<State>()};
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::Subscription<arm_cell_interfaces::msg::ConfiguredSupplyState>::SharedPtr
    supply_subscription_;
  rclcpp::TimerBase::SharedPtr lease_watchdog_;
};
}
