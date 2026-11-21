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
#include "arm_cell_task_execution/configured_grasp_catalog.hpp"
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
    std::shared_ptr<TaskExecutor>, ConfiguredGraspCatalog);

private:
  struct State
  {
    std::mutex mutex;
    std::unordered_set<std::string> canceled;
    std::unordered_set<std::string> invalidated;
    struct Active
    {
      bool conveyor{false};
      PreparedGraspCycle cycle;
      unique_identifier_msgs::msg::UUID uuid;
    };
    std::map<std::string, Active> active;
    ConfiguredSupplyLease lease;
    using LeaseKey = std::tuple<std::string, std::string, std::string>;
    std::set<LeaseKey> consumed;
    std::set<std::pair<std::string, std::string>> executed_arrivals;
    std::string observed_supply, observed_epoch;
  };
  static void fence_leases(const std::shared_ptr<State> &, const std::shared_ptr<TaskExecutor> &);
  static void execute(
    const std::shared_ptr<GoalHandle> &, std::shared_ptr<MotionPort>,
    std::shared_ptr<TaskExecutor>, std::shared_ptr<State>, PreparedGraspCycle, bool fixed_teaching);
  std::shared_ptr<State> state_{std::make_shared<State>()};
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::Subscription<arm_cell_interfaces::msg::ConfiguredSupplyState>::SharedPtr
    supply_subscription_;
  rclcpp::TimerBase::SharedPtr lease_watchdog_;
};
}
