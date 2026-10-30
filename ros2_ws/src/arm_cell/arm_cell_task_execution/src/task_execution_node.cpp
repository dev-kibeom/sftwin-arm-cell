#include <exception>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "arm_cell_motion/motion_node.hpp"
#include "arm_cell_task_execution/execute_pnp_action_server.hpp"
#include "arm_cell_task_execution/execute_disturb_action_server.hpp"
#include "arm_cell_task_execution/legacy_execute_task_action_server.hpp"
#include "arm_cell_task_execution/task_execution_strategy.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto motion = std::make_shared<arm_cell_motion::MotionNode>();
  rcl_interfaces::msg::ParameterDescriptor strategy_descriptor;
  strategy_descriptor.read_only = true;
  strategy_descriptor.description = "Task Execution strategy, fixed for each task";
  const auto pick_strategy = motion->declare_parameter<std::string>(
    "pick_strategy", "PLANNED", strategy_descriptor);
  const auto place_strategy = motion->declare_parameter<std::string>(
    "place_strategy", "PLANNED", strategy_descriptor);
  const arm_cell_task_execution::PickPlaceStrategies strategies{
    arm_cell_task_execution::parse_pick_place_strategy(pick_strategy, "motion.pick_strategy"),
    arm_cell_task_execution::parse_pick_place_strategy(place_strategy, "motion.place_strategy")};
  motion->configure_waypoint_mode(
    strategies.pick == arm_cell_task_execution::PickPlaceStrategy::WAYPOINT,
    strategies.place == arm_cell_task_execution::PickPlaceStrategy::WAYPOINT);
  auto task_executor = std::make_shared<arm_cell_task_execution::TaskExecutor>(
    motion->motion_core(), motion->gripper_port(), motion->motion_geometry(),
    arm_cell_task_execution::TaskExecutor::InterruptionChecker{},
    motion->holding_confirmation_timeout(), strategies);
  auto pnp_action_server = std::make_shared<arm_cell_task_execution::ExecutePnpActionServer>(
    motion.get(), motion->motion_core(), task_executor);
  auto disturb_action_server =
    std::make_shared<arm_cell_task_execution::ExecuteDisturbActionServer>(
    motion.get(), motion->motion_core(), task_executor->execution_mutex(),
    std::function<void(
      arm_cell_task_execution::ExecuteDisturbActionServer::MotionStartPhase)>{},
    arm_cell_task_execution::ExecuteDisturbActionServer::DisturbExecutor{
      [task_executor](const arm_cell_motion::MotionDisturbRequest & request) {
        return task_executor->execute_disturb(request);
      }});
  auto action_server = std::make_shared<arm_cell_task_execution::LegacyExecuteTaskActionServer>(
    motion.get(), motion->motion_core(), task_executor);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(motion);
  if (const auto backend_node = motion->backend_node()) {executor.add_node(backend_node);}

  std::exception_ptr initialization_error;
  std::thread executor_thread([&executor]() {executor.spin();});
  try {
    motion->initialize_backend();
  } catch (...) {
    initialization_error = std::current_exception();
    rclcpp::shutdown();
  }
  if (!initialization_error) {executor_thread.join();}
  else {
    executor_thread.join();
    std::rethrow_exception(initialization_error);
  }
  (void)action_server;
  (void)pnp_action_server;
  (void)disturb_action_server;
  rclcpp::shutdown();
  return 0;
}
