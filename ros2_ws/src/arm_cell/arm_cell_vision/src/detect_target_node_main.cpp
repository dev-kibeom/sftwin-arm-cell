#include "detect_target_node_factory.hpp"

#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  auto node = arm_cell_vision::make_detect_target_node();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
