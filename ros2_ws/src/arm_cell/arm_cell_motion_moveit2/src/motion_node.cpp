#include "arm_cell_motion_moveit2/motion_node.hpp"

#include <exception>
#include <thread>

#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  auto node = std::make_shared<arm_cell_motion_moveit2::MotionNode>();
  executor.add_node(node);
  if (const auto backend_node = node->backend_node()) {
    executor.add_node(backend_node);
  }

  std::exception_ptr initialization_error;
  std::thread executor_thread([&executor]() {executor.spin();});
  try {
    node->initialize_backend();
  } catch (...) {
    initialization_error = std::current_exception();
    rclcpp::shutdown();
  }
  if (!initialization_error) {
    executor_thread.join();
  } else {
    executor_thread.join();
    std::rethrow_exception(initialization_error);
  }
  rclcpp::shutdown();
  return 0;
}
