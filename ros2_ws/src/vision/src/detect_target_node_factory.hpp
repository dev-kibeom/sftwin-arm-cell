#ifndef ARM_CELL_VISION__DETECT_TARGET_NODE_FACTORY_HPP_
#define ARM_CELL_VISION__DETECT_TARGET_NODE_FACTORY_HPP_

#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

namespace arm_cell_vision
{

// Internal composition seam shared by the executable and ROS boundary tests.
std::shared_ptr<rclcpp::Node> make_detect_target_node(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

}  // namespace arm_cell_vision

#endif  // ARM_CELL_VISION__DETECT_TARGET_NODE_FACTORY_HPP_
