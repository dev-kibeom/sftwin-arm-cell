#include <rclcpp/rclcpp.hpp>

#include "arm_cell_vision/vision_ros_ingress.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm_cell_vision::VisionRosIngress>());
  rclcpp::shutdown();
  return 0;
}
