#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "arm_cell_motion_moveit2/fake_motion_backend.hpp"
#include "arm_cell_motion_moveit2/motion_node.hpp"

TEST(MotionNodeParameters, ValidatesRuntimeTuningAndRejectsStaticParameters)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("pick_approach_distance_m", 0.15),
    rclcpp::Parameter("pick_retract_distance_m", 0.05)});
  auto node = std::make_shared<arm_cell_motion_moveit2::MotionNode>(
    std::make_shared<arm_cell_motion_moveit2::FakeMotionBackend>(), options);

  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("planning_time_s", 20.0)).successful);
  EXPECT_FALSE(
    node->set_parameter(rclcpp::Parameter("planning_time_s", std::nan(""))).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("planning_time_s", 0.0)).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("backend_mode", "isaac")).successful);

  node.reset();
  rclcpp::shutdown();
}
