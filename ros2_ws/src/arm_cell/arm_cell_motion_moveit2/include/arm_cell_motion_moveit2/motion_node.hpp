#pragma once

#include <chrono>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <arm_cell_interfaces/srv/stop_motion.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "arm_cell_motion_moveit2/motion_action_server.hpp"
#include "arm_cell_motion_moveit2/motion_backend.hpp"
#include "arm_cell_motion_moveit2/motion_tuning.hpp"

namespace arm_cell_motion_moveit2
{

class IsaacRosMotionTransport;

class MotionNode final : public rclcpp::Node
{
public:
  explicit MotionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  MotionNode(
    std::shared_ptr<MotionBackend> backend,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  rclcpp::Node::SharedPtr backend_node() const {return backend_node_;}
  void initialize_backend();

private:
  std::shared_ptr<MotionCore> core_;
  std::shared_ptr<IsaacRosMotionTransport> isaac_transport_;
  rclcpp::Node::SharedPtr backend_node_;
  std::shared_ptr<MotionActionServer> action_server_;
  rclcpp::Subscription<SafetyState>::SharedPtr safety_state_subscription_;
  rclcpp::Publisher<MotionStatus>::SharedPtr status_publisher_;
  rclcpp::CallbackGroup::SharedPtr stop_callback_group_;
  rclcpp::Service<arm_cell_interfaces::srv::StopMotion>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr test_backend_active_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr test_backend_inactivity_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr tuning_callback_;
  MotionTuning runtime_tuning_{};
  MotionGeometry runtime_geometry_{};
  std::chrono::milliseconds holding_confirmation_timeout_{500};
  std::mutex tuning_parameters_mutex_;
};

}  // namespace arm_cell_motion_moveit2
