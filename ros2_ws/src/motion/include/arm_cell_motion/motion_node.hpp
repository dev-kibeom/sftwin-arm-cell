#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <rclcpp_action/rclcpp_action.hpp>
#include <arm_cell_interfaces/action/execute_teaching.hpp>
#include <arm_cell_interfaces/srv/configure_teaching.hpp>

#include <rclcpp/rclcpp.hpp>
#include <arm_cell_interfaces/srv/stop_motion.hpp>
#include <arm_cell_interfaces/srv/request_gripper_open.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "arm_cell_motion/motion_core.hpp"
#include "arm_cell_motion/motion_backend.hpp"
#include "arm_cell_motion/motion_tuning.hpp"

namespace arm_cell_motion
{

class IsaacRosMotionTransport;

class MotionNode final : public rclcpp::Node
{
public:
  explicit MotionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  MotionNode(
    std::shared_ptr<MotionBackend> backend,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~MotionNode() override;

  rclcpp::Node::SharedPtr backend_node() const {return backend_node_;}
  std::shared_ptr<MotionCore> motion_core() const {return core_;}
  std::shared_ptr<GripperPort> gripper_port() const {return core_->gripper_port();}
  MotionGeometry motion_geometry() const {return runtime_geometry_;}
  std::chrono::milliseconds holding_confirmation_timeout() const
  {return holding_confirmation_timeout_;}
  void configure_waypoint_mode(bool pick_waypoint, bool place_waypoint);
  void initialize_backend();
  bool backend_ready() const;

private:
  void initialize_teaching();
  using ExecuteTeaching = arm_cell_interfaces::action::ExecuteTeaching;
  rclcpp_action::Server<ExecuteTeaching>::SharedPtr teaching_action_;
  rclcpp::Service<arm_cell_interfaces::srv::ConfigureTeaching>::SharedPtr teaching_service_;
  std::thread teaching_worker_;
  UUID teaching_goal_uuid_{};
  std::shared_ptr<MotionCore> core_;
  std::weak_ptr<MotionBackend> backend_readiness_;
  std::shared_ptr<IsaacRosMotionTransport> isaac_transport_;
  rclcpp::Node::SharedPtr backend_node_;
  rclcpp::Subscription<SafetyState>::SharedPtr safety_state_subscription_;
  rclcpp::Publisher<MotionStatus>::SharedPtr status_publisher_;
  rclcpp::CallbackGroup::SharedPtr stop_callback_group_;
  rclcpp::Service<arm_cell_interfaces::srv::StopMotion>::SharedPtr stop_service_;
  rclcpp::CallbackGroup::SharedPtr gripper_open_callback_group_;
  rclcpp::Service<arm_cell_interfaces::srv::RequestGripperOpen>::SharedPtr
  gripper_open_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr place_waypoint_generate_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr test_backend_active_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr test_backend_inactivity_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr tuning_callback_;
  MotionTuning runtime_tuning_{};
  MotionGeometry runtime_geometry_{};
  std::chrono::milliseconds holding_confirmation_timeout_{500};
  std::mutex tuning_parameters_mutex_;
};

}  // namespace arm_cell_motion
