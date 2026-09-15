#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <urdf/model.h>
#include "arm_cell_sim_adapter/canonical_joint_state.hpp"

namespace arm_cell_sim_adapter
{
class JointStateAdapter : public rclcpp::Node
{
public:
  JointStateAdapter()
  : Node("joint_state_adapter")
  {
    urdf::Model model;
    if (!model.initString(declare_parameter<std::string>("robot_description", ""))) {
      throw std::invalid_argument("robot_description must contain a valid canonical URDF");
    }
    // Tree order is stable and puts the arm before its attached tool.
    collect_joints(model.getRoot());
    if (joints_.empty()) {
      throw std::invalid_argument("canonical model has no independent movable joints");
    }
    publisher_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::ClockQoS(), [this](rosgraph_msgs::msg::Clock::ConstSharedPtr msg) {
        const auto stamp = nanoseconds(msg->clock);
        if (!stamp) {
          latch_reset("invalid simulation clock");
        } else if (clock_stamp_ && *stamp < *clock_stamp_) {
          latch_reset("simulation clock moved backward");
        } else {
          clock_stamp_ = stamp;
        }
      });
    state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "raw_joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {publish_state(*msg);});
    RCLCPP_INFO(get_logger(), "Waiting for simulation clock and complete measured joint state");
  }

private:
  void collect_joints(const urdf::LinkConstSharedPtr & link)
  {
    for (const auto & child : link->child_links) {
      const auto & joint = child->parent_joint;
      if (joint->type != urdf::Joint::FIXED && !joint->mimic) {
        if (joint->type != urdf::Joint::REVOLUTE &&
          joint->type != urdf::Joint::CONTINUOUS && joint->type != urdf::Joint::PRISMATIC)
        {
          throw std::invalid_argument("JointState adapter requires scalar movable joints");
        }
        joints_.push_back(joint->name);
      }
      collect_joints(child);
    }
  }

  static std::optional<int64_t> nanoseconds(const builtin_interfaces::msg::Time & stamp)
  {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000u) {
      return std::nullopt;
    }
    return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  }

  void latch_reset(const char * reason)
  {
    if (!reset_required_) {
      reset_required_ = true;
      RCLCPP_ERROR(get_logger(), "%s; restart the complete Isaac ROS profile", reason);
    }
  }

  void publish_state(const sensor_msgs::msg::JointState & raw)
  {
    if (reset_required_ || !clock_stamp_) {
      return;
    }
    const auto stamp = nanoseconds(raw.header.stamp);
    const auto canonical = canonical_joint_state(raw, joints_);
    if (!stamp || !canonical) {
      if (!invalid_sample_) {
        RCLCPP_WARN(get_logger(), "Rejecting invalid/incomplete measured joint state");
        invalid_sample_ = true;
      }
      return;
    }
    if (state_stamp_ && *stamp < *state_stamp_) {
      latch_reset("measured state timestamp moved backward");
      return;
    }
    invalid_sample_ = false;
    state_stamp_ = stamp;
    publisher_->publish(*canonical);
  }

  std::vector<std::string> joints_;
  std::optional<int64_t> clock_stamp_;
  std::optional<int64_t> state_stamp_;
  bool reset_required_{false};
  bool invalid_sample_{false};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
};
}  // namespace arm_cell_sim_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<arm_cell_sim_adapter::JointStateAdapter>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("joint_state_adapter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
