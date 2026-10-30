#pragma once

#include <behaviortree_cpp_v3/action_node.h>

#include <functional>
#include <string>
#include <utility>

namespace arm_cell_motion
{

class CallbackActionNode final : public BT::SyncActionNode
{
public:
  using Callback = std::function<BT::NodeStatus()>;

  CallbackActionNode(
    const std::string & name, const BT::NodeConfiguration & config, Callback callback)
  : BT::SyncActionNode(name, config), callback_(std::move(callback)) {}

  static BT::PortsList providedPorts() {return {};}

private:
  BT::NodeStatus tick() override {return callback_();}
  Callback callback_;
};

inline void register_callback_action(
  BT::BehaviorTreeFactory & factory, const std::string & registration_name,
  CallbackActionNode::Callback callback)
{
  factory.registerBuilder<CallbackActionNode>(
    registration_name,
    [callback = std::move(callback)](
      const std::string & node_name, const BT::NodeConfiguration & config) {
      return std::make_unique<CallbackActionNode>(node_name, config, callback);
    });
}

}  // namespace arm_cell_motion
