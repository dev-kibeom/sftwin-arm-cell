#pragma once
#include <functional>
#include <behaviortree_cpp_v3/action_node.h>

namespace arm_cell_task_execution
{
// The owning executor supplies the operation. Keeping the BT node independent
// allows deployment-specific plugins to replace it without owning robot state.
class PayloadHomeAction : public BT::SyncActionNode
{
public:
  using Operation = std::function<BT::NodeStatus()>;
  PayloadHomeAction(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() {return {};}

private:
  BT::NodeStatus tick() override
  {
    return config().blackboard->get<Operation>("payload_home_operation")();
  }
};
}  // namespace arm_cell_task_execution
