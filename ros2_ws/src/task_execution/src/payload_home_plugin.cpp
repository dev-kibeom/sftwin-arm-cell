#include <behaviortree_cpp_v3/bt_factory.h>
#include "arm_cell_task_execution/payload_home_action.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<arm_cell_task_execution::PayloadHomeAction>("ReturnHomeWithPayload");
}
