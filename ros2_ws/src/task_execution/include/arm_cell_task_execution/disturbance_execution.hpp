#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <arm_cell_interfaces/msg/disturbance_candidate.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "arm_cell_motion/disturbance_motion.hpp"
#include "arm_cell_motion/motion_port.hpp"
#include "arm_cell_task_execution/task_execution_support.hpp"

namespace arm_cell_task_execution
{

class DisturbanceExecution final
{
public:
  DisturbanceExecution(
    TaskExecutionSupport & support, arm_cell_motion::MotionDisturbRequest request);
  void register_nodes(BT::BehaviorTreeFactory & factory);
  arm_cell_motion::MotionDisturbResult result() const;

private:
  bool prepare_search();
  bool select_candidate();
  bool select_direction();
  bool prepare_route();
  arm_cell_motion::MotionPortRequest route_request(std::size_t index) const;
  bool allow_direction_transition() const;
  bool allow_candidate_transition() const;
  TaskExecutionSupport & support_;
  arm_cell_motion::MotionDisturbRequest request_;
  std::vector<std::size_t> candidate_order_;
  std::size_t candidate_cursor_{0};
  std::size_t selected_candidate_{0};
  std::size_t direction_cursor_{0};
  arm_cell_motion::DisturbanceDirection selected_direction_{
    arm_cell_motion::DisturbanceDirection::POSITIVE_X};
  std::vector<arm_cell_motion::MotionPortRequest> route_requests_;
  std::vector<arm_cell_motion::MotionPlan> planned_route_;
  std::uint8_t last_result_{
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED};
  bool execution_started_{false};
};

}  // namespace arm_cell_task_execution
