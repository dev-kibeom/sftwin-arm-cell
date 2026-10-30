#include "arm_cell_task_execution/motion_bt_executor.hpp"

#include <array>
#include <exception>
#include <memory>
#include <string>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include <rclcpp/rclcpp.hpp>

#include "arm_cell_task_execution/bt_callback_node.hpp"
#include "arm_cell_task_execution/candidate_search_control.hpp"
#include "arm_cell_task_execution/disturbance_execution.hpp"
#include "arm_cell_task_execution/task_execution_modules.hpp"

namespace arm_cell_task_execution
{
using namespace arm_cell_motion;
namespace
{
constexpr unsigned kGrootPublisherPort = 1676;
constexpr unsigned kGrootServerPort = 1677;

class IsTaskNode final : public BT::SyncActionNode
{
public:
  IsTaskNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {return {BT::InputPort<std::string>("type")};}

private:
  BT::NodeStatus tick() override
  {
    const auto task_type = config().blackboard->get<std::uint8_t>("task_type");
    std::string expected;
    if (!getInput("type", expected)) {return BT::NodeStatus::FAILURE;}
    const bool matches =
      (expected == "PICK" && task_type == MotionTaskType::TASK_TYPE_PICK) ||
      (expected == "PLACE" && task_type == MotionTaskType::TASK_TYPE_PLACE) ||
      (expected == "RETURN_HELD" && task_type == MotionTaskType::TASK_TYPE_RETURN_HELD) ||
      (expected == "GO_HOME" && task_type == MotionTaskType::TASK_TYPE_GO_HOME) ||
      (expected == "RETRACT" && task_type == MotionTaskType::TASK_TYPE_RETRACT);
    return matches ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class IsStrategyNode final : public BT::SyncActionNode
{
public:
  IsStrategyNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("task"), BT::InputPort<std::string>("strategy")};
  }

private:
  BT::NodeStatus tick() override
  {
    std::string task;
    std::string expected;
    if (!getInput("task", task) || !getInput("strategy", expected)) {
      return BT::NodeStatus::FAILURE;
    }
    const auto key = task == "PICK" ? "pick_strategy" :
      task == "PLACE" ? "place_strategy" : "";
    if (key[0] == '\0') {return BT::NodeStatus::FAILURE;}
    std::string selected_strategy = "PLANNED";
    config().blackboard->get(key, selected_strategy);
    return selected_strategy == expected ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class UnsupportedWaypointNode final : public BT::SyncActionNode
{
public:
  UnsupportedWaypointNode(
    const std::string & name, const BT::NodeConfiguration & config,
    TaskExecutionReport & report)
  : BT::SyncActionNode(name, config), report_(report) {}

  static BT::PortsList providedPorts() {return {BT::InputPort<std::string>("task")};}

private:
  BT::NodeStatus tick() override
  {
    std::string task;
    if (!getInput("task", task)) {return BT::NodeStatus::FAILURE;}
    report_.result_code = MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
    report_.failure_stage = "strategy:" + task + "_WAYPOINT_UNSUPPORTED";
    return BT::NodeStatus::FAILURE;
  }

  TaskExecutionReport & report_;
};

void register_task_selector(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<IsTaskNode>("IsTask");
  factory.registerNodeType<IsStrategyNode>("IsStrategy");
}
}  // namespace

MotionBtExecutionResult MotionBtExecutor::execute(
  const ExecuteTask::Goal & goal, const std::string & motion_execution_id,
  TaskExecutionReport & report, const PickPlaceStrategies & strategies)
{
  MotionBtExecutionResult execution;
  report = TaskExecutionReport{};
  report.motion_execution_id = motion_execution_id;
  report.result_code = MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
  report.object_disposition =
    goal.task_type.value == MotionTaskType::TASK_TYPE_GO_HOME ||
    goal.task_type.value == MotionTaskType::TASK_TYPE_RETRACT ?
    ExecuteTask::Result::OBJECT_DISPOSITION_UNCHANGED :
    ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;

  auto blackboard = BT::Blackboard::create();
  blackboard->set("goal", goal);
  blackboard->set("task_type", goal.task_type.value);
  blackboard->set("pick_strategy", pick_place_strategy_name(strategies.pick));
  blackboard->set("place_strategy", pick_place_strategy_name(strategies.place));

  PickCandidateSearch candidate_search(support_, CandidatePriorityPolicy::DISTANCE_FIRST);
  PickExecution pick_execution(support_);
  PickRecovery pick_recovery(support_);
  PlaceExecution place_execution(support_);
  PlaceRecovery place_recovery(support_);
  HeldObjectReturn held_object_return(support_);

  BT::BehaviorTreeFactory factory;
  register_candidate_search_control(factory);
  register_task_selector(factory);
  factory.registerBuilder<UnsupportedWaypointNode>(
    "UnsupportedWaypoint",
    [&report](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<UnsupportedWaypointNode>(name, config, report);
    });
  candidate_search.register_nodes(factory, blackboard, report);
  pick_execution.register_nodes(factory, blackboard, report);
  pick_recovery.register_nodes(factory, report);
  place_execution.register_nodes(factory, blackboard, report);
  place_recovery.register_nodes(factory, goal, report);
  held_object_return.register_nodes(factory, goal, report);
  register_callback_action(
    factory, "ExecuteDirectTask", [this, &goal, &report, &motion_execution_id]() {
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_RETRACT &&
      support_.object_state_tracker().state() == MotionObjectState::UNCERTAIN)
      {
        report.result_code = MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
        return BT::NodeStatus::FAILURE;
      }
      report.failure_stage = goal.task_type.value == MotionTaskType::TASK_TYPE_GO_HOME ?
      "execution:go_home_backend_request" : "execution:retract_backend_request";
      if (support_.phase_callback()) {
        support_.phase_callback()(
          arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_PLANNING, 0, 0, 0);
      }
      report.result_code = support_.execute_legacy_movement(goal.task_type.value);
      return report.result_code == MotionTaskResultCode::TASK_RESULT_SUCCESS ?
      BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });

  const auto config_directory = ament_index_cpp::get_package_share_directory(
    "arm_cell_task_execution") + "/config/";
  for (const auto * file : std::array<const char *, 6>{
      "pick_recovery.xml", "pick_execution.xml", "place_recovery.xml",
      "pick_candidate_search.xml", "place_execution.xml", "motion_tasks.xml"})
  {
    factory.registerBehaviorTreeFromFile(config_directory + file);
  }
  auto tree = factory.createTree("MotionTask", blackboard);
  std::unique_ptr<BT::PublisherZMQ> groot_publisher;
  try {
    groot_publisher = std::make_unique<BT::PublisherZMQ>(
      tree, 25, kGrootPublisherPort, kGrootServerPort);
  } catch (const std::exception & error) {
    RCLCPP_WARN_ONCE(
      rclcpp::get_logger("arm_cell_task_execution"),
      "Groot MotionTask monitor unavailable; task will continue: %s", error.what());
  }
  (void)tree.tickRoot();

  candidate_search.record_search();
  execution.attempted_material_candidate_ids = candidate_search.attempted_candidate_ids();
  execution.outcome_material_candidate_id = candidate_search.selected_candidate_id();
  return execution;
}

MotionDisturbResult MotionBtExecutor::execute_disturb(const MotionDisturbRequest & request)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("disturbance_request", request);
  DisturbanceExecution disturbance(support_, request);
  BT::BehaviorTreeFactory factory;
  register_candidate_search_control(factory);
  disturbance.register_nodes(factory);
  const auto config_directory = ament_index_cpp::get_package_share_directory(
    "arm_cell_task_execution") + "/config/";
  factory.registerBehaviorTreeFromFile(config_directory + "disturbance_execution.xml");
  auto tree = factory.createTree("DisturbanceExecution", blackboard);
  (void)tree.tickRoot();
  return disturbance.result();
}

}  // namespace arm_cell_task_execution
