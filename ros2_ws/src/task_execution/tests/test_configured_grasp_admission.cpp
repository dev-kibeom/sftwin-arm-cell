#include <gtest/gtest.h>
#include <atomic>
#include <thread>
#include "arm_cell_task_execution/execute_configured_grasp_action_server.hpp"
#include <arm_cell_interfaces/msg/configured_supply_state.hpp>

namespace arm_cell_task_execution
{
namespace
{
using Code = arm_cell_interfaces::msg::MotionTaskResultCode;
class LeaseMotion : public MotionPort
{
public:
  std::atomic_int moves{0}, cancellations{0};
  std::atomic_bool canceled{false};
  arm_cell_motion::MotionPortResult planAndExecuteMotion(const arm_cell_motion::MotionPortRequest &)
  override {return {};}
  arm_cell_motion::MotionObservation observeMotionState() const override {return {};}
  arm_cell_motion::MotionCancelResult cancelMotion(
    const unique_identifier_msgs::msg::UUID &,
    const std::string &) override
  {++cancellations; canceled = true; return {};}
  std::uint8_t executeTaughtMotion(
    const std::string &, const std::string &,
    const std::string &) override
  {
    ++moves;
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!canceled && std::chrono::steady_clock::now() < end) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return canceled ? Code::TASK_RESULT_CANCELED : Code::TASK_RESULT_SUCCESS;
  }
};
class LeaseTool : public arm_cell_motion::GripperPort
{
public:
  mutable std::uint64_t sequence{1};
  std::uint8_t close() override {return Code::TASK_RESULT_SUCCESS;}
  std::uint8_t open() override {++sequence; return Code::TASK_RESULT_SUCCESS;}
  bool supportsGraspStrategy(arm_cell_motion::GraspStrategy) const override {return true;}
  std::uint8_t prepareConfiguredGrasp(
    const arm_cell_motion::GraspSpecification &,
    const arm_cell_motion::GraspExecutionIdentity &) override
  {
    return Code::TASK_RESULT_SUCCESS;
  }
  arm_cell_motion::HoldingObservation holding() const override
  {
    return {arm_cell_motion::HoldingState::RELEASED,
      arm_cell_motion::HoldingObservation::Clock::now(), sequence, std::nullopt};
  }
  bool active() const override {return false;}
  void stop() override {}
};
}
TEST(ConfiguredGraspAdmission, ForgedLeaseNeverMovesAndChangedLeaseStopsActiveMotion)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto node = std::make_shared<rclcpp::Node>("configured_lease_test");
  auto motion = std::make_shared<LeaseMotion>();
  auto task = std::make_shared<TaskExecutor>(motion, std::make_shared<LeaseTool>());
  PreparedGraspCycle cycle;
  cycle.grasp.strategy = arm_cell_motion::GraspStrategy::INTERNAL_RADIAL;
  cycle.grasp.insertion_depth_m = .012; cycle.grasp.thickness_m = .017;
  cycle.grasp.bore_radius_m = .025;
  cycle.grasp.release_position_m = .02; cycle.grasp.grasp_position_m = .01;
  cycle.identity.supply_id = "supply"; cycle.identity.material_id = "Tape";
  cycle.identity.slot_id = "slot_0";
  cycle.approach_motion = "approach"; cycle.insertion_motion = "insert";
  cycle.transfer_motion = "transfer"; cycle.retreat_motion = "retreat";
  ExecuteConfiguredGraspActionServer server(node.get(), motion, task, {{"slot_0", cycle}});
  using Action = ExecuteConfiguredGraspActionServer::Action;
  auto client = rclcpp_action::create_client<Action>(
    node,
    "/task_execution/execute_configured_grasp");
  auto publisher = node->create_publisher<arm_cell_interfaces::msg::ConfiguredSupplyState>(
    "/cell/configured_supply/state", 10);
  rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(node);
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(1)));
  arm_cell_interfaces::msg::ConfiguredSupplyState supply;
  supply.supply_id = "supply"; supply.material_id = "Tape"; supply.supply_epoch = "epoch";
  supply.slot_ids = {"slot_0"}; supply.occupied = {true}; supply.selected_slot_id = "slot_0";
  supply.reservation_id = "token"; supply.valid = true;
  const auto pump = [&](int count) {
      for (int i = 0; i < count; ++i) {
        executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    };
  publisher->publish(supply); pump(10);
  Action::Goal goal; goal.supply_id = "supply"; goal.slot_id = "slot_0";
  goal.supply_epoch = "epoch"; goal.reservation_id = "forged";
  auto rejected = client->async_send_goal(goal);
  ASSERT_EQ(
    executor.spin_until_future_complete(
      rejected, std::chrono::seconds(
        1)), rclcpp::FutureReturnCode::SUCCESS);
  const auto wrong = rejected.get();
  EXPECT_FALSE(wrong);
  if (wrong) {
    auto cancel = client->async_cancel_goal(wrong); executor.spin_until_future_complete(
      cancel, std::chrono::seconds(
        1)); pump(10);
  }
  EXPECT_EQ(motion->moves.load(), 0);
  motion->canceled = false;
  goal.reservation_id = "token"; publisher->publish(supply); pump(5);
  {
    std::unique_lock<std::mutex> existing_owner(*task->execution_mutex());
    auto busy_goal = client->async_send_goal(goal);
    ASSERT_EQ(executor.spin_until_future_complete(busy_goal, std::chrono::seconds(1)),
      rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_TRUE(busy_goal.get());
    auto busy = client->async_get_result(busy_goal.get());
    ASSERT_EQ(executor.spin_until_future_complete(busy, std::chrono::seconds(1)),
      rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(busy.get().result->result_code, Code::TASK_RESULT_BUSY);
    EXPECT_EQ(busy.get().result->failure_stage, "configured_supply:execution_admission");
    EXPECT_FALSE(busy.get().result->material_placed);
    EXPECT_EQ(motion->moves.load(), 0);
  }
  auto accepted = client->async_send_goal(goal);
  ASSERT_EQ(
    executor.spin_until_future_complete(
      accepted, std::chrono::seconds(
        1)), rclcpp::FutureReturnCode::SUCCESS);
  auto handle = accepted.get(); ASSERT_TRUE(handle);
  auto result = client->async_get_result(handle);
  for (int i = 0; i < 50 && motion->moves.load() == 0; ++i) {
    publisher->publish(supply); pump(1);
  }
  ASSERT_GT(motion->moves.load(), 0);
  supply.reservation_id = "replacement"; publisher->publish(supply);
  ASSERT_EQ(
    executor.spin_until_future_complete(
      result, std::chrono::seconds(
        2)), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_GT(motion->cancellations.load(), 0);
  EXPECT_EQ(result.get().result->result_code, Code::TASK_RESULT_PERMISSION_DENIED);
  EXPECT_EQ(motion->moves.load(), 1);
  // Restoring the old token does not authorize a second transfer after the
  // first Action has already used that lease, even though Cell COMPLETE has
  // not happened and the slot remains occupied.
  supply.reservation_id = "token"; publisher->publish(supply); pump(5);
  auto replay = client->async_send_goal(goal);
  ASSERT_EQ(executor.spin_until_future_complete(replay, std::chrono::seconds(1)), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_FALSE(replay.get());
  EXPECT_EQ(motion->moves.load(), 1);
  executor.remove_node(node);
  rclcpp::shutdown();
}
}
