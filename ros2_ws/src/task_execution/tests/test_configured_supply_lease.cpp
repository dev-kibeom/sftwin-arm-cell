#include <gtest/gtest.h>
#include "arm_cell_task_execution/configured_supply_lease.hpp"
using namespace arm_cell_task_execution;
TEST(ConfiguredSupplyLease, RequiresLiveAuthoritativeMatchingOccupiedReservation)
{
  ConfiguredSupplyLease lease;
  const auto now = ConfiguredSupplyLease::Clock::now();
  arm_cell_motion::GraspExecutionIdentity identity{
    "11111111111111111111111111111111", "Tape", "supply", "slot_0", "epoch", "token"};
  EXPECT_FALSE(lease.matches(identity, now));
  arm_cell_interfaces::msg::ConfiguredSupplyState state;
  state.supply_id = "supply"; state.material_id = "Tape"; state.supply_epoch = "epoch";
  state.slot_ids = {"slot_0"}; state.occupied = {true}; state.selected_slot_id = "slot_0";
  state.reservation_id = "token"; state.valid = true;
  lease.update(state, now);
  EXPECT_TRUE(lease.matches(identity, now));
  EXPECT_FALSE(lease.matches(identity, now + std::chrono::milliseconds(501)));
  auto forged = identity; forged.reservation_id = "forged";
  EXPECT_FALSE(lease.matches(forged, now));
  state.supply_epoch = "next"; lease.update(state, now);
  EXPECT_FALSE(lease.matches(identity, now));
  state.supply_epoch = "epoch"; state.occupied = {false}; lease.update(state, now);
  EXPECT_FALSE(lease.matches(identity, now));
  state.occupied = {true}; state.valid = false; lease.update(state, now);
  EXPECT_FALSE(lease.matches(identity, now));
}

TEST(ConfiguredSupplyLease, ActiveConveyorSeparatesSilenceFromRevocation)
{
  ConfiguredSupplyLease lease;
  const auto now = ConfiguredSupplyLease::Clock::now();
  arm_cell_motion::GraspExecutionIdentity identity{
    "goal", "Part", "supply", "pick", "epoch", "token", "A", "stream/1"};
  arm_cell_interfaces::msg::ConfiguredSupplyState state;
  state.supply_id = "supply"; state.material_id = "Part"; state.supply_epoch = "epoch";
  state.slot_ids = {"pick"}; state.occupied = {true}; state.selected_slot_id = "pick";
  state.reservation_id = "token"; state.material_instance_id = "stream/1";
  state.material_type_id = "A"; state.valid = true;
  lease.update(state, now);
  EXPECT_FALSE(lease.observed_at_pick(identity, now));
  state.reserved_material_at_pick = true;
  lease.update(state, now);
  EXPECT_TRUE(lease.observed_at_pick(identity, now));
  EXPECT_FALSE(lease.observed_at_pick(identity, now + std::chrono::milliseconds(500)));
  EXPECT_FALSE(lease.matches(identity, now + std::chrono::seconds(6)));
  EXPECT_TRUE(lease.retains(identity, now + std::chrono::seconds(6), true));
  EXPECT_FALSE(lease.retains(identity, now + std::chrono::seconds(6), false));
  state.valid = false; state.observation_pending = true;
  lease.update(state, now);
  EXPECT_FALSE(lease.matches(identity, now));
  EXPECT_TRUE(lease.retains(identity, now, true));
  state.observation_pending = false;
  lease.update(state, now);
  EXPECT_FALSE(lease.observed_at_pick(identity, now));
  EXPECT_FALSE(lease.retains(identity, now, true));
  state.valid = true; state.material_instance_id = "stream/2";
  lease.update(state, now);
  EXPECT_FALSE(lease.observed_at_pick(identity, now));
  EXPECT_FALSE(lease.retains(identity, now, true));
}
