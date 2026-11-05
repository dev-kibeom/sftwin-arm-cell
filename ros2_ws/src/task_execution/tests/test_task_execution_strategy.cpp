#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>
#include <memory>
#include <string>

#include <arm_cell_interfaces/action/execute_task.hpp>
#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>
#include <arm_cell_interfaces/msg/motion_task_type.hpp>

#include "arm_cell_motion/fake_motion_backend.hpp"
#include "arm_cell_task_execution/task_execution_strategy.hpp"
#include "arm_cell_task_execution/task_executor.hpp"
#include "legacy_backend_motion_port.hpp"

namespace arm_cell_task_execution
{
using arm_cell_interfaces::msg::MotionTaskResultCode;
using arm_cell_interfaces::msg::MotionTaskType;

TEST(TaskExecutionStrategyTest, DefaultsBothTaskStrategiesToPlanned)
{
  const PickPlaceStrategies strategies;
  EXPECT_EQ(strategies.pick, PickPlaceStrategy::PLANNED);
  EXPECT_EQ(strategies.place, PickPlaceStrategy::PLANNED);
}

TEST(TaskExecutionStrategyTest, ParsesPickAndPlaceStrategiesIndependently)
{
  const PickPlaceStrategies strategies{
    parse_pick_place_strategy("PLANNED", "motion.pick_strategy"),
    parse_pick_place_strategy("WAYPOINT", "motion.place_strategy")};
  EXPECT_EQ(strategies.pick, PickPlaceStrategy::PLANNED);
  EXPECT_EQ(strategies.place, PickPlaceStrategy::WAYPOINT);
}

TEST(TaskExecutionStrategyTest, RejectsUnsupportedStrategyWithParameterContext)
{
  try {
    (void)parse_pick_place_strategy("CARTESIAN", "motion.pick_strategy");
    FAIL() << "unsupported strategy was accepted";
  } catch (const std::invalid_argument & error) {
    EXPECT_NE(std::string(error.what()).find("motion.pick_strategy"), std::string::npos);
    EXPECT_NE(std::string(error.what()).find("CARTESIAN"), std::string::npos);
  }
}

TEST(TaskExecutionStrategyTest, PickWaypointRemainsUnsupported)
{
    auto backend = std::make_shared<arm_cell_motion::FakeMotionBackend>();
    const auto gripper = backend->gripper_port();
  const PickPlaceStrategies strategies{PickPlaceStrategy::WAYPOINT, PickPlaceStrategy::PLANNED};
    TaskExecutor executor(
      std::make_shared<test::LegacyBackendMotionPort>(backend, arm_cell_motion::MotionGeometry{}),
      gripper, arm_cell_motion::MotionGeometry{}, TaskExecutionSupport::InterruptionChecker{},
      std::chrono::milliseconds(10), strategies);
    ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_PICK;

    EXPECT_EQ(
    executor.execute(goal, MotionObjectState::NO_OBJECT),
      MotionTaskResultCode::TASK_RESULT_INVALID_GOAL);
    EXPECT_EQ(backend->execute_task_calls(), 0);
    EXPECT_EQ(gripper->holding().sequence, 0U);
  EXPECT_EQ(executor.last_failure_stage(), "strategy:PICK_WAYPOINT_UNSUPPORTED");
    EXPECT_EQ(
      executor.object_disposition(), arm_cell_interfaces::action::ExecuteTask::Result::
      OBJECT_DISPOSITION_UNKNOWN);
  EXPECT_EQ(executor.object_state(), MotionObjectState::NO_OBJECT);
}
}  // namespace arm_cell_task_execution
