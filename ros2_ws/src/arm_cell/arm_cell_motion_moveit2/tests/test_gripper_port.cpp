#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <vector>

#include "arm_cell_motion_moveit2/task_executor.hpp"

namespace arm_cell_motion_moveit2
{

namespace
{
class FakeGripperPort final : public GripperPort
{
public:
  uint8_t close(float) override
  {
    waiting_for_close_observation = true;
    holding_calls = 0;
    if (auto_observe_close) {
      publish(HoldingState::HELD);
    }
    return close_result;
  }
  uint8_t open() override
  {
    if (auto_observe_open) {
      publish(HoldingState::RELEASED);
    }
    return open_result;
  }
  HoldingObservation holding() const override
  {
    ++holding_calls;
    if (waiting_for_close_observation && held_after_polls > 0 &&
      holding_calls >= held_after_polls &&
      observation.state != HoldingState::HELD)
    {
      observation.state = HoldingState::HELD;
      observation.observed_at = HoldingObservation::Clock::now();
      ++observation.sequence;
    }
    return observation;
  }
  bool active() const override {return active_value;}
  void stop() override
  {
    ++stop_calls;
    active_value = false;
  }

  uint8_t close_result{MotionTaskResultCode::TASK_RESULT_SUCCESS};
  uint8_t open_result{MotionTaskResultCode::TASK_RESULT_SUCCESS};
  mutable HoldingObservation observation;
  mutable bool active_value{false};
  int stop_calls{0};
  bool auto_observe_open{false};
  bool auto_observe_close{false};
  mutable bool waiting_for_close_observation{false};
  mutable int holding_calls{0};
  int held_after_polls{0};

  void publish(HoldingState state)
  {
    observation.state = state;
    observation.observed_at = HoldingObservation::Clock::now();
    ++observation.sequence;
  }
};

class MotionOnlyBackend final : public MotionBackend
{
public:
  bool available() const override {return true;}
  bool submit(const ExecuteTask::Goal &) override {return true;}
  uint8_t execute_normalized(const NormalizedMotionRequest & request) override
  {
    phases.push_back(request.phase);
    return MotionTaskResultCode::TASK_RESULT_SUCCESS;
  }
  void cancel() override {}
  void hold() override {}
  void stop() override {}
  bool execution_active() const override {return false;}
  bool backend_inactivity_confirmed() const override {return true;}

  std::vector<uint8_t> phases;
};

class FakeMotionBackend final : public MotionBackend
{
public:
  bool available() const override {return true;}
  bool submit(const ExecuteTask::Goal &) override {return true;}
  uint8_t execute_normalized(const NormalizedMotionRequest & request) override
  {
    phases.push_back(request.phase);
    return motion_result;
  }
  void cancel() override {}
  void hold() override {}
  void stop() override {}
  bool execution_active() const override {return false;}
  bool backend_inactivity_confirmed() const override {return true;}

  uint8_t motion_result{MotionTaskResultCode::TASK_RESULT_SUCCESS};
  std::vector<uint8_t> phases;
};

ExecuteTask::Goal pick_goal()
{
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_PICK;
  goal.has_target_pose = true;
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.pose.orientation.w = 1.0;
  goal.target_pose.pose.position.z = 1.0;
  goal.has_grasp_width = true;
  goal.grasp_width_mm = 20.0F;
  return goal;
}

ExecuteTask::Goal place_goal()
{
  auto goal = pick_goal();
  goal.task_type.value = MotionTaskType::TASK_TYPE_PLACE;
  goal.place_approach_direction_object.z = 1.0;
  goal.place_approach_distance_m = 0.2;
  goal.place_retract_distance_m = 0.4;
  return goal;
}

MotionGeometry valid_geometry()
{
  MotionGeometry geometry;
  geometry.configured = true;
  geometry.observation_reference = "profile_reference";
  geometry.observation_to_object.rotation.w = 1.0;
  geometry.object_to_grasp_tcp.rotation.w = 1.0;
  geometry.insertion_axis_tcp.z = 1.0;
  geometry.pick_approach_distance_m = 0.2;
  geometry.pick_retract_distance_m = 0.4;
  return geometry;
}

HoldingObservation observation(HoldingState state, std::chrono::milliseconds age, uint64_t sequence)
{
  HoldingObservation result;
  result.state = state;
  result.observed_at = HoldingObservation::Clock::now() - age;
  result.sequence = sequence;
  return result;
}

}  // namespace

TEST(GripperPortTaskTest, MissingPortFailsClosedWithoutLegacyFallback)
{
  auto backend = std::make_shared<MotionOnlyBackend>();
  TaskExecutor executor(backend, valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_TRUE(backend->phases.empty());
}

TEST(GripperPortTaskTest, PickAllowsProgressionOnlyForFreshHeld)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<FakeGripperPort>();
  port->observation = observation(HoldingState::RELEASED, std::chrono::milliseconds(100), 1);
  port->auto_observe_open = true;
  port->auto_observe_close = true;
  TaskExecutor executor(backend, port, valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_EQ(backend->phases.size(), 3U);
}

TEST(GripperPortTaskTest, PendingCloseObservationCanBeFollowedByFreshHeld)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<FakeGripperPort>();
  port->observation = observation(HoldingState::UNKNOWN, std::chrono::milliseconds(10), 0);
  port->auto_observe_open = true;
  port->held_after_polls = 2;
  TaskExecutor executor(backend, port, valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_EQ(backend->phases.size(), 3U);
}

TEST(GripperPortTaskTest, PickCommandSuccessWithoutHeldObservationFailsClosed)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<FakeGripperPort>();
  port->observation = observation(HoldingState::UNKNOWN, std::chrono::milliseconds(10), 1);
  port->auto_observe_open = true;
  TaskExecutor executor(backend, port, valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(backend->phases.size(), 2U);
}

TEST(GripperPortTaskTest, PickStaleHeldObservationFailsClosed)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<FakeGripperPort>();
  port->observation = observation(HoldingState::HELD, std::chrono::milliseconds(501), 1);
  port->auto_observe_open = true;
  TaskExecutor executor(backend, port, valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(backend->phases.size(), 2U);
}

TEST(GripperPortTaskTest, PickUnavailableTransportFailsClosed)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<FakeGripperPort>();
  port->close_result = MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
  port->auto_observe_open = true;
  TaskExecutor executor(backend, port, valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
  EXPECT_EQ(backend->phases.size(), 2U);
}

TEST(GripperPortTaskTest, PlaceAllowsProgressionOnlyForFreshReleased)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<FakeGripperPort>();
  port->observation = observation(HoldingState::HELD, std::chrono::milliseconds(100), 2);
  port->auto_observe_open = true;
  TaskExecutor executor(backend, port, valid_geometry());
  executor.set_object_state(MotionObjectState::ATTACHED);

  EXPECT_EQ(executor.execute(place_goal()), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_EQ(backend->phases.size(), 3U);
}

TEST(GripperPortTaskTest, PlaceCommandSuccessWithoutReleasedObservationFailsClosed)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<FakeGripperPort>();
  port->observation = observation(HoldingState::UNKNOWN, std::chrono::milliseconds(10), 2);
  TaskExecutor executor(backend, port, valid_geometry());
  executor.set_object_state(MotionObjectState::ATTACHED);

  EXPECT_EQ(executor.execute(place_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(backend->phases.size(), 2U);
}

TEST(GripperPortTaskTest, PlaceStaleReleasedObservationFailsClosed)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<FakeGripperPort>();
  port->observation = observation(HoldingState::RELEASED, std::chrono::milliseconds(501), 2);
  TaskExecutor executor(backend, port, valid_geometry());
  executor.set_object_state(MotionObjectState::ATTACHED);

  EXPECT_EQ(executor.execute(place_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(backend->phases.size(), 2U);
}

TEST(GripperPortTaskTest, ActivePortCanBeStoppedWithoutLeakingActiveState)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<FakeGripperPort>();
  port->active_value = true;

  ASSERT_TRUE(port->active());
  port->stop();
  EXPECT_FALSE(port->active());
  EXPECT_EQ(port->stop_calls, 1);
}

}  // namespace arm_cell_motion_moveit2
