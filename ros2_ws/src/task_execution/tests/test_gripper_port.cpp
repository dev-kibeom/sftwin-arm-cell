#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <vector>

#include "arm_cell_task_execution/task_executor.hpp"
#include "legacy_backend_motion_port.hpp"

namespace arm_cell_motion
{
using namespace arm_cell_task_execution;

namespace
{
class FakeGripperPort final : public GripperPort
{
public:
  uint8_t close() override
  {
    ++close_calls;
    waiting_for_close_observation = true;
    holding_calls = 0;
    if (auto_observe_close) {
      publish(HoldingState::HELD);
    }
    return close_result;
  }
  uint8_t open() override
  {
    ++open_calls;
    if (open_calls == release_on_open_call || auto_observe_open) {
      publish(HoldingState::RELEASED);
    } else if (unknown_on_open) {
      publish(HoldingState::UNKNOWN);
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
  int close_calls{0};
  int open_calls{0};
  int release_on_open_call{0};
  bool auto_observe_open{false};
  bool auto_observe_close{false};
  bool unknown_on_open{false};
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
    return motion_result;
  }
  void cancel() override {}
  void hold() override {}
  void stop() override {}
  bool execution_active() const override {return false;}
  bool backend_inactivity_confirmed() const override {return true;}

  std::vector<uint8_t> phases;
  uint8_t motion_result{MotionTaskResultCode::TASK_RESULT_SUCCESS};
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
  goal.orientation_symmetry_order = 4;
  goal.vision_acquisition_id = "test-acquisition";
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.pose.orientation.w = 1.0;
  goal.target_pose.pose.position.z = 1.0;
  goal.pick_approach_distance_m = 0.2;
  goal.pick_grasp_position_tolerance_m = 0.01;
  goal.pick_grasp_orientation_tolerance_rad = 0.1;
  goal.has_estimated_object_height = true;
  goal.estimated_object_height_m = 0.1F;
  goal.has_observed_rectangle = true;
  goal.rectangle_long_side_m = 0.07F;
  goal.rectangle_short_side_m = 0.07F;
  goal.grasp_width_min_mm = 10.0F;
  goal.grasp_width_max_mm = 85.0F;
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
  TaskExecutor executor(
    std::make_shared<test::LegacyBackendMotionPort>(backend, valid_geometry()),
    backend->gripper_port(), valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_TRUE(backend->phases.empty());
}

TEST(GripperPortTaskTest, FailedApproachNeverClosesTheGripper)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  backend->motion_result = MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE;
  auto port = std::make_shared<FakeGripperPort>();
  port->observation = observation(HoldingState::RELEASED, std::chrono::milliseconds(10), 1);
  port->auto_observe_open = true;
  TaskExecutor executor(
    std::make_shared<test::LegacyBackendMotionPort>(backend, valid_geometry()), port,
    valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE);
  EXPECT_EQ(port->close_calls, 0);
  ASSERT_EQ(backend->phases.size(), 1U);
}

TEST(GripperPortTaskTest, PickAllowsProgressionOnlyForFreshHeld)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<FakeGripperPort>();
  port->observation = observation(HoldingState::RELEASED, std::chrono::milliseconds(100), 1);
  port->auto_observe_open = true;
  port->auto_observe_close = true;
  TaskExecutor executor(
    std::make_shared<test::LegacyBackendMotionPort>(backend, valid_geometry()), port,
    valid_geometry());

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
  TaskExecutor executor(
    std::make_shared<test::LegacyBackendMotionPort>(backend, valid_geometry()), port,
    valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_EQ(backend->phases.size(), 3U);
}

TEST(GripperPortTaskTest, PickCommandSuccessWithoutHeldObservationFailsClosed)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<FakeGripperPort>();
  port->observation = observation(HoldingState::UNKNOWN, std::chrono::milliseconds(10), 1);
  port->auto_observe_open = true;
  TaskExecutor executor(
    std::make_shared<test::LegacyBackendMotionPort>(backend, valid_geometry()), port,
    valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(backend->phases.size(), 2U);
}

TEST(GripperPortTaskTest, PickStaleHeldObservationFailsClosed)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<FakeGripperPort>();
  port->observation = observation(HoldingState::HELD, std::chrono::milliseconds(501), 1);
  port->auto_observe_open = true;
  TaskExecutor executor(
    std::make_shared<test::LegacyBackendMotionPort>(backend, valid_geometry()), port,
    valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(backend->phases.size(), 2U);
}

TEST(GripperPortTaskTest, PickUnavailableTransportFailsClosed)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<FakeGripperPort>();
  port->close_result = MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
  port->auto_observe_open = true;
  TaskExecutor executor(
    std::make_shared<test::LegacyBackendMotionPort>(backend, valid_geometry()), port,
    valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
  EXPECT_EQ(backend->phases.size(), 2U);
}

TEST(GripperPortTaskTest, PlaceAllowsProgressionOnlyForFreshReleased)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<FakeGripperPort>();
  port->observation = observation(HoldingState::HELD, std::chrono::milliseconds(100), 2);
  port->auto_observe_open = true;
  TaskExecutor executor(
    std::make_shared<test::LegacyBackendMotionPort>(backend, valid_geometry()), port,
    valid_geometry());
  executor.set_object_state(MotionObjectState::ATTACHED);

  EXPECT_EQ(executor.execute(place_goal()), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_EQ(backend->phases.size(), 3U);
}

TEST(GripperPortTaskTest, PlaceReportsInitialReleaseAttempt)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<FakeGripperPort>();
  port->observation = observation(HoldingState::HELD, std::chrono::milliseconds(0), 2);
  port->auto_observe_open = true;
  TaskExecutor executor(
    std::make_shared<test::LegacyBackendMotionPort>(backend, valid_geometry()), port,
    valid_geometry());
  executor.set_object_state(MotionObjectState::ATTACHED);
  struct Report {std::uint8_t phase; std::uint8_t attempt; std::uint8_t retries; std::uint8_t failure;};
  std::vector<Report> reports;
  executor.set_phase_callback([&reports](std::uint8_t phase, std::uint8_t attempt,
      std::uint8_t retries, std::uint8_t failure) {
    if (attempt > 0) {
      reports.push_back({phase, attempt, retries, failure});
    }
  });

  EXPECT_EQ(executor.execute(place_goal()), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_EQ(reports.size(), 1U);
  EXPECT_EQ(reports.front().attempt, 1);
  EXPECT_EQ(reports.front().retries, 3);
  EXPECT_EQ(reports.front().failure,
    arm_cell_interfaces::action::ExecuteTask::Feedback::RELEASE_FAILURE_NONE);
  EXPECT_EQ(reports.front().phase, arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_GRIPPER);
}

TEST(GripperPortTaskTest, PlaceReportsTimeoutAndRetriesOpenUntilFreshReleased)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<FakeGripperPort>();
  port->observation = observation(HoldingState::HELD, std::chrono::milliseconds(0), 2);
  port->release_on_open_call = 2;
  TaskExecutor executor(
    std::make_shared<test::LegacyBackendMotionPort>(backend, valid_geometry()), port,
    valid_geometry(), {}, std::chrono::milliseconds(5));
  executor.set_object_state(MotionObjectState::ATTACHED);
  struct Report {std::uint8_t phase; std::uint8_t attempt; std::uint8_t retries; std::uint8_t failure;};
  std::vector<Report> reports;
  executor.set_phase_callback([&reports](std::uint8_t phase, std::uint8_t attempt,
      std::uint8_t retries, std::uint8_t failure) {
    if (attempt > 0) {
      reports.push_back({phase, attempt, retries, failure});
    }
  });

  EXPECT_EQ(executor.execute(place_goal()), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(port->open_calls, 2);
  EXPECT_EQ(backend->phases.size(), 3U);
  ASSERT_EQ(reports.size(), 2U);
  EXPECT_EQ(reports[1].attempt, 2);
  EXPECT_EQ(reports[1].retries, 3);
  EXPECT_EQ(reports[1].failure,
    arm_cell_interfaces::action::ExecuteTask::Feedback::RELEASE_FAILURE_CONFIRMATION_TIMEOUT);
}

TEST(GripperPortTaskTest, PlaceStopsAfterFourReleaseTimeoutsWithoutRetracting)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<FakeGripperPort>();
  port->observation = observation(HoldingState::HELD, std::chrono::milliseconds(0), 2);
  TaskExecutor executor(
    std::make_shared<test::LegacyBackendMotionPort>(backend, valid_geometry()), port,
    valid_geometry(), {}, std::chrono::milliseconds(2));
  executor.set_object_state(MotionObjectState::ATTACHED);
  struct Report {std::uint8_t attempt; std::uint8_t retries; std::uint8_t failure;};
  std::vector<Report> reports;
  executor.set_phase_callback([&reports](std::uint8_t, std::uint8_t attempt,
      std::uint8_t retries, std::uint8_t failure) {
    if (attempt > 0) {
      reports.push_back({attempt, retries, failure});
    }
  });

  EXPECT_EQ(executor.execute(place_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  ASSERT_EQ(port->open_calls, 4);
  ASSERT_EQ(reports.size(), 4U);
  EXPECT_EQ(reports.back().attempt, 4);
  EXPECT_EQ(reports.back().retries, 3);
  EXPECT_EQ(reports.back().failure,
    arm_cell_interfaces::action::ExecuteTask::Feedback::RELEASE_FAILURE_CONFIRMATION_TIMEOUT);
  EXPECT_EQ(backend->phases.size(), 2U);
}

TEST(GripperPortTaskTest, PlaceDoesNotRetryUnknownBackendCancelOrSafetyOutcomes)
{
  const auto execute = [](uint8_t expected, bool unknown, bool unavailable, bool safety) {
      auto backend = std::make_shared<FakeMotionBackend>();
      auto port = std::make_shared<FakeGripperPort>();
      port->observation = observation(HoldingState::HELD, std::chrono::milliseconds(0), 2);
      port->unknown_on_open = unknown;
      if (unavailable) {
        port->open_result = MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
      }
      TaskExecutor executor(
        std::make_shared<test::LegacyBackendMotionPort>(backend, valid_geometry()), port,
        valid_geometry(), {}, std::chrono::milliseconds(2));
      executor.set_object_state(MotionObjectState::ATTACHED);
      executor.set_interruption_checker([port, safety]() {
        return port->open_calls >= 2 ? (safety ?
          MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED :
          MotionTaskResultCode::TASK_RESULT_CANCELED) :
          MotionTaskResultCode::TASK_RESULT_SUCCESS;
      });
      EXPECT_EQ(executor.execute(place_goal()), expected);
      EXPECT_EQ(port->open_calls, unknown || unavailable ? 1 : 2);
      EXPECT_EQ(backend->phases.size(), 2U);
    };
  execute(MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE, true, false, false);
  execute(MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE, false, true, false);
  execute(MotionTaskResultCode::TASK_RESULT_CANCELED, false, false, false);
  execute(MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED, false, false, true);
}

TEST(GripperPortTaskTest, PlaceCommandSuccessWithoutReleasedObservationFailsClosed)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<FakeGripperPort>();
  port->observation = observation(HoldingState::UNKNOWN, std::chrono::milliseconds(10), 2);
  TaskExecutor executor(
    std::make_shared<test::LegacyBackendMotionPort>(backend, valid_geometry()), port,
    valid_geometry());
  executor.set_object_state(MotionObjectState::ATTACHED);

  EXPECT_EQ(executor.execute(place_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(backend->phases.size(), 2U);
}

TEST(GripperPortTaskTest, PlaceStaleReleasedObservationFailsClosed)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<FakeGripperPort>();
  port->observation = observation(HoldingState::RELEASED, std::chrono::milliseconds(501), 2);
  TaskExecutor executor(
    std::make_shared<test::LegacyBackendMotionPort>(backend, valid_geometry()), port,
    valid_geometry());
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

}  // namespace arm_cell_motion
