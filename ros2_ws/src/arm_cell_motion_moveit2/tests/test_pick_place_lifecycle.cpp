#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <arm_cell_interfaces/msg/motion_task_phase.hpp>

#include "arm_cell_motion_moveit2/task_executor.hpp"

namespace arm_cell_motion_moveit2
{

namespace
{
class RecordingBackend final : public MotionBackend
{
public:
  bool available() const override {return true;}

  bool submit(const ExecuteTask::Goal &) override {return true;}

  uint8_t execute_normalized(const NormalizedMotionRequest & request) override
  {
    requests.push_back(request);
    events.push_back("motion:" + std::to_string(request.phase));
    if (request.task_type.value == MotionTaskType::TASK_TYPE_PICK &&
      request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING)
    {
      geometry_msgs::msg::Transform relation;
      relation.rotation.w = 1.0;
      stage_selected_pick_grasp_relation(relation);
    }
    {
      std::unique_lock<std::mutex> lock(completion_mutex);
      ++motion_call_count;
      completion_condition.notify_all();
      if (block_motion) {
        completion_condition.wait(lock, [this]() {return release_motion;});
        release_motion = false;
      }
    }
    tcp_x_values.push_back(request.tcp_target.pose.position.x);
    tcp_y_values.push_back(request.tcp_target.pose.position.y);
    tcp_z_values.push_back(request.tcp_target.pose.position.z);
    tcp_orientations.push_back(request.tcp_target.pose.orientation);
    approach_z_values.push_back(request.approach_target.pose.position.z);
    retract_x_values.push_back(request.retract_target.pose.position.x);
    return motion_result;
  }

  uint8_t command_gripper(float width_mm) override
  {
    events.push_back("gripper");
    gripper_widths.push_back(width_mm);
    return gripper_result;
  }

  bool gripper_active() const override {return gripper_active_value;}

  void stop_gripper() override {++stop_gripper_calls;}

  void release_current_motion()
  {
    std::lock_guard<std::mutex> lock(completion_mutex);
    release_motion = true;
    completion_condition.notify_all();
  }

  bool wait_for_motion_call(int expected)
  {
    std::unique_lock<std::mutex> lock(completion_mutex);
    return completion_condition.wait_for(
      lock, std::chrono::seconds(1), [this, expected]() {
        return motion_call_count >= expected;
      });
  }

  void cancel() override {}
  void hold() override {}
  void stop() override {}
  bool execution_active() const override {return false;}
  bool backend_inactivity_confirmed() const override {return true;}

  void begin_pick_grasp_relation() override
  {
    pending_relation = false;
    accepted_relation = false;
    ++begin_relation_count;
  }

  void stage_selected_pick_grasp_relation(
    const geometry_msgs::msg::Transform &) override
  {
    pending_relation = true;
  }

  void discard_pending_grasp_relation() override
  {
    pending_relation = false;
    ++discard_relation_count;
  }

  bool confirm_fresh_held_grasp_relation() override
  {
    if (!pending_relation) {
      return false;
    }
    pending_relation = false;
    accepted_relation = true;
    ++promote_relation_count;
    return true;
  }

  void confirm_fresh_released_grasp_relation() override
  {
    pending_relation = false;
    accepted_relation = false;
    ++release_relation_count;
  }

  std::vector<std::string> events;
  std::vector<double> tcp_z_values;
  std::vector<double> tcp_x_values;
  std::vector<double> tcp_y_values;
  std::vector<geometry_msgs::msg::Quaternion> tcp_orientations;
  std::vector<double> approach_z_values;
  std::vector<double> retract_x_values;
  std::vector<NormalizedMotionRequest> requests;
  std::vector<float> gripper_widths;
  uint8_t motion_result{MotionTaskResultCode::TASK_RESULT_SUCCESS};
  uint8_t gripper_result{MotionTaskResultCode::TASK_RESULT_SUCCESS};
  bool gripper_active_value{false};
  std::atomic_bool gripper_commanded{false};
  std::atomic_bool gripper_open_commanded{false};
  int stop_gripper_calls{0};
  bool block_motion{false};
  bool release_motion{false};
  int motion_call_count{0};
  bool pending_relation{false};
  bool accepted_relation{false};
  int begin_relation_count{0};
  int discard_relation_count{0};
  int promote_relation_count{0};
  int release_relation_count{0};
  std::mutex completion_mutex;
  std::condition_variable completion_condition;
};

class RecordingGripperPort final : public GripperPort
{
public:
  explicit RecordingGripperPort(RecordingBackend & backend)
  : backend_(backend)
  {
  }

  uint8_t close(float width_mm) override
  {
    backend_.events.push_back("gripper");
    backend_.gripper_widths.push_back(width_mm);
    backend_.gripper_commanded.store(true);
    if (backend_.gripper_result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      return backend_.gripper_result;
    }
    if (auto_observe) {
      publish(HoldingState::HELD);
    }
    return MotionTaskResultCode::TASK_RESULT_SUCCESS;
  }

  uint8_t open() override
  {
    backend_.events.push_back("gripper");
    backend_.gripper_widths.push_back(85.0F);
    backend_.gripper_open_commanded.store(true);
    if (backend_.gripper_result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      return backend_.gripper_result;
    }
    if (open_auto_observe) {
      publish(HoldingState::RELEASED);
    }
    return MotionTaskResultCode::TASK_RESULT_SUCCESS;
  }

  HoldingObservation holding() const override {return observation_;}
  bool active() const override {return backend_.gripper_active();}
  void stop() override {backend_.stop_gripper();}

  void publish(HoldingState state)
  {
    observation_.state = state;
    observation_.observed_at = HoldingObservation::Clock::now();
    ++observation_.sequence;
  }

  void set_observation(HoldingState state, std::uint64_t sequence)
  {
    observation_.state = state;
    observation_.observed_at = HoldingObservation::Clock::now();
    observation_.sequence = sequence;
  }

  bool auto_observe{true};
  bool open_auto_observe{true};

private:
  RecordingBackend & backend_;
  HoldingObservation observation_;
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

MotionGeometry valid_geometry()
{
  MotionGeometry geometry;
  geometry.configured = true;
  geometry.observation_reference = "profile_reference";
  geometry.observation_to_object.rotation.w = 1.0;
  geometry.object_to_grasp_tcp.rotation.w = 1.0;
  geometry.object_to_grasp_tcp.translation.z = 0.1;
  geometry.object_to_grasp_tcp.rotation.w = 1.0;
  geometry.insertion_axis_tcp.z = 1.0;
  geometry.pick_approach_distance_m = 0.2;
  geometry.pick_retract_distance_m = 0.4;
  return geometry;
}

ExecuteTask::Goal place_goal()
{
  auto goal = pick_goal();
  goal.task_type.value = MotionTaskType::TASK_TYPE_PLACE;
  goal.place_position_tolerance_m = 0.01;
  goal.place_orientation_tolerance_rad = 0.02;
  goal.place_orientation_constraint = ExecuteTask::Goal::PLACE_ORIENTATION_BOUNDED;
  goal.place_approach_direction_object.z = 1.0;
  goal.place_approach_distance_m = 0.2;
  goal.place_retract_distance_m = 0.4;
  return goal;
}
}  // namespace

TEST(TaskExecutorPickPlaceTest, PickOrdersApproachGraspAttachRetractAndConvertsTargetInMotion)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto geometry = valid_geometry();
  geometry.pick_approach_distance_m = 0.15;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    geometry);
  auto goal = pick_goal();
  goal.has_place_tool_orientation_preference = true;
  goal.place_tool_orientation_preference.x = 1.0;
  goal.place_tool_orientation_preference.w = 0.0;

  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(
    backend->events,
    (std::vector<std::string>{
      "gripper", "motion:3", "motion:4", "gripper", "motion:8"}));
  EXPECT_EQ(backend->gripper_widths, (std::vector<float>{85.0F, 20.0F}));
  ASSERT_EQ(backend->tcp_z_values.size(), 3U);
  ASSERT_EQ(backend->approach_z_values.size(), 3U);
  ASSERT_EQ(backend->requests.size(), 3U);
  for (const auto & request : backend->requests) {
    EXPECT_FALSE(request.has_place_tool_orientation_preference);
  }
  EXPECT_DOUBLE_EQ(backend->tcp_z_values.front(), 0.95);
  EXPECT_DOUBLE_EQ(backend->tcp_z_values[1], 1.1);
  EXPECT_DOUBLE_EQ(backend->tcp_z_values.back(), 0.7);
  EXPECT_DOUBLE_EQ(backend->approach_z_values.front(), 0.95);
  EXPECT_DOUBLE_EQ(backend->requests.front().approach_distance_m, 0.15);
  ASSERT_EQ(backend->retract_x_values.size(), 3U);
  EXPECT_DOUBLE_EQ(backend->retract_x_values.back(), 0.0);
  EXPECT_EQ(executor.object_state(), MotionObjectState::ATTACHED);
  EXPECT_TRUE(backend->accepted_relation);
  EXPECT_EQ(backend->promote_relation_count, 1);
}

TEST(TaskExecutorPickPlaceTest, FailedPickDiscardsSelectedRelationBeforeHeld)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->motion_result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);
  EXPECT_FALSE(backend->pending_relation);
  EXPECT_FALSE(backend->accepted_relation);
  EXPECT_EQ(backend->promote_relation_count, 0);
  EXPECT_EQ(backend->discard_relation_count, 1);
}

TEST(TaskExecutorPickPlaceTest, FreshReleaseInvalidatesAcceptedRelation)
{
  auto backend = std::make_shared<RecordingBackend>();
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  ASSERT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_TRUE(backend->accepted_relation);
  EXPECT_EQ(executor.execute(place_goal()), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_FALSE(backend->pending_relation);
  EXPECT_FALSE(backend->accepted_relation);
  EXPECT_EQ(backend->release_relation_count, 1);
}

TEST(TaskExecutorPickPlaceTest, ResolvesProfileObservationToObjectCenterBeforeTargetTransform)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto geometry = valid_geometry();
  geometry.observation_to_object.translation.z = -0.04;
  geometry.object_to_grasp_tcp.translation.z = 0.0;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend), geometry);

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_FALSE(backend->requests.empty());
  const auto & approach_request = backend->requests.front();
  EXPECT_DOUBLE_EQ(approach_request.observation_pose.pose.position.z, 1.0);
  EXPECT_DOUBLE_EQ(approach_request.object_pose.pose.position.z, 0.96);
  EXPECT_DOUBLE_EQ(approach_request.target_pose.pose.position.z, 0.96);
  EXPECT_DOUBLE_EQ(approach_request.approach_target.pose.position.z, 0.76);
  ASSERT_EQ(backend->tcp_z_values.size(), 3U);
  EXPECT_DOUBLE_EQ(backend->tcp_z_values[0], 0.76);
  EXPECT_DOUBLE_EQ(backend->tcp_z_values[1], 0.96);
  EXPECT_DOUBLE_EQ(backend->tcp_z_values[2], 0.56);
}

TEST(TaskExecutorPickPlaceTest, PlaceTreatsRecipePoseAsDesiredObjectPose)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto geometry = valid_geometry();
  geometry.observation_to_object.translation.z = -0.04;
  geometry.object_to_grasp_tcp.translation.z = 0.1;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend), geometry);
  auto goal = place_goal();
  goal.target_pose.pose.position.z = 0.23;
  goal.target_pose.pose.orientation.z = 0.7071067811865476;
  goal.target_pose.pose.orientation.w = 0.7071067811865476;
  goal.has_place_tool_orientation_preference = true;
  goal.place_tool_orientation_preference.x = 1.0;
  goal.place_tool_orientation_preference.w = 0.0;
  goal.place_approach_direction_object.z = 1.0;
  goal.place_approach_distance_m = 0.2;
  goal.place_retract_distance_m = 0.4;
  executor.set_object_state(MotionObjectState::ATTACHED);

  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_FALSE(backend->requests.empty());
  const auto & approach_request = backend->requests.front();
  EXPECT_DOUBLE_EQ(approach_request.observation_pose.pose.position.z, 0.0);
  EXPECT_DOUBLE_EQ(approach_request.object_pose.pose.position.z, 0.23);
  EXPECT_DOUBLE_EQ(approach_request.object_pose.pose.orientation.z, 0.7071067811865476);
  EXPECT_DOUBLE_EQ(approach_request.target_pose.pose.position.z, 0.33);
  EXPECT_DOUBLE_EQ(approach_request.target_pose.pose.orientation.z, 0.7071067811865476);
  EXPECT_DOUBLE_EQ(approach_request.target_pose.pose.orientation.w, 0.7071067811865476);
  EXPECT_TRUE(approach_request.has_place_tool_orientation_preference);
  EXPECT_DOUBLE_EQ(approach_request.place_tool_orientation_preference.x, 1.0);
  EXPECT_DOUBLE_EQ(approach_request.approach_target.pose.position.z, 0.53);
  EXPECT_DOUBLE_EQ(approach_request.place_position_tolerance_m, 0.01);
  EXPECT_DOUBLE_EQ(approach_request.place_orientation_tolerance_rad, 0.02);
  EXPECT_EQ(
    approach_request.place_orientation_constraint, PlaceOrientationConstraint::BOUNDED);
}

TEST(TaskExecutorPickPlaceTest, RecipeApproachDistanceChangesObjectCentricPlaceApproach)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto geometry = valid_geometry();
  geometry.object_to_grasp_tcp.translation.z = 0.1;
  auto goal = place_goal();
  goal.target_pose.pose.position.z = 0.23;
  goal.place_approach_distance_m = 0.3;
  goal.place_retract_distance_m = 0.4;
  TaskExecutor executor_with_recipe_distance(
    backend, std::make_shared<RecordingGripperPort>(*backend), geometry);
  executor_with_recipe_distance.set_object_state(MotionObjectState::ATTACHED);

  ASSERT_EQ(executor_with_recipe_distance.execute(goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_FALSE(backend->requests.empty());
  EXPECT_DOUBLE_EQ(backend->requests.front().approach_target.pose.position.z, 0.63);
  EXPECT_DOUBLE_EQ(backend->requests.front().retract_target.pose.position.z, 0.73);
}

TEST(TaskExecutorPickPlaceTest, NonUnitInsertionAxisDoesNotScaleConfiguredDistances)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto geometry = valid_geometry();
  geometry.insertion_axis_tcp.z = 2.0;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend), geometry);

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_EQ(backend->tcp_z_values.size(), 3U);
  EXPECT_DOUBLE_EQ(backend->tcp_z_values[0], 0.9);
  EXPECT_DOUBLE_EQ(backend->tcp_z_values[1], 1.1);
  EXPECT_DOUBLE_EQ(backend->tcp_z_values[2], 0.7);
}

TEST(TaskExecutorPickPlaceTest, PickWaitsForEachMotionPhaseBeforeContinuing)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->block_motion = true;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());
  auto result = std::async(
    std::launch::async, [&executor]() {return executor.execute(pick_goal());});

  ASSERT_TRUE(backend->wait_for_motion_call(1));
  EXPECT_EQ(backend->events, (std::vector<std::string>{"gripper", "motion:3"}));
  EXPECT_EQ(
    result.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);

  backend->release_current_motion();
  ASSERT_TRUE(backend->wait_for_motion_call(2));
  EXPECT_EQ(
    backend->events,
    (std::vector<std::string>{"gripper", "motion:3", "motion:4"}));
  EXPECT_EQ(
    result.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);

  backend->release_current_motion();
  ASSERT_TRUE(backend->wait_for_motion_call(3));
  EXPECT_EQ(
    backend->events,
    (std::vector<std::string>{
      "gripper", "motion:3", "motion:4", "gripper", "motion:8"}));
  EXPECT_EQ(
    result.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);

  backend->release_current_motion();
  EXPECT_EQ(result.get(), MotionTaskResultCode::TASK_RESULT_SUCCESS);
}

TEST(TaskExecutorPickPlaceTest, MotionPhaseFailureDoesNotContinuePick)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->motion_result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);
  EXPECT_EQ(backend->events, (std::vector<std::string>{"gripper", "motion:3"}));
}

TEST(TaskExecutorPickPlaceTest, ObjectYawRotatesTcpTranslationAndComposesToolOrientation)
{
  constexpr double pi = 3.14159265358979323846;
  auto backend = std::make_shared<RecordingBackend>();
  auto geometry = valid_geometry();
  geometry.object_to_grasp_tcp.translation.x = 0.1;
  geometry.object_to_grasp_tcp.translation.z = 0.0;
  geometry.object_to_grasp_tcp.rotation.z = std::sin(pi / 4.0);
  geometry.object_to_grasp_tcp.rotation.w = std::cos(pi / 4.0);
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend), geometry);

  auto goal = pick_goal();
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.z = std::sin(pi / 4.0);
  goal.target_pose.pose.orientation.w = std::cos(pi / 4.0);

  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_EQ(backend->tcp_orientations.size(), 3U);
  ASSERT_EQ(backend->tcp_z_values.size(), 3U);
  EXPECT_NEAR(backend->tcp_x_values[1], 0.0, 1e-9);
  EXPECT_NEAR(backend->tcp_y_values[1], 0.1, 1e-9);
  EXPECT_NEAR(backend->tcp_z_values[1], 0.0, 1e-9);
  EXPECT_NEAR(backend->tcp_orientations[1].z, 1.0, 1e-9);
  EXPECT_NEAR(backend->tcp_orientations[1].w, 0.0, 1e-9);
}

TEST(TaskExecutorPickPlaceTest, PickFailureBeforeAttachmentCannotReportSuccessOrAttach)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->gripper_result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(executor.object_state(), MotionObjectState::NO_OBJECT);
}

TEST(TaskExecutorPickPlaceTest, GripperSuccessWithoutConfirmationCannotAttachOrSucceed)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->gripper_result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(executor.object_state(), MotionObjectState::NO_OBJECT);
  EXPECT_EQ(backend->events.back(), "gripper");
}

TEST(TaskExecutorPickPlaceTest, GraspConfirmationFailureCannotAttachOrSucceed)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->gripper_result = MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(executor.object_state(), MotionObjectState::NO_OBJECT);
}

TEST(TaskExecutorPickPlaceTest, FailedGripperCommandIsStoppedWhenStillActive)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->gripper_result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  backend->gripper_active_value = true;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(backend->stop_gripper_calls, 1);
}

TEST(TaskExecutorPickPlaceTest, AttachmentFailureCannotReportSuccessOrMarkHeld)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->gripper_result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(executor.object_state(), MotionObjectState::NO_OBJECT);
  EXPECT_EQ(backend->events.back(), "gripper");
}

TEST(TaskExecutorPickPlaceTest, PlaceOrdersReleaseDetachRetreatAndClearsOwnership)
{
  auto backend = std::make_shared<RecordingBackend>();
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());
  executor.set_object_state(MotionObjectState::ATTACHED);

  EXPECT_EQ(executor.execute(place_goal()), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(
    backend->events,
    (std::vector<std::string>{
      "motion:3", "motion:4", "gripper", "motion:8"}));
  EXPECT_EQ(backend->gripper_widths, (std::vector<float>{85.0F}));
  EXPECT_EQ(executor.object_state(), MotionObjectState::NO_OBJECT);
}

TEST(TaskExecutorPickPlaceTest, PlaceObjectReferenceComposesConfiguredToolOrientation)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto geometry = valid_geometry();
  geometry.object_to_grasp_tcp.translation.z = -0.0025;
  geometry.object_to_grasp_tcp.rotation.x = 1.0;
  geometry.object_to_grasp_tcp.rotation.w = 0.0;
  geometry.pick_approach_distance_m = 0.15;
  geometry.pick_retract_distance_m = 0.4;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend), geometry);
  executor.set_object_state(MotionObjectState::ATTACHED);

  auto goal = place_goal();
  goal.target_pose.pose.position.x = 0.35;
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.position.z = 0.80;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;
  goal.place_approach_direction_object.z = 1.0;
  goal.place_approach_distance_m = 0.05;
  goal.place_retract_distance_m = 0.05;

  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_EQ(backend->tcp_z_values.size(), 3U);
  EXPECT_DOUBLE_EQ(backend->tcp_z_values[0], 0.8475);
  EXPECT_DOUBLE_EQ(backend->requests.front().approach_distance_m, 0.05);
  EXPECT_DOUBLE_EQ(backend->tcp_z_values[1], 0.7975);
  EXPECT_DOUBLE_EQ(backend->tcp_z_values[2], 0.8475);
  ASSERT_EQ(backend->tcp_orientations.size(), 3U);
  for (const auto & orientation : backend->tcp_orientations) {
    EXPECT_DOUBLE_EQ(orientation.x, 1.0);
    EXPECT_DOUBLE_EQ(orientation.y, 0.0);
    EXPECT_DOUBLE_EQ(orientation.z, 0.0);
    EXPECT_DOUBLE_EQ(orientation.w, 0.0);
  }
  EXPECT_DOUBLE_EQ(goal.target_pose.pose.orientation.w, 1.0);
}

TEST(TaskExecutorPickPlaceTest, SuccessfulConfirmationPermitsAttachThenRetract)
{
  auto backend = std::make_shared<RecordingBackend>();
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(
    backend->events,
    (std::vector<std::string>{
      "gripper", "motion:3", "motion:4", "gripper", "motion:8"}));
  EXPECT_EQ(executor.object_state(), MotionObjectState::ATTACHED);
}

TEST(TaskExecutorPickPlaceTest, DetachFailureCannotReportSuccessOrMarkReleased)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->gripper_result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());
  executor.set_object_state(MotionObjectState::ATTACHED);

  EXPECT_EQ(executor.execute(place_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(executor.object_state(), MotionObjectState::ATTACHED);
  EXPECT_EQ(backend->events.back(), "gripper");
}

TEST(TaskExecutorPickPlaceTest, InvalidGeometryFailsClosedBeforePlanning)
{
  auto backend = std::make_shared<RecordingBackend>();
  TaskExecutor executor(backend);

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_INVALID_GOAL);
  EXPECT_TRUE(backend->events.empty());
}

namespace
{
class MandatoryOnlyBackend final : public MotionBackend
{
public:
  bool available() const override {return true;}
  bool submit(const ExecuteTask::Goal &) override {return true;}
  uint8_t execute_normalized(const NormalizedMotionRequest &) override
  {
    return MotionTaskResultCode::TASK_RESULT_SUCCESS;
  }
  void cancel() override {}
  void hold() override {}
  void stop() override {}
  bool execution_active() const override {return false;}
  bool backend_inactivity_confirmed() const override {return true;}
};

TEST(TaskExecutorPickPlaceTest, PickRejectsCommandSuccessUntilNewHeldObservationArrives)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto port = std::make_shared<RecordingGripperPort>(*backend);
  port->auto_observe = false;
  port->set_observation(HoldingState::RELEASED, 7);
  TaskExecutor executor(backend, port, valid_geometry());

  auto result = std::async(
    std::launch::async, [&executor]() {return executor.execute(pick_goal());});

  while (!backend->gripper_commanded.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(
    result.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
  port->publish(HoldingState::HELD);

  EXPECT_EQ(result.get(), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(executor.object_state(), MotionObjectState::ATTACHED);
}

TEST(TaskExecutorPickPlaceTest, PickRejectsNewUnknownObservation)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto port = std::make_shared<RecordingGripperPort>(*backend);
  port->auto_observe = false;
  port->set_observation(HoldingState::RELEASED, 7);
  TaskExecutor executor(backend, port, valid_geometry());

  auto result = std::async(
    std::launch::async, [&executor]() {return executor.execute(pick_goal());});

  while (!backend->gripper_commanded.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  port->publish(HoldingState::UNKNOWN);

  EXPECT_EQ(result.get(), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
}

TEST(TaskExecutorPickPlaceTest, LateHeldAfterConfirmationTimeoutCannotSucceed)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto port = std::make_shared<RecordingGripperPort>(*backend);
  port->auto_observe = false;
  port->set_observation(HoldingState::RELEASED, 7);
  TaskExecutor executor(
    backend, port, valid_geometry(), {}, std::chrono::milliseconds(10));

  auto result = std::async(
    std::launch::async, [&executor]() {return executor.execute(pick_goal());});

  while (!backend->gripper_commanded.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_EQ(result.wait_for(std::chrono::milliseconds(200)), std::future_status::ready);
  EXPECT_EQ(result.get(), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);

  port->publish(HoldingState::HELD);
  EXPECT_EQ(executor.object_state(), MotionObjectState::NO_OBJECT);
  EXPECT_EQ(
    backend->events,
    (std::vector<std::string>{"gripper", "motion:3", "motion:4", "gripper"}));
}

TEST(TaskExecutorPickPlaceTest, PlaceRejectsCommandSuccessUntilNewReleasedObservationArrives)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto port = std::make_shared<RecordingGripperPort>(*backend);
  port->auto_observe = false;
  port->open_auto_observe = false;
  port->set_observation(HoldingState::HELD, 11);
  TaskExecutor executor(backend, port, valid_geometry());
  executor.set_object_state(MotionObjectState::ATTACHED);

  auto result = std::async(
    std::launch::async, [&executor]() {return executor.execute(place_goal());});

  while (!backend->gripper_open_commanded.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(
    result.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
  port->publish(HoldingState::RELEASED);

  EXPECT_EQ(result.get(), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(executor.object_state(), MotionObjectState::NO_OBJECT);
}


TEST(TaskExecutorPickPlaceTest, PickCancellationDuringHoldingWaitBlocksRetract)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto port = std::make_shared<RecordingGripperPort>(*backend);
  port->auto_observe = false;
  port->set_observation(HoldingState::RELEASED, 20);
  auto interruption = std::make_shared<std::atomic<uint8_t>>(
    MotionTaskResultCode::TASK_RESULT_SUCCESS);
  TaskExecutor executor(
    backend, port, valid_geometry(),
    [interruption]() {return interruption->load();});

  auto result = std::async(
    std::launch::async, [&executor]() {return executor.execute(pick_goal());});
  while (!backend->gripper_commanded.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  interruption->store(MotionTaskResultCode::TASK_RESULT_CANCELED);
  port->publish(HoldingState::HELD);

  EXPECT_EQ(result.get(), MotionTaskResultCode::TASK_RESULT_CANCELED);
  EXPECT_EQ(
    backend->events,
    (std::vector<std::string>{"gripper", "motion:3", "motion:4", "gripper"}));
}

TEST(TaskExecutorPickPlaceTest, PickSafetyPreemptionDuringHoldingWaitBlocksRetract)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto port = std::make_shared<RecordingGripperPort>(*backend);
  port->auto_observe = false;
  port->set_observation(HoldingState::RELEASED, 20);
  auto interruption = std::make_shared<std::atomic<uint8_t>>(
    MotionTaskResultCode::TASK_RESULT_SUCCESS);
  TaskExecutor executor(
    backend, port, valid_geometry(),
    [interruption]() {return interruption->load();});

  auto result = std::async(
    std::launch::async, [&executor]() {return executor.execute(pick_goal());});
  while (!backend->gripper_commanded.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  interruption->store(MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
  port->publish(HoldingState::HELD);

  EXPECT_EQ(result.get(), MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
  EXPECT_EQ(
    backend->events,
    (std::vector<std::string>{"gripper", "motion:3", "motion:4", "gripper"}));
}

TEST(TaskExecutorPickPlaceTest, PlaceCancellationDuringHoldingWaitBlocksRetreat)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto port = std::make_shared<RecordingGripperPort>(*backend);
  port->auto_observe = false;
  port->open_auto_observe = false;
  port->set_observation(HoldingState::HELD, 20);
  auto interruption = std::make_shared<std::atomic<uint8_t>>(
    MotionTaskResultCode::TASK_RESULT_SUCCESS);
  TaskExecutor executor(
    backend, port, valid_geometry(),
    [interruption]() {return interruption->load();});
  executor.set_object_state(MotionObjectState::ATTACHED);

  auto result = std::async(
    std::launch::async, [&executor]() {return executor.execute(place_goal());});
  while (!backend->gripper_open_commanded.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  interruption->store(MotionTaskResultCode::TASK_RESULT_CANCELED);
  port->publish(HoldingState::RELEASED);

  EXPECT_EQ(result.get(), MotionTaskResultCode::TASK_RESULT_CANCELED);
  EXPECT_EQ(
    backend->events,
    (std::vector<std::string>{"motion:3", "motion:4", "gripper"}));
}

TEST(TaskExecutorPickPlaceTest, PlaceSafetyPreemptionDuringHoldingWaitBlocksRetreat)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto port = std::make_shared<RecordingGripperPort>(*backend);
  port->auto_observe = false;
  port->open_auto_observe = false;
  port->set_observation(HoldingState::HELD, 20);
  auto interruption = std::make_shared<std::atomic<uint8_t>>(
    MotionTaskResultCode::TASK_RESULT_SUCCESS);
  TaskExecutor executor(
    backend, port, valid_geometry(),
    [interruption]() {return interruption->load();});
  executor.set_object_state(MotionObjectState::ATTACHED);

  auto result = std::async(
    std::launch::async, [&executor]() {return executor.execute(place_goal());});
  while (!backend->gripper_open_commanded.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  interruption->store(MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
  port->publish(HoldingState::RELEASED);

  EXPECT_EQ(result.get(), MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
  EXPECT_EQ(
    backend->events,
    (std::vector<std::string>{"motion:3", "motion:4", "gripper"}));
}

TEST(TaskExecutorPickPlaceTest, InterruptedPickDoesNotPoisonNextIndependentPick)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto port = std::make_shared<RecordingGripperPort>(*backend);
  port->auto_observe = false;
  port->set_observation(HoldingState::RELEASED, 20);
  auto interruption = std::make_shared<std::atomic<uint8_t>>(
    MotionTaskResultCode::TASK_RESULT_SUCCESS);
  TaskExecutor executor(
    backend, port, valid_geometry(),
    [interruption]() {return interruption->load();});

  auto interrupted = std::async(
    std::launch::async, [&executor]() {return executor.execute(pick_goal());});
  while (!backend->gripper_commanded.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  interruption->store(MotionTaskResultCode::TASK_RESULT_CANCELED);
  port->publish(HoldingState::HELD);
  EXPECT_EQ(interrupted.get(), MotionTaskResultCode::TASK_RESULT_CANCELED);

  interruption->store(MotionTaskResultCode::TASK_RESULT_SUCCESS);
  backend->gripper_commanded.store(false);
  port->publish(HoldingState::RELEASED);
  auto next = std::async(
    std::launch::async, [&executor]() {return executor.execute(pick_goal());});
  while (!backend->gripper_commanded.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  port->publish(HoldingState::HELD);

  EXPECT_EQ(next.get(), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(executor.object_state(), MotionObjectState::ATTACHED);
}

}  // namespace

TEST(TaskExecutorPickPlaceTest, UnsupportedOptionalCapabilitiesFailClosedForPickAndPlace)
{
  auto backend = std::make_shared<MandatoryOnlyBackend>();
  TaskExecutor executor(backend, valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(executor.object_state(), MotionObjectState::NO_OBJECT);

  executor.set_object_state(MotionObjectState::ATTACHED);
  EXPECT_EQ(executor.execute(place_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(executor.object_state(), MotionObjectState::ATTACHED);
}

}  // namespace arm_cell_motion_moveit2
