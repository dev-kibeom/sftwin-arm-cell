#include "arm_cell_motion/isaac_ros_motion_transport.hpp"

namespace arm_cell_motion
{
void IsaacRosMotionTransport::configure_taught_motions(
  TaughtMotions motions, std::vector<JointMotionLimit> limits,
  std::vector<std::string> cycle)
{
  {
    std::lock_guard<std::mutex> lock(teaching_mutex_);
    taught_motions_ = std::move(motions);
    taught_joint_limits_ = std::move(limits);
    taught_cycle_motion_names_ = std::move(cycle);
    teaching_startup_pending_ = !taught_cycle_motion_names_.empty();
    prepared_teaching_.reset();
    ++teaching_generation_;
  }
  teaching_prepare_condition_.notify_one();
}

moveit::core::RobotModelConstPtr IsaacRosMotionTransport::teaching_model_locked()
{
  auto & state = *move_group_state_;
  if (!state.move_group) {return nullptr;}
  if (!state.teaching_model) {
    const auto source = state.move_group->getRobotModel();
    const auto * group = source->getJointModelGroup(planning_group_);
    if (!group) {return nullptr;}
    auto solver = state.pick_ik_solver;
    if (!solver) {
      solver = std::const_pointer_cast<kinematics::KinematicsBase>(group->getSolverInstance());
    }
    state.teaching_model = make_kinematic_model(source, planning_group_, solver);
  }
  return state.teaching_model;
}

void IsaacRosMotionTransport::start_teaching_preparation()
{
  std::lock_guard<std::mutex> lock(teaching_mutex_);
  if (teaching_shutdown_ || teaching_prepare_worker_.joinable() ||
    taught_cycle_motion_names_.empty())
  {return;}
  try {
    teaching_prepare_worker_ = std::thread(
      [this] {
        try {
          run_teaching_preparation();
        } catch (const std::exception & error) {
          teaching_startup_pending_ = false;
          RCLCPP_WARN(node_->get_logger(), "Teaching preparation worker ended: %s", error.what());
        }
      });
  } catch (const std::exception & error) {
    teaching_startup_pending_ = false;
    RCLCPP_WARN(node_->get_logger(), "Teaching preparation could not start: %s", error.what());
  }
}

void IsaacRosMotionTransport::run_teaching_preparation()
{
  using Code = arm_cell_interfaces::msg::MotionTaskResultCode;
  std::uint64_t revision = 0;
  while (true) {
    TaughtMotions motions;
    std::vector<std::string> cycle;
    std::vector<JointMotionLimit> limits;
    {
      std::unique_lock<std::mutex> lock(teaching_mutex_);
      teaching_prepare_condition_.wait(
        lock, [&] {
          return teaching_shutdown_ || teaching_generation_ != revision;
        });
      if (teaching_shutdown_) {return;}
      revision = teaching_generation_;
      motions = taught_motions_;
      cycle = taught_cycle_motion_names_;
      limits = taught_joint_limits_;
    }

    const auto interrupted = [&]() -> std::uint8_t {
        std::lock_guard<std::mutex> lock(teaching_mutex_);
        return teaching_shutdown_ || teaching_generation_ != revision ?
               Code::TASK_RESULT_CANCELED : Code::TASK_RESULT_SUCCESS;
      };
    PreparedTeaching entries;
    const auto started = std::chrono::steady_clock::now();
    try {
      moveit::core::RobotModelConstPtr model;
      {
        std::lock_guard<std::mutex> lock(move_group_state_->mutex);
        model = teaching_model_locked();
      }
      const auto * group = model ? model->getJointModelGroup(planning_group_) : nullptr;
      const auto home_motion = motions.find("home");
      if (group && home_motion != motions.end() &&
        group->getVariableNames() == arm_joint_names_ &&
        home_motion->second.joint_positions.size() == arm_joint_names_.size())
      {
        moveit::core::RobotState home(model);
        home.setToDefaultValues();
        home.setJointGroupPositions(group, home_motion->second.joint_positions);
        home.update();
        entries = prepare_teaching_cycle(
          motions, cycle, home, planning_group_, limits, move_group_state_->mutex, interrupted);
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN(node_->get_logger(), "Teaching preparation failed: %s", error.what());
    }

    std::lock_guard<std::mutex> lock(teaching_mutex_);
    if (teaching_shutdown_) {return;}
    if (revision != teaching_generation_) {continue;}
    std::size_t count = 0;
    for (const auto & entry : entries) {
      count += entry.second.size();
    }
    if (count != 0) {
      prepared_teaching_ = std::make_shared<const PreparedTeaching>(std::move(entries));
    }
    teaching_startup_pending_ = false;
    RCLCPP_INFO(
      node_->get_logger(), "Prepared %zu taught trajectories in %.3f s", count,
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count());
  }
}
}  // namespace arm_cell_motion
