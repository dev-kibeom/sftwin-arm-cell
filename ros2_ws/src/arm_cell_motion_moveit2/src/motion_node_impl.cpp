#include "arm_cell_motion_moveit2/motion_node.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <arm_cell_interfaces/msg/stop_mode.hpp>
#include <arm_cell_interfaces/srv/stop_motion.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include "arm_cell_motion_moveit2/isaac_motion_backend.hpp"
#include "arm_cell_motion_moveit2/isaac_ros_motion_transport.hpp"
#include "arm_cell_motion_moveit2/task_executor.hpp"

namespace arm_cell_motion_moveit2
{

namespace
{
MotionGeometry read_motion_geometry(rclcpp::Node & node)
{
  const auto observation_translation = node.declare_parameter<std::vector<double>>(
    "observation_to_object_translation", std::vector<double>{});
  const auto observation_rotation = node.declare_parameter<std::vector<double>>(
    "observation_to_object_quaternion", std::vector<double>{});
  const auto observation_reference = node.declare_parameter<std::string>(
    "observation_reference", "");
  const auto translation = node.declare_parameter<std::vector<double>>(
    "object_to_grasp_tcp_translation", std::vector<double>{});
  const auto rotation = node.declare_parameter<std::vector<double>>(
    "object_to_grasp_tcp_quaternion", std::vector<double>{});
  const auto insertion_axis = node.declare_parameter<std::vector<double>>(
    "insertion_axis_tcp", std::vector<double>{});
  const auto roll_tolerance = node.declare_parameter<double>("target_roll_tolerance_rad", 0.0);
  const auto pitch_tolerance = node.declare_parameter<double>("target_pitch_tolerance_rad", 0.0);
  const auto roll_step = node.declare_parameter<double>("target_roll_sample_step_rad", 0.0);
  const auto pitch_step = node.declare_parameter<double>("target_pitch_sample_step_rad", 0.0);
  const auto pick_approach_distance = node.declare_parameter<double>(
    "pick_approach_distance_m", -1.0);
  const auto pick_retract_distance = node.declare_parameter<double>(
    "pick_retract_distance_m", -1.0);

  MotionGeometry geometry;
  geometry.observation_reference = observation_reference;
  geometry.target_roll_tolerance_rad = roll_tolerance;
  geometry.target_pitch_tolerance_rad = pitch_tolerance;
  geometry.target_roll_sample_step_rad = roll_step;
  geometry.target_pitch_sample_step_rad = pitch_step;
  geometry.pick_approach_distance_m = pick_approach_distance;
  geometry.pick_retract_distance_m = pick_retract_distance;
  if (observation_translation.size() == 3U && observation_rotation.size() == 4U &&
    translation.size() == 3U && rotation.size() == 4U && insertion_axis.size() == 3U)
  {
    geometry.observation_to_object.translation.x = observation_translation[0];
    geometry.observation_to_object.translation.y = observation_translation[1];
    geometry.observation_to_object.translation.z = observation_translation[2];
    geometry.observation_to_object.rotation.x = observation_rotation[0];
    geometry.observation_to_object.rotation.y = observation_rotation[1];
    geometry.observation_to_object.rotation.z = observation_rotation[2];
    geometry.observation_to_object.rotation.w = observation_rotation[3];
    geometry.object_to_grasp_tcp.translation.x = translation[0];
    geometry.object_to_grasp_tcp.translation.y = translation[1];
    geometry.object_to_grasp_tcp.translation.z = translation[2];
    geometry.object_to_grasp_tcp.rotation.x = rotation[0];
    geometry.object_to_grasp_tcp.rotation.y = rotation[1];
    geometry.object_to_grasp_tcp.rotation.z = rotation[2];
    geometry.object_to_grasp_tcp.rotation.w = rotation[3];
    geometry.insertion_axis_tcp.x = insertion_axis[0];
    geometry.insertion_axis_tcp.y = insertion_axis[1];
    geometry.insertion_axis_tcp.z = insertion_axis[2];
    geometry.configured = true;
  }
  return geometry;
}

class UnavailableMotionBackend final : public MotionBackend
{
public:
  using StopCallback = std::function<void (uint8_t)>;

  UnavailableMotionBackend(
    bool available, std::chrono::milliseconds availability_delay, bool controllable,
    StopCallback stop_callback)
  : available_(available), availability_delay_(availability_delay),
    controllable_(controllable), stop_callback_(std::move(stop_callback)) {}

  bool available() const override
  {
    std::this_thread::sleep_for(availability_delay_);
    return available_.load();
  }

  bool set_motion_envelope(const MotionEnvelope & envelope) override
  {
    if (!envelope.valid || !std::isfinite(envelope.max_velocity_scale) ||
      !std::isfinite(envelope.max_acceleration_scale) ||
      envelope.max_velocity_scale <= 0.0F || envelope.max_velocity_scale > 1.0F ||
      envelope.max_acceleration_scale <= 0.0F || envelope.max_acceleration_scale > 1.0F)
    {
      return false;
    }
    applied_velocity_scale_.store(envelope.max_velocity_scale);
    applied_acceleration_scale_.store(envelope.max_acceleration_scale);
    applied_envelope_valid_.store(true);
    return true;
  }

  MotionEnvelope applied_motion_envelope() const override
  {
    return MotionEnvelope{
      applied_velocity_scale_.load(), applied_acceleration_scale_.load(),
      applied_envelope_valid_.load()};
  }

  bool submit(const ExecuteTask::Goal &) override
  {
    if (!controllable_ || !available()) {
      return false;
    }
    execution_active_.store(true);
    inactivity_confirmed_.store(false);
    return true;
  }

  void cancel() override
  {
    if (stop_callback_) {
      stop_callback_(arm_cell_interfaces::msg::StopMode::STOP_MODE_IMMEDIATE);
    }
  }

  void hold() override
  {
    if (stop_callback_) {
      stop_callback_(arm_cell_interfaces::msg::StopMode::STOP_MODE_CONTROLLED);
    }
  }

  void stop() override
  {
    if (stop_callback_) {
      stop_callback_(arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY);
    }
  }

  bool execution_active() const override {return execution_active_.load();}
  bool backend_inactivity_confirmed() const override {return inactivity_confirmed_.load();}

  void set_execution_active(bool value) {execution_active_.store(value);}
  void set_inactivity_confirmed(bool value) {inactivity_confirmed_.store(value);}

private:
  std::atomic_bool available_;
  std::chrono::milliseconds availability_delay_;
  bool controllable_;
  StopCallback stop_callback_;
  std::atomic_bool execution_active_{false};
  std::atomic_bool inactivity_confirmed_{true};
  std::atomic<float> applied_velocity_scale_{1.0F};
  std::atomic<float> applied_acceleration_scale_{1.0F};
  std::atomic_bool applied_envelope_valid_{true};
};
}  // namespace

MotionNode::MotionNode(const rclcpp::NodeOptions & options)
: MotionNode(nullptr, options) {}

MotionNode::MotionNode(
  std::shared_ptr<MotionBackend> backend, const rclcpp::NodeOptions & options)
: Node("motion_node", options)
{
  const auto backend_mode = declare_parameter<std::string>("backend_mode", "unavailable");
  const auto robot_description = declare_parameter<std::string>("robot_description", "");
  const auto robot_description_semantic = declare_parameter<std::string>(
    "robot_description_semantic", "");
  const auto kinematics_solver = declare_parameter<std::string>(
    "robot_description_kinematics.arm.kinematics_solver", "");
  const auto kinematics_search_resolution = declare_parameter<double>(
    "robot_description_kinematics.arm.kinematics_solver_search_resolution", 0.0);
  const auto kinematics_timeout = declare_parameter<double>(
    "robot_description_kinematics.arm.kinematics_solver_timeout", 0.0);
  const auto availability_delay_ms = declare_parameter<int64_t>(
    "backend_availability_delay_ms", 0);
  const auto backend_available = declare_parameter<bool>("backend_available", false);
  const auto enable_test_backend_controls = declare_parameter<bool>(
    "enable_test_backend_controls", false);
  const auto holding_confirmation_timeout_ms = declare_parameter<int64_t>(
    "holding_confirmation_timeout_ms", 500);
  const auto planning_velocity_scaling_factor = declare_parameter<double>(
    "planning_velocity_scaling_factor", 0.1);
  const auto planning_acceleration_scaling_factor = declare_parameter<double>(
    "planning_acceleration_scaling_factor", 0.1);
  const auto planning_time_s = declare_parameter<double>("planning_time_s", 5.0);
  const auto place_candidate_planning_time_s = declare_parameter<double>(
    "place_candidate_planning_time_s", 1.0);
  const auto place_total_planning_time_s = declare_parameter<double>(
    "place_total_planning_time_s", 5.0);
  const auto approach_entry_position_tolerance_m = declare_parameter<double>(
    "approach_entry_position_tolerance_m", 0.005);
  const auto approach_entry_orientation_tolerance_rad = declare_parameter<double>(
    "approach_entry_orientation_tolerance_rad", 0.08726646259971647);
  const auto motion_progress_stall_timeout_s = declare_parameter<double>(
    "motion_progress_stall_timeout_s", 60.0);
  const auto place_tracking_tolerance_rad = declare_parameter<double>(
    "place_tracking_tolerance_rad", 0.01);
  const auto place_approach_entry_position_tolerance_m = declare_parameter<double>(
    "place_approach_entry_position_tolerance_m", approach_entry_position_tolerance_m);
  if (holding_confirmation_timeout_ms <= 0) {
    throw std::invalid_argument("holding_confirmation_timeout_ms must be positive");
  }
  const auto motion_geometry = read_motion_geometry(*this);
  runtime_geometry_ = motion_geometry;
  holding_confirmation_timeout_ = std::chrono::milliseconds(holding_confirmation_timeout_ms);
  runtime_tuning_.planning_time_s = planning_time_s;
  runtime_tuning_.place_candidate_planning_time_s = place_candidate_planning_time_s;
  runtime_tuning_.place_total_planning_time_s = place_total_planning_time_s;
  runtime_tuning_.planning_velocity_scaling_factor = planning_velocity_scaling_factor;
  runtime_tuning_.planning_acceleration_scaling_factor = planning_acceleration_scaling_factor;
  runtime_tuning_.motion_progress_stall_timeout_s = motion_progress_stall_timeout_s;
  runtime_tuning_.approach_entry_position_tolerance_m = approach_entry_position_tolerance_m;
  runtime_tuning_.approach_entry_orientation_tolerance_rad =
    approach_entry_orientation_tolerance_rad;
  runtime_tuning_.place_approach_entry_position_tolerance_m =
    place_approach_entry_position_tolerance_m;
  runtime_tuning_.place_tracking_tolerance_rad = place_tracking_tolerance_rad;

  rclcpp::Publisher<arm_cell_interfaces::msg::StopMode>::SharedPtr stop_mode_publisher;
  if (enable_test_backend_controls) {
    stop_mode_publisher = create_publisher<arm_cell_interfaces::msg::StopMode>(
      "/motion/test_backend/stop_mode", rclcpp::QoS(10));
  }
  std::shared_ptr<UnavailableMotionBackend> test_backend;
  if (!backend) {
    if (backend_mode == "isaac") {
      if (enable_test_backend_controls) {
        throw std::invalid_argument("test backend controls are not valid for the Isaac backend");
      }
      const auto arm_joint_names = declare_parameter<std::vector<std::string>>(
        "arm_joint_names", {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});
      const auto home_joint_positions = declare_parameter<std::vector<double>>(
        "home_joint_positions", std::vector<double>{});
      const auto retract_joint_positions = declare_parameter<std::vector<double>>(
        "retract_joint_positions", std::vector<double>{});
      const auto validation_grasp_contact_policy_enabled = declare_parameter<bool>(
        "validation_grasp_contact_policy_enabled", false);
      const auto validation_grasp_contact_object_id_prefix = declare_parameter<std::string>(
        "validation_grasp_contact_object_id_prefix", "");
      const auto validation_grasp_contact_links = declare_parameter<std::vector<std::string>>(
        "validation_grasp_contact_links", std::vector<std::string>{});
      const auto planning_max_acceleration_rad_s2 = declare_parameter<double>(
        "planning_max_acceleration_rad_s2", 0.0);
      if (!std::isfinite(place_candidate_planning_time_s) ||
        place_candidate_planning_time_s <= 0.0 ||
        !std::isfinite(place_total_planning_time_s) || place_total_planning_time_s <= 0.0)
      {
        throw std::invalid_argument("PLACE planning budgets must be finite and positive");
      }
      const auto trajectory_sampler = declare_parameter<std::string>(
        "trajectory_sampler", "linear");
      const auto trajectory_clock_liveness_timeout_s = declare_parameter<double>(
        "trajectory_clock_liveness_timeout_s", 2.0);
      const auto approach_entry_require_settled = declare_parameter<bool>(
        "approach_entry_require_settled", true);
      const auto approach_entry_settled_joint_delta_rad = declare_parameter<double>(
        "approach_entry_settled_joint_delta_rad", 0.001);
      const auto approach_entry_settled_samples = declare_parameter<int64_t>(
        "approach_entry_settled_samples", 3);
      rclcpp::NodeOptions backend_options;
      std::vector<rclcpp::Parameter> backend_parameters;
      if (has_parameter("use_sim_time")) {
        backend_parameters.emplace_back(
          "use_sim_time", get_parameter("use_sim_time").as_bool());
      }
      if (!robot_description.empty()) {
        backend_parameters.emplace_back("robot_description", robot_description);
      }
      if (!robot_description_semantic.empty()) {
        backend_parameters.emplace_back("robot_description_semantic", robot_description_semantic);
      }
      if (!kinematics_solver.empty()) {
        backend_parameters.emplace_back(
          "robot_description_kinematics.arm.kinematics_solver", kinematics_solver);
        backend_parameters.emplace_back(
          "robot_description_kinematics.arm.kinematics_solver_search_resolution",
          kinematics_search_resolution);
        backend_parameters.emplace_back(
          "robot_description_kinematics.arm.kinematics_solver_timeout", kinematics_timeout);
      }
      if (!backend_parameters.empty()) {
        backend_options.parameter_overrides(backend_parameters);
      }
      backend_options.use_global_arguments(false);
      backend_node_ = std::make_shared<rclcpp::Node>("isaac_motion_backend", backend_options);
      isaac_transport_ = std::make_shared<IsaacRosMotionTransport>(
        backend_node_, arm_joint_names, home_joint_positions, retract_joint_positions,
        false, validation_grasp_contact_policy_enabled,
        validation_grasp_contact_object_id_prefix, validation_grasp_contact_links,
        planning_max_acceleration_rad_s2, trajectory_sampler,
        trajectory_clock_liveness_timeout_s,
        approach_entry_position_tolerance_m, approach_entry_orientation_tolerance_rad,
        approach_entry_require_settled, approach_entry_settled_joint_delta_rad,
        static_cast<std::size_t>(std::max<int64_t>(1, approach_entry_settled_samples)),
        motion_progress_stall_timeout_s, place_tracking_tolerance_rad,
        place_approach_entry_position_tolerance_m, planning_velocity_scaling_factor,
        planning_acceleration_scaling_factor, planning_time_s,
        place_candidate_planning_time_s, place_total_planning_time_s);
      backend = std::make_shared<IsaacMotionBackend>(isaac_transport_);
    } else if (backend_mode == "unavailable") {
      test_backend = std::make_shared<UnavailableMotionBackend>(
        backend_available, std::chrono::milliseconds(availability_delay_ms),
        enable_test_backend_controls,
        [stop_mode_publisher](uint8_t mode) {
          if (stop_mode_publisher) {
            arm_cell_interfaces::msg::StopMode message;
            message.value = mode;
            stop_mode_publisher->publish(message);
          }
        });
      backend = test_backend;
    } else {
      throw std::invalid_argument("unsupported Motion backend_mode: " + backend_mode);
    }
  }

  core_ = std::make_shared<MotionCore>(
    backend, std::chrono::milliseconds(500), motion_geometry,
    std::chrono::milliseconds(holding_confirmation_timeout_ms));
  if (!core_->set_runtime_tuning(
      runtime_tuning_, runtime_geometry_, holding_confirmation_timeout_))
  {
    throw std::runtime_error("failed to initialize Motion tuning snapshot");
  }
  tuning_callback_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      std::lock_guard<std::mutex> parameter_lock(tuning_parameters_mutex_);
      rcl_interfaces::msg::SetParametersResult result;
      auto tuning = runtime_tuning_;
      auto geometry = runtime_geometry_;
      auto holding_timeout = holding_confirmation_timeout_;
      bool idle_mutable_update = false;
      auto set_double = [&](const rclcpp::Parameter & parameter, double & destination) {
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
            result.reason = parameter.get_name() + " must be a double";
            return false;
          }
          destination = parameter.as_double();
          return true;
        };
      for (const auto & parameter : parameters) {
        const auto & name = parameter.get_name();
        double * target = nullptr;
        if (name == "planning_time_s") {target = &tuning.planning_time_s;}
        else if (name == "place_candidate_planning_time_s") {
          target = &tuning.place_candidate_planning_time_s;
        } else if (name == "place_total_planning_time_s") {
          target = &tuning.place_total_planning_time_s;
        } else if (name == "planning_velocity_scaling_factor") {
          target = &tuning.planning_velocity_scaling_factor;
        } else if (name == "planning_acceleration_scaling_factor") {
          target = &tuning.planning_acceleration_scaling_factor;
        } else if (name == "motion_progress_stall_timeout_s") {
          target = &tuning.motion_progress_stall_timeout_s;
          idle_mutable_update = true;
        } else if (name == "approach_entry_position_tolerance_m") {
          target = &tuning.approach_entry_position_tolerance_m;
          idle_mutable_update = true;
        } else if (name == "approach_entry_orientation_tolerance_rad") {
          target = &tuning.approach_entry_orientation_tolerance_rad;
          idle_mutable_update = true;
        } else if (name == "place_approach_entry_position_tolerance_m") {
          target = &tuning.place_approach_entry_position_tolerance_m;
          idle_mutable_update = true;
        } else if (name == "place_tracking_tolerance_rad") {
          target = &tuning.place_tracking_tolerance_rad;
          idle_mutable_update = true;
        } else if (name == "pick_approach_distance_m") {
          target = &geometry.pick_approach_distance_m;
          idle_mutable_update = true;
        } else if (name == "holding_confirmation_timeout_ms") {
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
            result.reason = name + " must be an integer";
            return result;
          }
          const auto value = parameter.as_int();
          if (value < 1 || value > 60000) {
            result.reason = name + " must be in [1, 60000] ms";
            return result;
          }
          holding_timeout = std::chrono::milliseconds(value);
          idle_mutable_update = true;
          continue;
        } else {
          result.reason = name + " is static/restart-required or not runtime tuning";
          return result;
        }
        if (!set_double(parameter, *target)) {
          return result;
        }
      }
      const auto in_range = [](double value, double minimum, double maximum) {
          return std::isfinite(value) && value >= minimum && value <= maximum;
        };
      if (!in_range(tuning.planning_time_s, 0.05, 300.0) ||
        !in_range(tuning.place_candidate_planning_time_s, 0.05, 60.0) ||
        !in_range(tuning.place_total_planning_time_s, 0.05, 300.0) ||
        tuning.place_total_planning_time_s < tuning.place_candidate_planning_time_s ||
        !in_range(tuning.planning_velocity_scaling_factor, 1.0e-4, 1.0) ||
        !in_range(tuning.planning_acceleration_scaling_factor, 1.0e-4, 1.0) ||
        !in_range(tuning.motion_progress_stall_timeout_s, 0.1, 600.0) ||
        !in_range(tuning.approach_entry_position_tolerance_m, 1.0e-5, 0.1) ||
        !in_range(tuning.approach_entry_orientation_tolerance_rad, 1.0e-5, 0.5) ||
        !in_range(tuning.place_approach_entry_position_tolerance_m, 1.0e-5, 0.1) ||
        !in_range(tuning.place_tracking_tolerance_rad, 1.0e-5, 1.0) ||
        !in_range(geometry.pick_approach_distance_m, 1.0e-4, 0.5) ||
        !in_range(geometry.pick_retract_distance_m, 1.0e-4, 0.5))
      {
        result.reason = "Motion tuning value is outside its finite supported range";
        return result;
      }
      if (!core_->set_runtime_tuning(
          tuning, geometry, holding_timeout, idle_mutable_update))
      {
        result.reason = "idle-mutable Motion tuning requires Motion to be idle";
        return result;
      }
      runtime_tuning_ = tuning;
      runtime_geometry_ = geometry;
      holding_confirmation_timeout_ = holding_timeout;
      result.successful = true;
      result.reason = "accepted for the next Motion action; the active action keeps its snapshot";
      return result;
    });
  action_server_ = std::make_shared<MotionActionServer>(this, core_);
  safety_state_subscription_ = create_subscription<SafetyState>(
    "/safety/state", rclcpp::QoS(10),
    [this](const SafetyState::SharedPtr message) {
      core_->update_safety_state(*message, MotionCore::Clock::now());
    });
  status_publisher_ = create_publisher<MotionStatus>("/motion/state", rclcpp::QoS(10));
  stop_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  stop_service_ = create_service<arm_cell_interfaces::srv::StopMotion>(
    "/safety/stop_motion",
    [this](
      const std::shared_ptr<arm_cell_interfaces::srv::StopMotion::Request> request,
      std::shared_ptr<arm_cell_interfaces::srv::StopMotion::Response> response) {
      response->accepted = core_->request_stop(request->request_id, request->stop_mode.value);
      response->diagnostic_detail = response->accepted ?
      "Stop request accepted for processing" : "No active Motion execution or invalid stop mode";
    }, rmw_qos_profile_services_default, stop_callback_group_);
  if (test_backend) {
    test_backend_active_service_ = create_service<std_srvs::srv::SetBool>(
      "/motion/test_backend/execution_active",
      [test_backend](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        test_backend->set_execution_active(request->data);
        response->success = true;
        response->message = "fake backend execution_active updated";
      });
    test_backend_inactivity_service_ = create_service<std_srvs::srv::SetBool>(
      "/motion/test_backend/inactivity_confirmed",
      [test_backend](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        test_backend->set_inactivity_confirmed(request->data);
        response->success = true;
        response->message = "fake backend inactivity confirmation updated";
      });
  }
  status_timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    [this]() {
      core_->refresh_stop_state();
      auto status = core_->status();
      status.header.stamp = now();
      status_publisher_->publish(status);
    });
}

void MotionNode::initialize_backend()
{
  if (isaac_transport_) {
    isaac_transport_->initialize_move_group();
  }
}

}  // namespace arm_cell_motion_moveit2
