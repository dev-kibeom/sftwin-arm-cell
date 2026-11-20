#include "arm_cell_motion/motion_node.hpp"
#include "arm_cell_motion/isaac_ros_motion_transport.hpp"
#include "arm_cell_motion/motion_result_code_name.hpp"
#include "arm_cell_motion/taught_motion.hpp"

#include <nlohmann/json.hpp>
#include <iomanip>
#include <sstream>

namespace arm_cell_motion
{
namespace
{
std::string uuid_text(const UUID & uuid)
{
  std::ostringstream text;
  for (auto byte : uuid.uuid) {
    text << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(byte);
  }
  return text.str();
}

bool valid_manual_motion(const std::string & json, std::size_t joint_count)
{
  try {
    const auto motions = parse_taught_motions("{\"manual\":" + json + "}", {}, {}, joint_count);
    return !motions.at("manual").cartesian_waypoints.empty();
  } catch (const std::exception &) {return false;}
}
}  // namespace

MotionNode::~MotionNode()
{
  if (teaching_worker_.joinable()) {
    try {core_->request_cancel(teaching_goal_uuid_);} catch (...) {
      RCLCPP_WARN(get_logger(), "Teaching cancellation failed during node shutdown");
    }
    teaching_worker_.join();
  }
}

void MotionNode::initialize_teaching()
{
  if (isaac_transport_) {
    const std::weak_ptr<MotionCore> core = core_;
    isaac_transport_->set_teaching_submission_check([core]() {
        const auto locked = core.lock();
        return locked ? locked->teachingSubmissionPermission() :
               MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
      });
  }
  teaching_service_ = create_service<arm_cell_interfaces::srv::ConfigureTeaching>(
    "/motion/configure_teaching", [this](
      const std::shared_ptr<arm_cell_interfaces::srv::ConfigureTeaching::Request> request,
      std::shared_ptr<arm_cell_interfaces::srv::ConfigureTeaching::Response> response) {
      if (!isaac_transport_) {
        response->diagnostic_detail = "teaching requires the configured MoveIt backend";
        return;
      }
      response->success = core_->configureTeaching([&]() {
          return isaac_transport_->replace_teaching(
            request->taught_motions_json, response->diagnostic_detail);
        });
      if (!response->success && response->diagnostic_detail.empty()) {
        response->diagnostic_detail = "Motion is busy or backend inactivity is unconfirmed";
      }
    });
  teaching_action_ = rclcpp_action::create_server<ExecuteTeaching>(
    this, "/motion/execute_teaching",
    [this](const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const ExecuteTeaching::Goal> goal) {
      UUID id;
      id.uuid = uuid;
      if (!isaac_transport_ || !valid_manual_motion(goal->motion_json,
          get_parameter("arm_joint_names").as_string_array().size()) ||
        !core_->reserveTeaching(id)) {return rclcpp_action::GoalResponse::REJECT;}
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteTeaching>> handle) {
      UUID id;
      id.uuid = handle->get_goal_id();
      return core_->request_cancel(id) ? rclcpp_action::CancelResponse::ACCEPT :
             rclcpp_action::CancelResponse::REJECT;
    },
    [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteTeaching>> handle) {
      teaching_goal_uuid_.uuid = handle->get_goal_id();
      const auto core = core_;
      const auto id = teaching_goal_uuid_;
      const auto logger = get_logger();
      const auto release = [core, id]() noexcept {
          try {core->releaseTeachingReservation(id);} catch (...) {}
        };
      try {
        if (teaching_worker_.joinable()) {teaching_worker_.join();}
        teaching_worker_ = std::thread([core, id, handle, logger, release]() noexcept {
            struct ReservationGuard
            {
              decltype(release) finish;
              ~ReservationGuard() {finish();}
            } reservation{release};
            try {
              auto result = std::make_shared<ExecuteTeaching::Result>();
              result->result_code = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
              try {
                auto feedback = std::make_shared<ExecuteTeaching::Feedback>();
                feedback->phase = "planning_and_executing";
                handle->publish_feedback(feedback);
                result->result_code = core->executeTaughtMotion(
                  nlohmann::json::parse(handle->get_goal()->motion_json).dump(),
                  uuid_text(id), "teaching");
              } catch (const std::exception & error) {
                result->diagnostic_detail = error.what();
              } catch (...) {
                result->diagnostic_detail = "unknown teaching execution exception";
              }
              release();
              if (result->diagnostic_detail.empty()) {
                result->diagnostic_detail = motion_result_code_name(result->result_code);
              }
              if (result->result_code == MotionTaskResultCode::TASK_RESULT_SUCCESS) {
                handle->succeed(result);
              } else if (handle->is_canceling()) {
                handle->canceled(result);
              } else {handle->abort(result);}
            } catch (...) {
              // Shutdown may invalidate feedback/result publication. Never let a
              // ROS handle exception escape the owning thread or strand admission.
              RCLCPP_WARN(logger, "Teaching action publication failed; worker is ending");
            }
          });
      } catch (...) {
        release();
        try {
          auto result = std::make_shared<ExecuteTeaching::Result>();
          result->result_code = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
          result->diagnostic_detail = "teaching worker could not start";
          handle->abort(result);
        } catch (...) {
          RCLCPP_WARN(logger, "Teaching worker startup/result publication failed");
        }
      }
    });
}
}  // namespace arm_cell_motion
