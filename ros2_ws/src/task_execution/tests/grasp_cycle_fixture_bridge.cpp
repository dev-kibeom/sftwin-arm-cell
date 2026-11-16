// Test-only ABI for running the real Task BT against live Isaac fixture
// callbacks in the same process. No ROS endpoint or production Motion runner.
#include <cstdint>
#include <memory>
#include <string>
#include <nlohmann/json.hpp>
#include "arm_cell_task_execution/task_executor.hpp"

namespace
{
using namespace arm_cell_motion;
using namespace arm_cell_task_execution;
using Code = arm_cell_interfaces::msg::MotionTaskResultCode;
// Callback returns a MotionTaskResultCode except OBSERVE (0 unknown, 1 held,
// 2 released). Python catches exceptions before returning across this ABI.
enum Event { MOVE_TAUGHT, GRASP, RELEASE, OBSERVE, COMMIT, CLEAR, STOP };
using Callback = int (*)(int, const char *);
class FixtureMotion final : public MotionPort
{
public:
  explicit FixtureMotion(Callback callback) : callback_(callback) {}
  MotionPortResult planAndExecuteMotion(const MotionPortRequest &) override
  {return MotionPortResult();}
  MotionCancelResult cancelMotion(const unique_identifier_msgs::msg::UUID &, const std::string &) override
  {callback_(STOP, ""); return MotionCancelResult();}
  MotionObservation observeMotionState() const override {return MotionObservation();}
  std::uint8_t executeTaughtMotion(const std::string & motion, const std::string &, const std::string &) override
  {return callback_(MOVE_TAUGHT, motion.c_str());}
  bool commitPendingGraspRelation() override {return callback_(COMMIT, "") == Code::TASK_RESULT_SUCCESS;}
  void clearGraspRelationAfterRelease() override
  {
    if (callback_(CLEAR, "") != Code::TASK_RESULT_SUCCESS) {
      throw std::runtime_error("Fixture relation clear failed");
    }
  }
private:
  Callback callback_;
};
class FixtureTool final : public GripperPort
{
public:
  FixtureTool(Callback callback, GraspStrategy strategy) : callback_(callback), strategy_(strategy) {}
  bool supportsGraspStrategy(GraspStrategy strategy) const override {return strategy == strategy_;}
  std::uint8_t prepareGrasp(const GraspSpecification & spec) override
  {return spec.valid() && supportsGraspStrategy(spec.strategy) ? Code::TASK_RESULT_SUCCESS : Code::TASK_RESULT_INVALID_GOAL;}
  std::uint8_t grasp() override {return callback_(GRASP, "");}
  std::uint8_t release() override {return callback_(RELEASE, "");}
  std::uint8_t close() override {return Code::TASK_RESULT_INVALID_GOAL;}
  std::uint8_t open() override {return Code::TASK_RESULT_INVALID_GOAL;}
  HoldingObservation holding() const override
  {
    const int state = callback_(OBSERVE, "");
    return {state == 1 ? HoldingState::HELD : state == 2 ? HoldingState::RELEASED : HoldingState::UNKNOWN,
      HoldingObservation::Clock::now(), ++sequence_, std::nullopt};
  }
  bool active() const override {return false;}
  void stop() override {callback_(STOP, "");}
private:
  Callback callback_;
  GraspStrategy strategy_;
  mutable std::uint64_t sequence_{0};
};
}
extern "C" int arm_cell_run_grasp_cycle(const char * input, Callback callback)
{
  if (!input || !callback) {return Code::TASK_RESULT_INVALID_GOAL;}
  try {
    const auto source = nlohmann::json::parse(input);
    PreparedGraspCycle request;
    const std::string strategy = source.at("strategy");
    if (strategy == "internal_radial") {request.grasp.strategy = GraspStrategy::INTERNAL_RADIAL;}
    else if (strategy == "box_exterior") {request.grasp.strategy = GraspStrategy::BOX_EXTERIOR;}
    else {return Code::TASK_RESULT_INVALID_GOAL;}
    const auto & solution = source.at("solution");
    request.grasp.insertion_depth_m = solution.at("insertion_depth_m");
    request.grasp.thickness_m = solution.at("thickness_m");
    request.grasp.bore_radius_m = solution.at("inner_radius_m");
    request.grasp.release_position_m = solution.at("release_position_m");
    request.grasp.grasp_position_m = solution.at("grasp_position_m");
    request.approach_motion = "approach";
    request.insertion_motion = "insert";
    request.transfer_motion = "transfer";
    request.retreat_motion = "retreat";
    auto motion = std::make_shared<FixtureMotion>(callback);
    auto tool = std::make_shared<FixtureTool>(callback, request.grasp.strategy);
    TaskExecutor executor(motion, tool);
    return executor.execute_grasp_cycle(request, source.at("execution_id"));
  } catch (...) {
    return Code::TASK_RESULT_BACKEND_ERROR;
  }
}
