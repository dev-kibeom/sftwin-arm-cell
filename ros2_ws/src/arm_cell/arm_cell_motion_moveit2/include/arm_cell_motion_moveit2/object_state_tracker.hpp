#pragma once

#include <cstdint>

namespace arm_cell_motion_moveit2
{

enum class MotionObjectState : uint8_t
{
  NO_OBJECT,
  ATTACHED,
  UNCERTAIN
};

class ObjectStateTracker
{
public:
  MotionObjectState state() const {return state_;}
  void set_state(MotionObjectState state) {state_ = state;}
  void mark_attached() {state_ = MotionObjectState::ATTACHED;}
  void mark_detached() {state_ = MotionObjectState::NO_OBJECT;}
  void mark_uncertain() {state_ = MotionObjectState::UNCERTAIN;}

private:
  MotionObjectState state_{MotionObjectState::NO_OBJECT};
};

}  // namespace arm_cell_motion_moveit2
