#pragma once

#include <memory>

#include "arm_cell_motion/gripper_port.hpp"
#include "arm_cell_motion/isaac_motion_backend.hpp"

namespace arm_cell_motion
{

class IsaacGripperPort final : public GripperPort
{
public:
  explicit IsaacGripperPort(std::shared_ptr<IsaacMotionTransport> transport);

  uint8_t close() override;
  uint8_t open() override;
  HoldingObservation holding() const override;
  bool active() const override;
  void stop() override;

private:
  std::shared_ptr<IsaacMotionTransport> transport_;
};

}  // namespace arm_cell_motion
