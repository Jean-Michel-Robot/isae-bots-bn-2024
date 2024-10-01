#ifndef _CONTROLLER_STATE_BRAKING_HPP_
#define _CONTROLLER_STATE_BRAKING_HPP_

#include "controller/ControllerState.hpp"
#include "geometry/Speeds.hpp"
#include "math/Ramp.hpp"

#include <memory>

namespace controller {

class StateBraking : public ControllerState {
  public:
    /// @param robotSpeed must not be null
    StateBraking(std::shared_ptr<const Speeds> robotSpeed, Accelerations brakingDecelerations);
    ControllerStatus getStatus() const override;
    StateUpdateResult update(double_t interval, Position2D<Meter> &setpoint, Position2D<Meter> actualRobotPosition) override;

  private:
    std::shared_ptr<const Speeds> m_speed;
    Ramp m_linear_ramp;
    Ramp m_angular_ramp;
};
} // namespace controller

#endif