#ifndef _CONTROLLER_STATE_STAND_STILL_HPP_
#define _CONTROLLER_STATE_STAND_STILL_HPP_

#include "controller/ControllerState.hpp"
#include "geometry/Position2D.hpp"

namespace controller {

class StateStandStill : public ControllerState {
  public:
    StateStandStill(Position2D<Meter> &setpoint, Position2D<Meter> actualRobotPosition, bool preserveCurrentSetpoint = false);
    ControllerStatus getStatus() const override;
    StateUpdateResult update(double_t interval, Position2D<Meter> &setpoint, Position2D<Meter> actualRobotPosition) override;

  private:
    Position2D<Meter> m_restPosition;
};
} // namespace controller

#endif