#ifndef _CONTROLLER_STATE_MANUAL_CONTROL_HPP_
#define _CONTROLLER_STATE_MANUAL_CONTROL_HPP_

#include "controller/ControllerState.hpp"
#include "geometry/Speeds.hpp"

namespace controller {

class StateManualControl : public ControllerState {
  public:
    StateManualControl();
    ControllerStatus getStatus() const override;
    StateUpdateResult update(double_t interval, Position2D<Meter> &setpoint, Position2D<Meter> actualRobotPosition) override;
    void notify(ControllerEvent event) override;

    private:
    Speeds m_speeds;
};
} // namespace controller

#endif