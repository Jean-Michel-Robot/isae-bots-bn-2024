#ifndef _CONTROLLER_STATE_UNINIT_HPP_
#define _CONTROLLER_STATE_UNINIT_HPP_

#include "controller/ControllerState.hpp"

namespace controller {

class StateUninitialized : public ControllerState {
  public:
    StateUninitialized();
    ControllerStatus getStatus() const override;
    StateUpdateResult update(double_t interval, Position2D<Meter> &setpoint, Position2D<Meter> actualRobotPosition) override;
};
} // namespace controller

#endif