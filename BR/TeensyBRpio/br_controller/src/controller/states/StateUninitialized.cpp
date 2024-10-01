#include "controller/states/StateUninitialized.hpp"
#include "logging.hpp"

namespace controller {

StateUninitialized::StateUninitialized() {}

ControllerStatus StateUninitialized::getStatus() const {
    return Invalid;
}

StateUpdateResult StateUninitialized::update(double_t interval, Position2D<Meter> &setpoint, Position2D<Meter> actualRobotPosition) {
    log(ERROR, "updateCommand called before the controller is initialized; you are missing a call to reset().");
    return Ongoing();
}

} // namespace controller
