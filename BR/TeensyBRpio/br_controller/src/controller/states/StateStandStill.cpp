#include "controller/states/StateStandStill.hpp"
#include "logging.hpp"

namespace controller {

StateStandStill::StateStandStill(Position2D<Meter> &setpoint, Position2D<Meter> actualRobotPosition, bool preserveCurrentSetpoint)
    : m_restPosition(preserveCurrentSetpoint ? setpoint : actualRobotPosition) {
    log(INFO, "Entering controller state: Ready (standing still)");

    setpoint = m_restPosition;
}

ControllerStatus StateStandStill::getStatus() const {
    return Still;
}

StateUpdateResult StateStandStill::update(double_t interval, Position2D<Meter> &setpoint, Position2D<Meter> actualRobotPosition) {
    setpoint = m_restPosition;
    return Ongoing();
}

} // namespace controller
