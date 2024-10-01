#include "controller/states/StateBraking.hpp"
#include "logging.hpp"

namespace controller {

StateBraking::StateBraking(std::shared_ptr<const Speeds> robotSpeed, Accelerations brakingDecelerations)
    : m_speed(std::move(robotSpeed)), m_linear_ramp(0, brakingDecelerations.linear, m_speed->linear),
      m_angular_ramp(0, brakingDecelerations.angular, m_speed->angular) {
    log(INFO, "Entering controller state: Braking to stop");
}

ControllerStatus StateBraking::getStatus() const {
    return Braking;
}

StateUpdateResult StateBraking::update(double_t interval, Position2D<Meter> &setpoint, Position2D<Meter> actualRobotPosition) {
    m_linear_ramp.update(interval);
    m_angular_ramp.update(interval);

    setpoint = setpoint.relativeOffset(m_linear_ramp.getCurrentSpeed() * interval, 0);
    setpoint.theta += m_angular_ramp.getCurrentSpeed() * interval;

    if (m_linear_ramp.getCurrentSpeed() == 0 && m_angular_ramp.getCurrentSpeed() == 0) {
        return BrakingComplete();
    } else {
        return Ongoing();
    }
}

} // namespace controller