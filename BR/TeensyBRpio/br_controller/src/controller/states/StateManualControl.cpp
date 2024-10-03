#include "controller/states/StateManualControl.hpp"
#include "defines/func.hpp"
#include "logging.hpp"

namespace controller {

StateManualControl::StateManualControl(Accelerations maxAcceleration)
    : m_linearRamp(0, maxAcceleration.linear), m_angularRamp(0, maxAcceleration.angular) {
    log(INFO, "Entering controller state: Manual control");
}

ControllerStatus StateManualControl::getStatus() const {
    return ManualControl;
}

StateUpdateResult StateManualControl::update(double_t interval, Position2D<Meter> &setpoint, Position2D<Meter> actualRobotPosition) {
    m_linearRamp.update(interval);
    m_angularRamp.update(interval);

    setpoint = setpoint.relativeOffset(m_linearRamp.getCurrentSpeed() * interval);
    setpoint.theta += m_angularRamp.getCurrentSpeed() * interval;

    return Ongoing();
}

void StateManualControl::notify(ControllerEvent event) {
    std::visit(overload{[&](const ManualSpeedCommand &event) {
                            m_linearRamp.setTargetSpeed(event.speeds.linear);
                            m_angularRamp.setTargetSpeed(event.speeds.angular);
                            if (!event.enforceMaxAcceleration) {
                                m_linearRamp.overwriteCurrentSpeed(event.speeds.linear);
                                m_angularRamp.overwriteCurrentSpeed(event.speeds.angular);
                            }
                        },
                        [](auto) {}},
               event);
}

} // namespace controller
