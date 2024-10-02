#include "controller/states/StateManualControl.hpp"
#include "defines/func.hpp"
#include "logging.hpp"

namespace controller {

StateManualControl::StateManualControl() : m_speeds() {
    log(INFO, "Entering controller state: Manual control");
}

ControllerStatus StateManualControl::getStatus() const {
    return ManualControl;
}

StateUpdateResult StateManualControl::update(double_t interval, Position2D<Meter> &setpoint, Position2D<Meter> actualRobotPosition) {
    setpoint = setpoint.relativeOffset(m_speeds.linear * interval);
    setpoint.theta += m_speeds.angular * interval;
    
    return Ongoing();
}

void StateManualControl::notify(ControllerEvent event) {
    std::visit(overload{[&](const ManualSpeedCommand &event) { m_speeds = event.speeds; }, [](auto) {}}, event);
}

} // namespace controller
