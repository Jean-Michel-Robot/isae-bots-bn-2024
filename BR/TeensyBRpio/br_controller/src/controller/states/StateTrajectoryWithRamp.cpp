#include "controller/states/StateTrajectoryWithRamp.hpp"
#include "logging.hpp"

namespace controller {

StateTrajectoryWithRamp::StateTrajectoryWithRamp(std::unique_ptr<Trajectory> trajectory, double_t maxLinSpeed, double_t maxLinAcceleration,
                                                 DisplacementKind kind, std::optional<Angle> finalOrientation)
    : m_trajectory(std::move(trajectory)), m_maxSpeed(maxLinSpeed), m_ramp(maxLinSpeed, maxLinAcceleration), m_kind(kind),
      m_finalOrientation(finalOrientation) {
    log(INFO, "Entering controller state: Forward");
}

ControllerStatus StateTrajectoryWithRamp::getStatus() const {
    if (m_kind == FORWARD) {
        return Forward;
    } else {
        return Reversing;
    }
}

StateUpdateResult StateTrajectoryWithRamp::update(double_t interval, Position2D<Meter> &setpoint, Position2D<Meter> actualRobotPosition) {
    if (m_trajectory) {
        m_ramp.update(interval);

        if (m_trajectory->advance(m_ramp.getCurrentSpeed() * interval)) {
            Position2D<Meter> position = m_trajectory->getCurrentPosition();
            setpoint.x = position.x;
            setpoint.y = position.y;
            setpoint.theta = position.theta + m_kind.getAlignmentOffset();

            m_ramp.setTargetSpeed(m_maxSpeed);
            std::optional<double_t> remainingDistance = m_trajectory->getRemainingDistance();
            if (remainingDistance) {
                m_ramp.ensureCanBrake(*remainingDistance);
            }
        } else {
            m_trajectory.reset();
        }
    }
    if (m_trajectory) {
        if (abs(setpoint.theta - actualRobotPosition.theta) > std::numbers::pi_v<double_t> / 2.0) {
            return BadRobotOrientation(std::move(m_trajectory), m_kind, m_finalOrientation);
        }
        return Ongoing();
    } else {
        return TrajectoryComplete(m_kind, m_finalOrientation);
    }
}

void StateTrajectoryWithRamp::notify(ControllerEvent event) {
    std::visit([&](const MaxSpeedsChanged &event) { m_maxSpeed = event.newSpeeds.linear; }, event);
}

} // namespace controller