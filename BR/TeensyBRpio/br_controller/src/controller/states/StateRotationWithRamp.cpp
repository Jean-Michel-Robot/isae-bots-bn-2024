#include "controller/states/StateRotationWithRamp.hpp"
#include "defines/func.hpp"
#include "rotations/OrientationProfile.hpp"

namespace controller {

StateRotationWithRamp::StateRotationWithRamp(std::unique_ptr<OrientationProfile> profile, double_t maxAngSpeed, double_t maxAngAcceleration)
    : m_profile(std::move(profile)), m_maxSpeed(maxAngSpeed), m_ramp(maxAngSpeed, maxAngAcceleration) {}

StateUpdateResult StateRotationWithRamp::update(double_t interval, Position2D<Meter> &setpoint, Position2D<Meter> actualRobotPosition) {
    if (m_profile) {
        m_ramp.update(interval);
        if (m_profile->advance(m_ramp.getCurrentSpeed() * interval)) {
            setpoint.theta = m_profile->getCurrentOrientation();

            m_ramp.setTargetSpeed(m_maxSpeed);
            std::optional<double_t> remainingAngle = m_profile->getRemainingAngle();
            if (remainingAngle) {
                m_ramp.ensureCanBrake(*remainingAngle);
            }
        } else {
            m_profile.reset();
        }
    }
    if (m_profile) {
        return Ongoing();
    } else {
        return RotationComplete();
    }
}

void StateRotationWithRamp::notify(ControllerEvent event) {
    std::visit(overload{[&](const MaxSpeedsChanged &event) { m_maxSpeed = event.newSpeeds.angular; }, [](auto) {}}, event);
}

} // namespace controller