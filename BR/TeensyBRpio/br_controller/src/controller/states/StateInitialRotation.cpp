#include "controller/states/StateInitialRotation.hpp"
#include "logging.hpp"
#include "rotations/SetHeadingProfile.hpp"

namespace controller {

StateInitialRotation::StateInitialRotation(std::unique_ptr<Trajectory> trajectory, Position2D<Meter> &setpoint, double_t maxAngSpeed,
                                           double_t maxAngAcceleration, DisplacementKind kind, std::optional<Angle> finalOrientation)
    : StateRotationWithRamp(std::make_unique<SetHeadingProfile>(setpoint.theta, trajectory->getCurrentPosition().theta + kind.getAlignmentOffset()),
                            maxAngSpeed, maxAngAcceleration),
      m_trajectory(std::move(trajectory)), m_kind(kind), m_finalOrientation(finalOrientation) {
    log(INFO, "Entering controller state: Initial rotation");
}

ControllerStatus StateInitialRotation::getStatus() const {
    return InitialRotation;
}

StateUpdateResult StateInitialRotation::update(double_t interval, Position2D<Meter> &setpoint, Position2D<Meter> actualRobotPosition) {
#ifdef _DEBUG
    if (!m_trajectory) {
        abort("Illegal call to StateInitialRotation::update after m_trajectory was moved out.");
    }
#endif

    StateUpdateResult result = StateRotationWithRamp::update(interval, setpoint, actualRobotPosition);
    if (std::holds_alternative<RotationComplete>(result)) {
        return InitialRotationComplete(std::move(m_trajectory), m_kind, m_finalOrientation);
    } else {
        return result;
    }
}

} // namespace controller