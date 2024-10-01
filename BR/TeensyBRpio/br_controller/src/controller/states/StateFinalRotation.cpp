#include "controller/states/StateFinalRotation.hpp"
#include "logging.hpp"
#include "rotations/OrientationProfile.hpp"

namespace controller {

StateFinalRotation::StateFinalRotation(std::unique_ptr<OrientationProfile> profile, double_t maxAngSpeed, double_t maxAngAcceleration)
    : StateRotationWithRamp(std::move(profile), maxAngSpeed, maxAngAcceleration) {
    log(INFO, "Entering controller state: Final rotation");
}

ControllerStatus StateFinalRotation::getStatus() const {
    return FinalRotation;
}

StateUpdateResult StateFinalRotation::update(double_t interval, Position2D<Meter> &setpoint, Position2D<Meter> actualRobotPosition) {
    StateUpdateResult result = StateRotationWithRamp::update(interval, setpoint, actualRobotPosition);
    if (std::holds_alternative<RotationComplete>(result)) {
        return FinalRotationComplete();
    } else {
        return result;
    }
}

} // namespace controller