#ifndef _CONTROLLER_STATE_FINAL_ROTATION_HPP_
#define _CONTROLLER_STATE_FINAL_ROTATION_HPP_

#include "controller/states/StateRotationWithRamp.hpp"

class OrientationProfile;

namespace controller {

class StateFinalRotation : public StateRotationWithRamp {
  public:
    /// @param profile must not be null
    StateFinalRotation(std::unique_ptr<OrientationProfile> profile, double_t maxAngSpeed, double_t maxAngAcceleration);
    ControllerStatus getStatus() const override;
    StateUpdateResult update(double_t interval, Position2D<Meter> &setpoint, Position2D<Meter> actualRobotPosition) override;
};
} // namespace controller

#endif