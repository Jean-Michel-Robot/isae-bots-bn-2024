#ifndef _CONTROLLER_STATE_ROTATION_RAMP_HPP_
#define _CONTROLLER_STATE_ROTATION_RAMP_HPP_

#include "controller/ControllerState.hpp"
#include "math/Ramp.hpp"

class OrientationProfile;

namespace controller {

class StateRotationWithRamp : public ControllerState {
  public:
    StateUpdateResult update(double_t interval, Position2D<Meter> &setpoint, Position2D<Meter> actualRobotPosition) override;
    void notify(ControllerEvent event) override;

  protected:
    StateRotationWithRamp(std::unique_ptr<OrientationProfile> profile, double_t maxAngSpeed, double_t maxAngAcceleration);

  private:
    std::unique_ptr<OrientationProfile> m_profile;
    double_t m_maxSpeed;
    Ramp m_ramp;
};

} // namespace controller
#endif