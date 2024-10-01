#ifndef _CONTROLLER_STATE_INITIAL_ROTATION_HPP_
#define _CONTROLLER_STATE_INITIAL_ROTATION_HPP_

#include "controller/states/StateRotationWithRamp.hpp"

class Trajectory;

namespace controller {

class StateInitialRotation : public StateRotationWithRamp {
  public:
    /**
     * @param trajectory must not be null
     * @param setpoint the reference must not escape the function
     */
    StateInitialRotation(std::unique_ptr<Trajectory> trajectory, Position2D<Meter> &setpoint, double_t maxAngSpeed, double_t maxAngAcceleration,
                         DisplacementKind kind, std::optional<Angle> finalOrientation = std::nullopt);

    ControllerStatus getStatus() const override;
    /// It is undefined behaviour to call this method again after it has returned InitialRotationComplete
    StateUpdateResult update(double_t interval, Position2D<Meter> &setpoint, Position2D<Meter> actualRobotPosition) override;

  private:
    std::unique_ptr<Trajectory> m_trajectory;
    DisplacementKind m_kind;
    std::optional<Angle> m_finalOrientation;
};
} // namespace controller

#endif