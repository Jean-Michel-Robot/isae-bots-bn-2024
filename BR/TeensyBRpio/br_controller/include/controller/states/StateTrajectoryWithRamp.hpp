#ifndef _CONTROLLER_STATE_DISPLACEMENT_RAMP_HPP_
#define _CONTROLLER_STATE_DISPLACEMENT_RAMP_HPP_

#include "controller/ControllerState.hpp"
#include "geometry/Angle.hpp"
#include "math/Ramp.hpp"

class Trajectory;

#include <optional>

namespace controller {

class StateTrajectoryWithRamp : public ControllerState {
  public:
    /// @param trajectory must not be null
    StateTrajectoryWithRamp(std::unique_ptr<Trajectory> trajectory, double_t maxLinSpeed, double_t maxLinAcceleration, DisplacementKind kind,
                            std::optional<Angle> finalOrientation = std::nullopt);
    ControllerStatus getStatus() const override;
    /// It is undefined behaviour to call this function after it has returned BadRobotOrientation
    StateUpdateResult update(double_t interval, Position2D<Meter> &setpoint, Position2D<Meter> actualRobotPosition) override;
    void notify(ControllerEvent event) override;

  private:
    std::unique_ptr<Trajectory> m_trajectory;
    double_t m_maxSpeed;
    Ramp m_ramp;

    DisplacementKind m_kind;
    std::optional<Angle> m_finalOrientation;
};
} // namespace controller

#endif