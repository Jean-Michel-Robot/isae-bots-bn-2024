#ifndef _CONTROLLER_STATE_SUSPEND_TRAJECTORY_HPP_
#define _CONTROLLER_STATE_SUSPEND_TRAJECTORY_HPP_

#include "controller/states/StateBraking.hpp"

namespace controller {

/// Needs to stop then re-align the robot with trajectory before continuing the displacement.
class StateSuspendTrajectory : public StateBraking {
  public:
    /// @param robotSpeed must not be null
    StateSuspendTrajectory(Speeds robotSpeed, Accelerations brakingDeceleration, TrajectoryContainer suspendedTrajectory);
    ControllerStatus getStatus() const override;
    /// It is undefined behaviour to call this function after it has returned BrakingComplete
    StateUpdateResult update(double_t interval, Position2D<Meter> &setpoint, Position2D<Meter> actualRobotPosition) override;

  private:
    TrajectoryContainer m_suspendedTrajectory;
};
} // namespace controller

#endif