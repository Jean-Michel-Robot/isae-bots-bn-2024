#ifndef _CONTROLLER_STATE_HPP_
#define _CONTROLLER_STATE_HPP_

#include "controller/ControllerEvent.hpp"
#include "controller/StateResult.hpp"
#include "defines/math.hpp"
#include "geometry/Position2D.hpp"

namespace controller {

enum StatusFlag : uint32_t {
    /// Is stopping (not set when already stopped)?
    STOPPING = 0b00001,
    /// Is a trajectory ongoing or pending?
    TRAJECTORY = 0b00010,
    /// Is moving straight (not set when braking)?
    MOVING = 0b00100,
    /// Is rotating (not set when braking)?
    ROTATING = 0b01000,
    /// Speed control instead of position control. This flag may alter how the controller computes the command.
    SPEED_CONTROL = 0b10000,

    /// No special meaning. Used to distinguish states with the same meaningful flags.
    EXTRA_1 = 0b100000
};

enum ControllerStatus : uint32_t {
    Invalid = ~(SPEED_CONTROL | ROTATING | MOVING | TRAJECTORY | STOPPING),
    /// The robot is standing still close to its rest point
    Still = 0,
    /// The robot is braking to stop
    Braking = SPEED_CONTROL | STOPPING,
    /// Suspending trajectory due to a bad orientation. The robot is braking and then will go back in state InitialRotation.
    SuspendingTrajectory = SPEED_CONTROL | TRAJECTORY | STOPPING,

    /// The robot is rotating in the direction of the trajectory it has to follow (required due to being unicycle)
    InitialRotation = ROTATING | TRAJECTORY,
    /// The robot is following a trajectory
    Forward = MOVING | TRAJECTORY,
    /// The robot is following a reverse trajectory
    Reversing = EXTRA_1 | MOVING | TRAJECTORY,
    /// The robot is rotating in the final requested direction (the final rotation is not considered to be part of the trajectory)
    FinalRotation = ROTATING,
    ManualControl = SPEED_CONTROL | ROTATING | MOVING
};

/**
 * Base class for the state of a controller.
 */
class ControllerState {
  public:
    virtual ControllerStatus getStatus() const = 0;
    /// @param setpoint The reference must not escape the function
    virtual StateUpdateResult update(double_t interval, Position2D<Meter> &setpoint, Position2D<Meter> actualRobotPosition) = 0;
    virtual void notify(ControllerEvent event) {}

  protected:
    ControllerState() = default;
};

} // namespace controller

#endif