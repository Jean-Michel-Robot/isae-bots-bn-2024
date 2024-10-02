#ifndef _CONTROLLER_STATE_HPP_
#define _CONTROLLER_STATE_HPP_

#include "controller/ControllerEvent.hpp"
#include "controller/StateResult.hpp"
#include "defines/math.hpp"
#include "geometry/Position2D.hpp"

namespace controller {

/**
 * Bit-flag:
 * - ControllerStatus & 0b00001  = Is stopping (not set when already stopped)?
 * - ControllerStatus & 0b00010  = Is a trajectory ongoing or pending?
 * - ControllerStatus & 0b00100  = Is moving straight (not set when braking)?
 * - ControllerStatus & 0b01000  = Is rotating (not set when braking)?
 * - ControllerStatus & 0b10000  = Reserved for future use?
 */
enum ControllerStatus : uint32_t {
    Invalid = 0b111100000,
    /// The robot is standing still close to its rest point
    Still = 0,
    /// The robot is braking to stop
    Braking = 1,
    /// Suspending trajectory due to a bad orientation. The robot is braking and then will go back in state InitialRotation.
    SuspendingTrajectory = 0b11,

    /// The robot is rotating in the direction of the trajectory it has to follow (required due to being unicycle)
    InitialRotation = 0b1010,
    /// The robot is following a trajectory
    Forward = 0b110,
    /// The robot is following a reverse trajectory
    Reversing = 0b100110,
    /// The robot is rotating in the final requested direction (the final rotation is not considered to be part of the trajectory)
    FinalRotation = 0b1000,

    ManualControl = 0b101100
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