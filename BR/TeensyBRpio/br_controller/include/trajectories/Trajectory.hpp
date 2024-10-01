#ifndef _TRAJECTORY_HPP_
#define _TRAJECTORY_HPP_

#include "defines/math.hpp"
#include "geometry/Position2D.hpp"

#include <optional>

/**
 * A continuous (reasonably smooth, i.e. at least of class C2) 2D-trajectory the robot is expected to follow.
 *
 * The trajectory is described by the successive positions of a point that moves along the trajectory.
 * This point can be seen as the target position for the robot.
 *
 * When the `Trajectory` instance is created, this point must be at the beginning of the trajectory.
 * The implementation is responsible for keeping track of this point internally as it advances.
 *
 * If the trajectory has tight curves, the robot may not be able to follow it in an accurate way. The controller and this
 * class might be extended later to implement automatic braking before curves.
 *
 * This class must be inherited.
 */
class Trajectory {
  public:
    /**
     * Advances the current position on the trajectory by the given distance.
     * @param distance The distance (in meters) between the current trajectory position and the new position. If the new position would fall
     * outside the trajectory, then the position should be advanced to the end of the trajectory instead.
     * @return true if the position was advanced. false if the position is already at the end of the trajectory. If this method returns false,
     * subsequent calls to advance() must also return false.
     */
    virtual bool advance(double_t distance) = 0;

    /**
     * Returns the current position on the trajectory. The position must be heading forward in the current local direction of the trajectory.
     * After advance() has returned false, this must return the last point of the trajectory.
     */
    virtual Position2D<Meter> getCurrentPosition() const = 0;

    /**
     * Returns the distance between the current position and the end of the trajectory. This allows to implement a deceleration ramp to stop the robot
     * as close to the end of the trajectory as possible (without overshoot and ringing).
     *
     * - 0 means the current position has reached the end of the trajectory. It is a logic error to return 0 unless advance() would return false.
     * - An empty optional means the trajectory is inifite or the remaining distance cannot be determined.
     * - Negative values are illegal and will lead to unspecified behaviour.
     *
     * After advance() has returned false, this must return 0.
     */
    virtual std::optional<double_t> getRemainingDistance() const = 0;

    /**
     * Returns the maximum curve of the trajectory for the next `distance` meters, starting at the current position. 0 means the trajectory is a
     * straight line.
     *
     * This will eventually allow to implement automatic braking before curves, but is not implemented yet.
     *
     * The default implementation returns 0. Overriding this method currently is not required and has no effect.
     */
    [[deprecated("Not implemented yet")]] virtual double_t getMaxCurve(double_t distance) const;

  protected:
    Trajectory() = default;
};

#endif