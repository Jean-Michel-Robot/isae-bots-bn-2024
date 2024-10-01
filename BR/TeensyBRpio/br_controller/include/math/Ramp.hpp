#ifndef _RAMP_HPP_
#define _RAMP_HPP_

#include "defines/math.hpp"

/**
 * A speed ramp with controlled acceleration.
 *
 * The ramp does not update autonomously and must be updated at regular intervals using `Ramp::update`.
 *
 * The ramp currently uses a linear interpolation (i.e. the maximum acceleration is applied whenever possible);
 * however, this is not a specification and could change in the future.
 */
class Ramp {
  public:
    /**
     * Creates a new ramp with the given target speed, maximal acceleration, and initial speed.
     * See Ramp::setTargetSpeed and Ramp::setMaximalAcceleration.
     */
    Ramp(double_t targetSpeed, double_t maximalAcceleration, double_t currentSpeed = 0);

    /** Returns the current speed computed by the ramp. The unit is the same as the target speed set with Ramp::setTargetSpeed. */
    double_t getCurrentSpeed() const;
    /** Returns the target speed set with Ramp::setTargetSpeed. */
    double_t getTargetSpeed() const;
    /** Returns the maximal acceleration set with Ramp::setMaximalAcceleration. */
    double_t getMaximalAcceleration() const;

    /**
     * Immediately sets the current speed to the given value, ignoring the ramp acceleration. See update() to enforce the maximum acceleration.
     */
    void overwriteCurrentSpeed(double_t speed);

    /**
     * Sets the signed target speed of the robot. The unit can be either meters per second (for linear speeds) or radians per
     * second (for angular speeds), but should be consistent with the acceleration.
     *
     * NB: Changing the target speed does NOT reset the current speed.
     */
    void setTargetSpeed(double_t speed);
    /**
     * Sets the maximal absolute acceleration (and deceleration) of the robot. The unit can be either meters per second
     * squared (for linear accelerations) or radians per second squared (for angular accelerations), but should be consistent
     * with the target speed.
     *
     * NB: Changing the maximal acceleration does NOT reset the current speed.
     */
    void setMaximalAcceleration(double_t acceleration);

    /**
     * Updates the current speed towards the target speed, while enforcing the maximal acceleration. If the current speed is already equal
     * to the target speed, the speed is not updated.
     *
     * @param interval The time elapsed since the last update.
     */
    void update(double_t interval);

    /**
     * Compute the maximum acceptable speed to be able to stop in at most `distance` meters, considering the braking deceleration is
     * constant and equal to the maximal acceleration of this ramp.
     * This returns an absolute (i.e. unsigned) speed.
     */
    double_t computeSpeedFromBrakingDistance(double_t distance) const;

    /**
     * Reduces the target speed if the current braking distance is higher than the specified distance. This does not alter the current speed.
     *
     * See computeSpeedFromBrakingDistance().
     */
    void ensureCanBrake(double_t distance);

  private:
    /// Flag indicating whether targetSpeed=currentSpeed, to avoid recomputing the condition at every update.
    bool m_isComplete;
    double_t m_currentSpeed;
    double_t m_targetSpeed;
    double_t m_maxAcceleration;
};

#endif