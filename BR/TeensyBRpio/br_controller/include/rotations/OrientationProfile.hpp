#ifndef _ORIENTATE_PROFILE_HPP_
#define _ORIENTATE_PROFILE_HPP_

#include "defines/math.hpp"
#include "geometry/Angle.hpp"
#include <optional>

/**
 * Describes an orientation profile. The profile is described by the successive orientations of an object that follows the profile.
 *
 * This class must be inherited.
 */
class OrientationProfile {
  public:
    /**
     * Advance the profile by the following angle.
     *
     * @param angleDist The absolute angular distance to advance the profile.
     * @return true if the profile was advanced. false if it is already complete. If this method returns false, subsequent calls to advance()
     * must also return false.
     */
    virtual bool advance(double_t diffAngle) = 0;

    /**
     * Get the current orientation of this profile.
     * After advance() has returned false, this must return the final orientation of the profile.
     */
    virtual Angle getCurrentOrientation() const = 0;

    /**
     * Returns the absolute angular distance between the current position and the end of the profile. This allows to implement a deceleration ramp to
     * stop the robot as close to the final orientation as possible (without overshoot and ringing).
     *
     * - 0 means the profile is complete It is a logic error to return 0 unless advance() would return false.
     * - An empty optional means the profile is inifite or the remaining distance cannot be determined.
     * - Negative values are illegal and will lead to unspecified behaviour.
     *
     * After advance() has returned false, this must return 0.
     */
    virtual std::optional<double_t> getRemainingAngle() const = 0;

  protected:
    OrientationProfile() = default;
};

#endif