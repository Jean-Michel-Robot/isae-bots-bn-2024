#ifndef _TRAJECTORY_SPEEDS_HPP_
#define _TRAJECTORY_SPEEDS_HPP_

#include "defines/math.hpp"

// TODO: rename class Speeds to a name that can apply to Accelerations and Tolerances as well, and
// create an alias for Speeds.

/**
 * A pair of a linear speed (or acceleration or distance) and an angular speed (or acceleration or distance). Whether signed or absolute
 * values are expected must be specified by each use case.
 */
class Speeds {
  public:
    Speeds() = default;
    Speeds(double_t linear, double_t angular);

    Speeds operator+(Speeds other) const;
    Speeds operator-(Speeds other) const;
    Speeds operator*(double_t factor) const;
    Speeds operator/(double_t factor) const;

    /// (Maybe signed) linear speed or acceleration (m/s or m/s²)
    double_t linear;
    /// (Maybe signed) angular speed or acceleration (rad/s or rad/s²)
    double_t angular;
};

/// See class Speeds
using Accelerations = Speeds;

/// See class Speeds
using Tolerances = Speeds;

#endif