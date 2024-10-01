#ifndef _ACTUATORS_HPP_
#define _ACTUATORS_HPP_

#include "geometry/Position2D.hpp"

#include <concepts>
#include <math.h> // type double_t

#if 0
// PSEUDO-CODE (for documentation purposes only)

/// The minimal interface an hardware-agnostic actuator interface must have.
/// isReady() and isIdle() may not both return true at the same time
class Actuators {
    /// Initiates transition: Idle --> Ready (closed loop).
    void switchOn();
    /// Initiates transition: Ready (closed loop) --> Idle
    void switchOff();
    /// Updates the state of the actuators. What this actually does is actuator-dependent.
    /// @param interval The time elapsed since the last update, in the time frame of the controller.
    void update(double_t interval);
    /// Indicates whether all actuators are ready
    bool isReady();
    /// Indicates whether all actuators are idle
    bool isIdle();
};

#endif

template <typename T>
concept Actuators = requires(T a, double_t interval) {
    a.switchOn();
    a.switchOff();
    a.update(interval);
    { a.isReady() } -> std::convertible_to<bool>;
    { a.isIdle() } -> std::convertible_to<bool>;
};

template <typename TController, typename TActuators>
concept CanControl =
    requires(TController c, TActuators a, double_t interval, Position2D<Meter> pos) { a.sendCommand(c.updateCommand(interval, pos)); };

#endif