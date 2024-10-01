#ifndef _POSITION_FEEDBACK_HPP_
#define _POSITION_FEEDBACK_HPP_

#include <concepts>
#include <math.h> // type double_t

#if 0
// PSEUDO-CODE (for documentation purposes only)
/// The minimal interface a position feedback must have.
class PositionFeedback {
    /**
     * Updates the estimated/simulated/measured position of the robot.
     * 
     * @param interval The time elapsed since the last update. 
     */
    void update(double_t interval);
    void resetPosition(Position2D<Meter> position);
    Position2D<Meter>> getRobotPosition() const;
};
#endif

/// The minimal interface a position feedback provider must have.
template <typename T>
concept PositionFeedback = requires(T fb, const T fb_const, double_t interval, Position2D<Meter> pos) {
    fb.update(interval);
    fb.resetPosition(pos.toMillimeters());
    { fb_const.getRobotPosition() } -> std::convertible_to<Position2D<Meter>>;
};

#endif