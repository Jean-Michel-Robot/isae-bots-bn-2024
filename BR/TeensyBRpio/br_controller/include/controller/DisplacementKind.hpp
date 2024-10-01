#ifndef _DISPLACEMENT_KIND_HPP_
#define _DISPLACEMENT_KIND_HPP_

#include "geometry/Angle.hpp"

class DisplacementKind {
  public:
    enum Value { FORWARD, REVERSE };
    constexpr DisplacementKind(Value kind) : m_value(kind) {}
    constexpr operator Value() const { return m_value; }
    explicit operator bool() const = delete;

    /// Difference between the direction of the trajectory and the orientation of the robot
    Angle getAlignmentOffset() const { return m_value == FORWARD ? 0 : std::numbers::pi_v<double_t>; }

  private:
    Value m_value;
};

const DisplacementKind FORWARD = DisplacementKind::FORWARD;
const DisplacementKind REVERSE = DisplacementKind::REVERSE;

#endif