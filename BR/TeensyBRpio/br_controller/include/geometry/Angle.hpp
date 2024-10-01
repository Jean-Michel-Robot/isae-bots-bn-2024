#ifndef _ANGLE_HPP_
#define _ANGLE_HPP_

#include "defines/math.hpp"

template <typename Unit>
class Vector2D;

/**
 * A signed angle in radians, in (-PI, PI]. This type supports implicit conversion
 * from and to double_t.
 */
class Angle {
  public:
    Angle(double_t value);

    operator double_t();
    double_t value() const;

    /// @return this + PI
    Angle reverse() const;

    Angle operator-() const;

    void operator+=(Angle other);
    void operator-=(Angle other);
    void operator*=(double_t factor);
    void operator/=(double_t factor);

    bool operator==(const Angle &other) const = default;
    Angle operator+(Angle other) const;
    Angle operator-(Angle other) const;
    Angle operator*(double_t factor) const;
    Angle operator/(double_t factor) const;

  private:
    double_t m_value;
};

template <typename Unit>
inline Angle operator*(double_t factor, Angle angle) {
    return angle * factor;
}

namespace std {
inline double_t cos(Angle angle) {
    return cos((double_t)angle);
}
inline double_t sin(Angle angle) {
    return sin((double_t)angle);
}
} // namespace std

#endif