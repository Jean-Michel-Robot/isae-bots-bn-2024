#ifndef _VECTOR_2D_HPP_
#define _VECTOR_2D_HPP_

#include "defines/math.hpp"
#include "geometry/Angle.hpp"

#ifdef _DEBUG
#include <string>
#endif

/** A 2D-vector.
 *
 * @tparam Unit The unit of x and y. Only `Meter` and `Millimeter` are supported.
 */
template <typename Unit>
class Vector2D {
  public:
    Vector2D();
    Vector2D(double_t x, double_t y);

    template <typename To>
        requires Convertible<Unit, To>
    Vector2D<To> convert() const {
        return Vector2D<To>(Unit::template convert<To>(x), Unit::template convert<To>(y));
    }

    Vector2D<Millimeter> toMillimeters() const
        requires Convertible<Unit, Millimeter>
    {
        return convert<Millimeter>();
    }
    Vector2D<Meter> toMeters() const
        requires Convertible<Unit, Meter>
    {
        return convert<Meter>();
    }

    double_t norm() const;
    /**
     * Returns the signed angle between the vector (1, 0) and the current vector, or, equivalently, the argument of the complex number x+iy.
     * If this is called on the zero vector, zero is returned instead.
     */
    Angle argument() const;
    /**
     * Returns the vector that has the same direction as this vector and norm 1.
     * If this is called on the zero vector, the zero vector is returned instead.
     */
    Vector2D<Unit> normalize() const;

    void operator+=(Vector2D<Unit> pos);
    void operator-=(Vector2D<Unit> pos);
    void operator*=(double_t factor);
    void operator/=(double_t factor);

    bool operator==(const Vector2D<Unit> &pos) const = default;
    Vector2D<Unit> operator+(Vector2D<Unit> pos) const;
    Vector2D<Unit> operator-(Vector2D<Unit> pos) const;
    Vector2D<Unit> operator*(double_t factor) const;
    Vector2D<Unit> operator/(double_t factor) const;

    static double_t distance(Vector2D<Unit> a, Vector2D<Unit> b);
    /**
     * Returns the signed angle between a and b (in radians).
     * If any of both vectors is the zero vector, 0 is returned instead.
     */
    static Angle angle(Vector2D<Unit> a, Vector2D<Unit> b);
    static double_t dot(Vector2D<Unit> a, Vector2D<Unit> b);

    double_t x;
    double_t y;

#ifdef _DEBUG
    operator std::string() const { return "(" + std::to_string(x) + ", " + std::to_string(y) + ")"; }
#endif
};

template <typename Unit>
inline Vector2D<Unit> operator*(double_t factor, Vector2D<Unit> pos) {
    return pos * factor;
}

/// See Vector2D. The alias is used to make it clear that a value is considered as a point instead of a vector.
template <typename Unit>
using Point2D = Vector2D<Unit>;

#endif