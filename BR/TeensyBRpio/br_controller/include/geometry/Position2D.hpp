#ifndef _POSITION_2D_HPP_
#define _POSITION_2D_HPP_

#include "defines/math.hpp"
#include "geometry/Angle.hpp"
#include "geometry/Vector2D.hpp"

#ifdef _DEBUG
#include <string>
#endif

/** A 2D-position with orientation. The unit of x and y is given by type parameter `Unit`.
 * theta is in radians.
 *
 * @tparam Unit The unit of x and y. Only `Meter` and `Millimeter` are supported.
 */
template <typename Unit>
class Position2D : public Vector2D<Unit> {
  public:
    Position2D();
    Position2D(double_t x, double_t y, Angle theta);
    Position2D(Vector2D<Unit> position, Angle theta = 0);

    template <typename To>
        requires Convertible<Unit, To>
    Position2D<To> convert() const {
        return Position2D<To>(Vector2D<Unit>::template convert<To>(), theta);
    }

    Position2D<Millimeter> toMillimeters() const
        requires Convertible<Unit, Millimeter>
    {
        return convert<Millimeter>();
    }
    Position2D<Meter> toMeters() const
        requires Convertible<Unit, Meter>
    {
        return convert<Meter>();
    }

    /**
     * Returns the absolute coordinates of `vector`.
     * @param vector The relative coordinates of the vector in relation to this position.
     */
    Vector2D<Unit> makeAbsolute(Vector2D<Unit> vector) const;

    /**
     * Returns the relative coordinates of `vector` in relation to this position.
     * @param vector The absolute coordinates of the vector, in the same global frame as this position.
     */
    Vector2D<Unit> makeRelative(Vector2D<Unit> vector) const;

    /**
     * Returns the new position after applying the specified relative offset, without changing theta.
     * @param xr The signed distance to move in the direction of this position's theta.
     * @param yr The signed distance to move in the direction normal to this position's theta.
     */
    Position2D<Unit> relativeOffset(double_t xr, double_t yr = 0) const;

    void operator+=(Position2D<Unit> pos);
    void operator-=(Position2D<Unit> pos);
    /// Note: this also multiplies theta
    void operator*=(double_t factor);
    /// Note: this also divides theta
    void operator/=(double_t factor);

    bool operator==(const Position2D<Unit> &pos) const = default;
    Position2D<Unit> operator+(Position2D<Unit> pos) const;
    Position2D<Unit> operator-(Position2D<Unit> pos) const;
    /// Note: this also multiplies theta
    Position2D<Unit> operator*(double_t factor) const;
    /// Note: this also divides theta
    Position2D<Unit> operator/(double_t factor) const;

    Angle theta;

#ifdef _DEBUG
    operator std::string() const {
        return "(" + std::to_string(this->x) + ", " + std::to_string(this->y) + ", " + std::to_string(theta.value()) + ")";
    }
#endif
};

template <typename Unit>
inline Position2D<Unit> operator*(double_t factor, Position2D<Unit> pos) {
    return pos * factor;
}

#endif