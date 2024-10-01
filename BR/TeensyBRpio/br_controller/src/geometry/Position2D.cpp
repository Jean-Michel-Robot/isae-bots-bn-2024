#include "geometry/Position2D.hpp"
#include <cmath>

template <typename Unit>
Position2D<Unit>::Position2D() : Vector2D<Unit>(), theta(0) {}

template <typename Unit>
Position2D<Unit>::Position2D(double_t x, double_t y, Angle theta) : Vector2D<Unit>(x, y), theta(theta) {}

template <typename Unit>
Position2D<Unit>::Position2D(Vector2D<Unit> pos, Angle theta) : Vector2D<Unit>(pos), theta(theta) {}

template <typename Unit>
Vector2D<Unit> Position2D<Unit>::makeAbsolute(Vector2D<Unit> vector) const {
    double_t cos = std::cos(theta);
    double_t sin = std::sin(theta);

    return Vector2D<Unit>(vector.x * cos - vector.y * sin, vector.x * sin + vector.y * cos);
}

template <typename Unit>
Vector2D<Unit> Position2D<Unit>::makeRelative(Vector2D<Unit> vector) const {
    return Position2D<Unit>(this->x, this->y, -theta).makeAbsolute(vector);
}

template <typename Unit>
Position2D<Unit> Position2D<Unit>::relativeOffset(double_t xr, double_t yr) const {
    Vector2D<Unit> vector = makeAbsolute(Vector2D<Unit>(xr, yr));

    return Position2D<Unit>(this->x + vector.x, this->y + vector.y, theta);
}

template <typename Unit>
void Position2D<Unit>::operator+=(Position2D<Unit> pos) {
    Vector2D<Unit>::operator+=(pos);
    theta += pos.theta;
}
template <typename Unit>
void Position2D<Unit>::operator-=(Position2D<Unit> pos) {
    Vector2D<Unit>::operator-=(pos);
    theta -= pos.theta;
}
template <typename Unit>
void Position2D<Unit>::operator*=(double_t factor) {
    Vector2D<Unit>::operator*=(factor);
    theta *= factor;
}
template <typename Unit>
void Position2D<Unit>::operator/=(double_t factor) {
    Vector2D<Unit>::operator/=(factor);
    theta /= factor;
}

template <typename Unit>
Position2D<Unit> Position2D<Unit>::operator+(Position2D<Unit> pos) const {
    return Position2D(Vector2D<Unit>::operator+(pos), theta + pos.theta);
}
template <typename Unit>
Position2D<Unit> Position2D<Unit>::operator-(Position2D<Unit> pos) const {
    return Position2D(Vector2D<Unit>::operator-(pos), theta - pos.theta);
}
template <typename Unit>
Position2D<Unit> Position2D<Unit>::operator*(double_t factor) const {
    return Position2D(Vector2D<Unit>::operator*(factor), theta * factor);
}
template <typename Unit>
Position2D<Unit> Position2D<Unit>::operator/(double_t factor) const {
    return Position2D(Vector2D<Unit>::operator/(factor), theta / factor);
}

#define __SPECIALIZE(UNIT)                                                                                                                           \
    template class Position2D<UNIT>;                                                                                                                 \
                                                                                                                                                     \
    template <>                                                                                                                                      \
    Position2D<UNIT> clamp<Position2D<UNIT>>(Position2D<UNIT> value, double_t minBound, double_t maxBound) {                                         \
        return Position2D<UNIT>(clamp<Vector2D<UNIT>>(value, minBound, maxBound), value.theta);                                                      \
    }

__SPECIALIZE(Meter);
__SPECIALIZE(Millimeter);

#undef __SPECIALIZE
