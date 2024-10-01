#include "geometry/Vector2D.hpp"

template <typename Unit>
Vector2D<Unit>::Vector2D() : Vector2D(0, 0) {}
template <typename Unit>
Vector2D<Unit>::Vector2D(double_t x, double_t y) : x(x), y(y) {}

template <typename Unit>
double_t Vector2D<Unit>::norm() const {
    return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
}

template <typename Unit>
Angle Vector2D<Unit>::argument() const {
    if (x == 0 && y == 0) {
        return 0;
    }
    return atan2(y, x);
}

template <typename Unit>
Vector2D<Unit> Vector2D<Unit>::normalize() const {
    if (x == 0 && y == 0) {
        return *this;
    }
    return *this / norm();
}

template <typename Unit>
void Vector2D<Unit>::operator+=(Vector2D<Unit> pos) {
    x += pos.x;
    y += pos.y;
}
template <typename Unit>
void Vector2D<Unit>::operator-=(Vector2D<Unit> pos) {
    x -= pos.x;
    y -= pos.y;
}
template <typename Unit>
void Vector2D<Unit>::operator*=(double_t factor) {
    x *= factor;
    y *= factor;
}
template <typename Unit>
void Vector2D<Unit>::operator/=(double_t factor) {
    x /= factor;
    y /= factor;
}

template <typename Unit>
Vector2D<Unit> Vector2D<Unit>::operator+(Vector2D<Unit> pos) const {
    return Vector2D(x + pos.x, y + pos.y);
}
template <typename Unit>
Vector2D<Unit> Vector2D<Unit>::operator-(Vector2D<Unit> pos) const {
    return Vector2D(x - pos.x, y - pos.y);
}
template <typename Unit>
Vector2D<Unit> Vector2D<Unit>::operator*(double_t factor) const {
    return Vector2D(x * factor, y * factor);
}
template <typename Unit>
Vector2D<Unit> Vector2D<Unit>::operator/(double_t factor) const {
    return Vector2D(x / factor, y / factor);
}

template <typename Unit>
double_t Vector2D<Unit>::distance(Vector2D<Unit> a, Vector2D<Unit> b) {
    return (b - a).norm();
}
template <typename Unit>
Angle Vector2D<Unit>::angle(Vector2D<Unit> a, Vector2D<Unit> b) {
    double_t prod = dot(a, b);
    double_t det = a.x * b.y - a.y * b.x; // Determinant
    if (det == 0 && dot == 0) {
        return 0;
    }
    return atan2(det, prod);
}
template <typename Unit>
double_t Vector2D<Unit>::dot(Vector2D<Unit> a, Vector2D<Unit> b) {
    return a.x * b.x + a.y * b.y;
}

#define __SPECIALIZE(UNIT)                                                                                                                           \
    template class Vector2D<UNIT>;                                                                                                                   \
                                                                                                                                                     \
    template <>                                                                                                                                      \
    Vector2D<UNIT> clamp<Vector2D<UNIT>>(Vector2D<UNIT> value, double_t minBound, double_t maxBound) {                                               \
        return Vector2D<UNIT>(std::clamp(value.x, minBound, maxBound), std::clamp(value.y, minBound, maxBound));                                     \
    }

__SPECIALIZE(Meter);
__SPECIALIZE(Millimeter);

#undef __SPECIALIZE
