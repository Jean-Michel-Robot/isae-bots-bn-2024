#include "geometry/Angle.hpp"
#include <cmath>
#include <numbers>

constexpr double_t PI = std::numbers::pi_v<double_t>;

/// Take an angle in [-2*PI;2*PI] and return the equivalent angle in (-PI;PI]
/// The result is unspecified if `value` is not within [-2*PI;2*PI].
double_t modulo_x2(double_t value) {
    return (value > PI ? value - 2 * PI : (value <= -PI ? value + 2 * PI : value));
}

/// Take any angle and return the equivalent angle in (-PI;PI]
double_t modulo_pipi(double_t value) {
    return modulo_x2(fmod(value, 2 * PI));
}

Angle::Angle(double_t value) : m_value(modulo_pipi(value)) {}

Angle::operator double_t() {
    return m_value;
}
double_t Angle::value() const {
    return m_value;
}

Angle Angle::reverse() const {
    return *this + PI;
}

Angle Angle::operator-() const {
    return Angle(-m_value);
}

void Angle::operator+=(Angle other) {
    // Cannot do m_value += other because condition `m_value in (-PI;PI]` would no longer hold
    *this = *this + other;
}
void Angle::operator-=(Angle other) {
    *this = *this - other;
}
void Angle::operator*=(double_t factor) {
    *this = *this * factor;
}
void Angle::operator/=(double_t factor) {
    *this = *this / factor;
}

Angle Angle::operator+(Angle other) const {
    return Angle(m_value + other.m_value);
}
Angle Angle::operator-(Angle other) const {
    return Angle(m_value - other.m_value);
}
Angle Angle::operator*(double_t factor) const {
    return Angle(m_value * factor);
}
Angle Angle::operator/(double_t factor) const {
    return Angle(m_value / factor);
}