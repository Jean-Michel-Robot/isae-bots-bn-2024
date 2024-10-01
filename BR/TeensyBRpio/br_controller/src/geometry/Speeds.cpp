#include "geometry/Speeds.hpp"

Speeds::Speeds(double_t linear, double_t angular) : linear(linear), angular(angular) {}

Speeds Speeds::operator+(Speeds other) const {
    return Speeds(linear + other.linear, angular + other.angular);
}
Speeds Speeds::operator-(Speeds other) const {
    return Speeds(linear - other.linear, angular - other.angular);
}
Speeds Speeds::operator*(double_t factor) const {
    return Speeds(linear * factor, angular * factor);
}
Speeds Speeds::operator/(double_t factor) const {
    return Speeds(linear / factor, angular / factor);
}