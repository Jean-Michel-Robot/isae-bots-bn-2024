#ifndef _DEFINE_MATH_HPP_
#define _DEFINE_MATH_HPP_

#include "defines/constraint.hpp"

#include <algorithm>
#include <concepts>

#include <math.h> // type double_t

template <typename T>
concept Add = requires(T a) {
    { a + a } -> std::convertible_to<T>;
    { a - a } -> std::convertible_to<T>;
};
template <typename TValue>
concept Mul = requires(double_t a, TValue b) {
    { b *a } -> std::convertible_to<TValue>;
};

/* #region convert */

/** A phantom marker that specifies that lengths are expressed in meters. */
class Meter {
  public:
    /** Converts `value` from meters to `Unit` */
    template <typename Unit>
    static double_t convert(double_t value);
};

/** A phantom marker that specifies that lengths are expressed in millimeters. */
class Millimeter {
  public:
    /** Converts `value` from millimeters to `Unit` */
    template <typename Unit>
    static double_t convert(double_t value);
};

/** No-op */
template <>
inline double_t Meter::convert<Meter>(double_t value) {
    return value;
}
/** Converts `value` from meters to millimeters */
template <>
inline double_t Meter::convert<Millimeter>(double_t value) {
    return value * 1000.0;
}

/** No-op */
template <>
inline double_t Millimeter::convert<Millimeter>(double_t value) {
    return value;
}
/** Converts `value` from millimeters to meters */
template <>
inline double_t Millimeter::convert<Meter>(double_t value) {
    return value / 1000.0;
}

template <typename From, typename To>
concept Convertible = requires(double_t a) {
    { From::template convert<To>(a) } -> std::convertible_to<double_t>;
};

/* #endregion */

/// @return 0 if val == 0, 1 if val > 0, -1 if val < 0
template <typename T>
int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

/* #region clamp */

/// Coordinate-wise clamping. See also std::clamp.
template <typename T>
T clamp(T value, double_t minBound, double_t maxBound);

template <typename T>
concept StdClampable = requires(T t, double_t b) {
    { std::clamp(t, b, b) } -> std::convertible_to<T>;
};

template <StdClampable T>
inline T clamp(T value, double_t minBound, double_t maxBound) {
    return std::clamp(value, minBound, maxBound);
}

template <typename T>
concept Clampable = requires(T t, double_t b) {
    { clamp<T>(t, -b, b) } -> std::convertible_to<T>;
};

/* #endregion */

#endif