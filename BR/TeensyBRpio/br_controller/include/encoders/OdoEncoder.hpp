#ifndef _ODO_ENCODER_HPP_
#define _ODO_ENCODER_HPP_

#include <concepts>
#include <cstdint>

/**
 * Encodes the state of the encoders into left and right tick counts.
 * This is designed for two-wheel robots.
 */
template <typename T>
concept OdoEncoder = requires(T a) {
    a.resetCounters();
    { a.getLeftCounter() } -> std::convertible_to<int32_t>;
    { a.getRightCounter() } -> std::convertible_to<int32_t>;
};

#endif