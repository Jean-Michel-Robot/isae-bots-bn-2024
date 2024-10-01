#ifndef _CLOCK_HPP_
#define _CLOCK_HPP_

#include <concepts>
#include <limits>

using duration_t = unsigned long int;
using instant_t = unsigned long int;

template <typename T>
concept Clock = requires(T c) {
    { c.micros() } -> std::convertible_to<instant_t>;
};

inline duration_t getDurationMicros(instant_t before, instant_t now) {
    if (now > before) {
        return now - before;
    } else {
        // Overflow
        // (On Arduino, this happens after about 70 minutes according to the doc)
        return now + (std::numeric_limits<instant_t>::max() - before);
    }
}

#ifdef ARDUINO
#include <Arduino.h>

class SystemClock {
  public:
    SystemClock() = default;
    instant_t micros() const { return ::micros(); }
};

#else
#include <chrono>

class SystemClock {
  public:
    SystemClock() = default;
    instant_t micros() const {
        using namespace std::chrono;
        return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
    }
};

#endif
#endif