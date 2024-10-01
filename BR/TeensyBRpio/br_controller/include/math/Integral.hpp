#ifndef _MATH_INTEGRAL_HPP_
#define _MATH_INTEGRAL_HPP_

#include "defines/math.hpp"

#include <optional>

/**
 * Estimates the integral of a function, using the rectangle method.
 */
template <typename TValue = double_t>
    requires Add<TValue> && Mul<TValue> && Default<TValue> && Clampable<TValue>
class Integral {
  public:
    Integral(std::optional<double_t> saturation = std::nullopt);
    /**
     * Updates the estimated integral.  If i(t) is the integral computed so far, then `i(t+interval) = i(t) + value * interval`.
     * @param value The current value of the function.
     * @param interval The time elapsed since the last call to update (or the creation of the estimator).
     */
    void update(TValue value, double_t interval);
    void reset();

    operator TValue() const;
    TValue value() const;

    std::optional<double_t> saturation() const;

  private:
    TValue m_value;
    std::optional<double_t> m_saturation;
};

#endif