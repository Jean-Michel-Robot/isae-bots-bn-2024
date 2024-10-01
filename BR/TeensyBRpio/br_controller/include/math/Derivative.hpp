#ifndef _MATH_DERIVATIVE_HPP_
#define _MATH_DERIVATIVE_HPP_

#include "defines/math.hpp"

#include <optional>

/**
 * Estimates the instantaneous derivative of a function. The estimator offers the possibility to add a low-pass filter to the derivative.
 * This class estimates the solution of the equation:
 *
 * `v(t) + K * alpha * (dv/dt)(t) = K (df/dt)(t)`
 *
 * where v is the output, f is the input, K is the gain and alpha is the weight of the filter.
 * The gain and the filter should be positive.
 *
 * When `gain = 1` and `alpha = 0`, the output is actually the estimated derivative of `f`.
 *
 * See http://www.bedwani.ch/regul/discret/top/df.htm
 */
template <typename TValue = double_t>
    requires Add<TValue> && Mul<TValue> && Default<TValue> && Clampable<TValue>
class Derivative {
  public:
    /**
     * @param gain,filter See class documentation. gain must be strictly positive. filter must be positive or zero.
     * @param initialValue Sets f(0) in case it is not zero. (see class documentation).
     */
    Derivative(double_t gain = 1, double_t filter = 0, TValue initialValue = {}, std::optional<double_t> saturation = std::nullopt);

    /**
     * Updates the estimation of the derivative.
     * @param value The current value of the function.
     * @param interval The time elapsed since the last call to update (or the creation of the estimator). Must be strictly positive.
     */
    void update(TValue value, double_t interval);
    /**
     * Resets the estimator to its initial state.
     * @param initialValue Sets f(0) in case it is not zero. Note that if this parameter is omitted, then f(0) is set to 0. The value of
     * `initialValue` used in the constructor is not used.
     */
    void reset(TValue initialValue = {});

    operator TValue() const;
    TValue value() const;

    double_t gain() const;
    double_t filter() const;
    std::optional<double_t> saturation() const;

  private:
    double_t m_gain;
    double_t m_filter;
    TValue m_lastValue;
    TValue m_lastDerivative;
    std::optional<double_t> m_saturation;
};

#endif