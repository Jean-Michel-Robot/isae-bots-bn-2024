#ifndef _MATH_PID_HPP_
#define _MATH_PID_HPP_

#include "configuration.hpp"

#include "defines/math.hpp"
#include "math/Derivative.hpp"
#include "math/Integral.hpp"

#include <optional>

/**
 * Applies a "standard" Proportional-Integral-Derivative (PID) control to an error.
 * See https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller and http://www.bedwani.ch/regul/discret/top/pidf.htm
 *
 * This class computes:
 * `u(t) = Kp * (e(t) + (1 / Ti) * Integral(e(t)dt) + D(e, t))`
 *
 * where `e` is the error and `D(e, t)` is the filtered derivative of `e`. See class `Derivative` for more details.
 */
template <typename TValue = double_t>
    requires Add<TValue> && Mul<TValue> && Default<TValue> && Clampable<TValue>
class ProportionalIntegralDerivative {
  public:
    /**
     * @param kp The proportional gain (positive or null)
     * @param ti = kp/ki Integration time (positive or null)
     * @param td = kd/kp Derivative time (positive or null)
     * @param filter Weight of the low-pass filter for the derivative. See class Derivative.
     */
    ProportionalIntegralDerivative(double_t kp, double_t ti, double_t td, double_t filter = 0,
                                   std::optional<double_t> outputSaturation = std::nullopt, std::optional<double_t> integralSaturation = std::nullopt,
                                   std::optional<double_t> derivativeSaturation = std::nullopt);

    ProportionalIntegralDerivative()
        : ProportionalIntegralDerivative(DEFAULT_KP, DEFAULT_TI, DEFAULT_TD, DERIVATIVE_FILTER, PID_SATURATION, INTEGRAL_SATURATION,
                                         DERIVATIVE_SATURATION) {}

    /**
     * Updates the output of the PID.
     * @param value The current value of the error.
     * @param interval The time elapsed since the last call to update. Must be strictly positive.
     */
    void update(TValue error, double_t interval);
    void reset();

    operator TValue() const;
    TValue value() const;

#ifdef _DEBUG
    TValue lastError() const;
#endif

    double_t kp() const;
    double_t ti() const;
    double_t td() const;
    double_t filter() const;

    std::optional<double_t> saturation() const;
    std::optional<double_t> integralSaturation() const;
    std::optional<double_t> derivativeSaturation() const;

    const Integral<TValue> &getIntegral() const;
    const Derivative<TValue> &getDerivative() const;

  private:
    double_t m_kp;
    double_t m_ki;

    TValue m_value;
#ifdef _DEBUG
    TValue m_lastError = {};
#endif

    Integral<TValue> m_integral;
    Derivative<TValue> m_derivative;
    std::optional<double_t> m_saturation;
};

#endif