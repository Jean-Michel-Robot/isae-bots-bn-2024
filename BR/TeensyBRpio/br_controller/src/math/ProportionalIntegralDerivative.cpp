#include "math/ProportionalIntegralDerivative.hpp"

#define TEMPLATE                                                                                                                                     \
    template <typename TValue>                                                                                                                       \
        requires Add<TValue> && Mul<TValue> && Default<TValue> && Clampable<TValue>

TEMPLATE
ProportionalIntegralDerivative<TValue>::ProportionalIntegralDerivative(double_t kp, double_t ti, double_t td, double_t filter,
                                                                       std::optional<double_t> outputSaturation,
                                                                       std::optional<double_t> integralSaturation,
                                                                       std::optional<double_t> derivativeSaturation)
    : m_kp(kp), m_ki(ti == 0 ? 0 : 1 / ti), m_value(), m_integral(integralSaturation), m_derivative(td, filter, {}, derivativeSaturation),
      m_saturation(outputSaturation) {}

TEMPLATE
void ProportionalIntegralDerivative<TValue>::update(TValue error, double_t interval) {
    m_derivative.update(error, interval);
    m_integral.update(error, interval);

    m_value = m_kp * (error + m_ki * m_integral.value() + m_derivative.value());
    if (m_saturation) {
        m_value = clamp<TValue>(m_value, -*m_saturation, *m_saturation);
    }

#ifdef _DEBUG
    m_lastError = error;
#endif
}

TEMPLATE
void ProportionalIntegralDerivative<TValue>::reset() {
    m_integral.reset();
    m_derivative.reset();
    m_value = TValue();
}

TEMPLATE
ProportionalIntegralDerivative<TValue>::operator TValue() const {
    return m_value;
}
TEMPLATE
TValue ProportionalIntegralDerivative<TValue>::value() const {
    return m_value;
}

#ifdef _DEBUG
TEMPLATE
TValue ProportionalIntegralDerivative<TValue>::lastError() const {
    return m_lastError;
}
#endif

TEMPLATE
double_t ProportionalIntegralDerivative<TValue>::kp() const {
    return m_kp;
}
TEMPLATE
double_t ProportionalIntegralDerivative<TValue>::ti() const {
    return m_ki == 0 ? 0 : 1 / m_ki;
}
TEMPLATE
double_t ProportionalIntegralDerivative<TValue>::td() const {
    return m_derivative.gain();
}
TEMPLATE
double_t ProportionalIntegralDerivative<TValue>::filter() const {
    return m_derivative.filter();
}

TEMPLATE
std::optional<double_t> ProportionalIntegralDerivative<TValue>::saturation() const {
    return m_saturation;
}
TEMPLATE
std::optional<double_t> ProportionalIntegralDerivative<TValue>::integralSaturation() const {
    return m_integral.saturation();
}
TEMPLATE
std::optional<double_t> ProportionalIntegralDerivative<TValue>::derivativeSaturation() const {
    return m_derivative.saturation();
}

TEMPLATE
const Integral<TValue> &ProportionalIntegralDerivative<TValue>::getIntegral() const {
    return m_integral;
}
TEMPLATE
const Derivative<TValue> &ProportionalIntegralDerivative<TValue>::getDerivative() const {
    return m_derivative;
}

#include "geometry/Vector2D.hpp"
template class ProportionalIntegralDerivative<double_t>;
template class ProportionalIntegralDerivative<Vector2D<Meter>>;