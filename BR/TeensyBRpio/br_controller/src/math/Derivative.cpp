#include "math/Derivative.hpp"

#define TEMPLATE                                                                                                                                     \
    template <typename TValue>                                                                                                                       \
        requires Add<TValue> && Mul<TValue> && Default<TValue> && Clampable<TValue>

TEMPLATE
Derivative<TValue>::Derivative(double_t gain, double_t filter, TValue initialValue, std::optional<double_t> saturation)
    : m_gain(gain), m_filter(filter), m_lastValue(initialValue), m_lastDerivative(), m_saturation(saturation) {}

TEMPLATE
void Derivative<TValue>::update(TValue value, double_t interval) {
    TValue derivative = m_gain / (interval + m_filter * m_gain) * (m_lastDerivative * m_filter + value - m_lastValue);
    m_lastDerivative = derivative;
    if (m_saturation) {
        m_lastDerivative = clamp<TValue>(m_lastDerivative, -*m_saturation, *m_saturation);
    }
    m_lastValue = value;
}

TEMPLATE
void Derivative<TValue>::reset(TValue initialValue) {
    m_lastValue = initialValue;
    m_lastDerivative = TValue();
}

TEMPLATE
Derivative<TValue>::operator TValue() const {
    return m_lastDerivative;
}

TEMPLATE
TValue Derivative<TValue>::value() const {
    return m_lastDerivative;
}

TEMPLATE
double_t Derivative<TValue>::gain() const {
    return m_gain;
}

TEMPLATE
double_t Derivative<TValue>::filter() const {
    return m_filter;
}

TEMPLATE
std::optional<double_t> Derivative<TValue>::saturation() const {
    return m_saturation;
}

#include "geometry/Position2D.hpp"
template class Derivative<double_t>;
template class Derivative<Position2D<Meter>>;
template class Derivative<Vector2D<Meter>>;