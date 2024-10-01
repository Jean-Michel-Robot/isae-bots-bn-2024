#include "math/LowPassFilter.hpp"
#include <cmath>

#define TEMPLATE                                                                                                                                     \
    template <typename TValue>                                                                                                                       \
        requires Add<TValue> && Mul<TValue> && Default<TValue>

TEMPLATE
LowPassFilter<TValue>::LowPassFilter(double_t tau) : m_tau(tau), m_output() {}

TEMPLATE
void LowPassFilter<TValue>::update(TValue value, double_t interval) {
    if (!std::isnan(value) && !std::isinf(value)) {
        m_output = (value + (m_output * m_tau) / interval) / (1.0 + m_tau / interval); // we consider dS/dt = (U_(n+1) - U_n) / dt
    }
}

TEMPLATE
LowPassFilter<TValue>::operator TValue() const {
    return m_output;
}

TEMPLATE
TValue LowPassFilter<TValue>::value() const {
    return m_output;
}

template class LowPassFilter<double_t>;