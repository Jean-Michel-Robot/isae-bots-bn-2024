#ifndef _LOW_PASS_FILTER_HPP_
#define _LOW_PASS_FILTER_HPP_

#include "defines/math.hpp"

/**
 * First-order low pass fiter.
 */
template <typename TValue = double_t>
    requires Add<TValue> && Mul<TValue> && Default<TValue>
class LowPassFilter {
  public:
    LowPassFilter(double_t tau);

    /**
     * Updates the output of the filter
     * @param value The current value of the function.
     * @param interval The time elapsed since the last call to update. Must be strictly positive.
     */
    void update(TValue input, double_t interval);

    TValue value() const;
    operator TValue() const;

  private:
    double_t m_tau;  // time constant, in s
    TValue m_output; // last output that has been computed
};

#endif
