/*
Allows for the creation of a first order low-pass filter object
*/

#include "utils/FilterLowPass.h"
#include "utils/math.h"

// constructor of FilterLowPass class
FilterLowPass::FilterLowPass(float tau) {
    m_tau = tau;
}

// returns the last output computed
float FilterLowPass::getOutput() const{
    return m_output;
}

// computes new output given a new input
float FilterLowPass::computeOutput(float input, unsigned long currentTimeMicro) {
    float deltaT = (float) (currentTimeMicro - m_lastTimeMicro)* 1e-6;
    
    if (!isnan(input) && !isinf(input) && deltaT > 0.0) {
        m_lastTimeMicro = currentTimeMicro;
        m_output = (input + (m_output * m_tau) / deltaT) / (1.0 + m_tau / deltaT); // we consider dS/dt = (U_(n+1) - U_n) / dt
    }

    return m_output;
}
