/*
  Allows for the creation of a first order low-pass filter object
*/

#ifndef FILTERLOWPASS_H
#define FILTERLOWPASS_H
#ifdef __linux
#include <math.h> // for testing purposes on the computer
#elif not defined(__SIMU__)
#include <Arduino.h> // compiled with arduino IDE
#endif

class FilterLowPass {
    private:
        float m_tau = 0.0; // time constant, in s
        unsigned long m_lastTimeMicro = 0; // last time when output as been computed, in us
        float m_output = 0.0; // last output that has been computed

    public:
        FilterLowPass(float tau); // constructor of FilterLowPass class

        float getOutput(); // returns the last output computed

        float computeOutput(float input, unsigned long currentTimeMicro); // computes new output given a new input

};

#endif
