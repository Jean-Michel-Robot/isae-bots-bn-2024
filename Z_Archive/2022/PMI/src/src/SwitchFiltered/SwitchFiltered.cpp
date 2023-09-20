/*
  Allows for the creation of a filtered switch object, that is a switch which output signal went through a low pass filter
*/

#include "SwitchFiltered.h"

// constructor of FilteredSwitch
SwitchFiltered::SwitchFiltered(int pin, float tau, float threshold, bool activeHigh)
        : filter(tau) {
    this->pin = pin;
    this->threshold = threshold;
    this->activeHigh = activeHigh;
}

// returns the filter output value, only for testing purposes
float SwitchFiltered::getFilterOutput() {
    return filter.getOutput();
}

// returns wether or not the output value of the filter is above/under threshold
bool SwitchFiltered::isSwitchPressed() {
    return filter.getOutput() > threshold; // switch value is 1 when pressed, 0 else
}

void SwitchFiltered::setup() {
    pinMode(pin, INPUT); // PULLUP/PULLDOWN ?
}

void SwitchFiltered::loop() {
    if (digitalRead(pin) == activeHigh) {
        filter.computeOutput(1.0, micros()); // to be changed iaw circuit
    }
    else {
        filter.computeOutput(0.0, micros()); // to be changed iaw circuit
    }
    
}