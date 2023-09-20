/*
  Allows for the creation of a filtered switch object, that is a switch which output signal went through a low pass filter
*/

#ifndef SWITCHFILTERED_H
#define SWITCHFILTERED_H

#include <Arduino.h>
#include "FilterLowPass.h"

class SwitchFiltered {
private:
    int pin; // pin where the switch is connected
    FilterLowPass filter; // low pass filter which the switch output signal will go through
    float threshold; // threshold above which the switch will be considered pushed
    bool activeHigh; // whether the switch pin is HIGH or LOW when pressed

public:
    SwitchFiltered(int pin, float tau, float threshold, bool activeHigh); // constructor of FilteredSwitch class

    float getFilterOutput(); // returns the filter output value, only for testing purposes

    bool isSwitchPressed(); // returns wether or not the output value of the filter is above/under threshold

    void setup();

    void loop();

};

#endif