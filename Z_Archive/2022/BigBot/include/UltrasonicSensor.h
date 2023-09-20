/*
  Allows for the creation of ultrasonic sensor objects
  Designed for UNDK 30U6103/S14 sensor
*/

#ifndef ULTRASONICSENSOR_h
#define ULTRASONICSENSOR_H

#include <Arduino.h>

class UltrasonicSensor
{
  private:
    String id; // id of the sensor
    int pin; // pin from which the analog values will be read
    float base; // 0 corresponds to this value
    float coeff; // mm^-1, ie analog value / coeff = value in mm
    float timeBeforeRefresh; // s, time before refreshing the value

    float distance; // mm, last measured distance

  public:
    UltrasonicSensor(String id, int pin, float base, float coeff, float timeBeforeRefresh); // constructor of UltrasonicSensor class

    float getDistance(); // getter for distance variable

    void updateDistance(); // updates distance with the current measured distance

    void setup();

    void loop(); // updates the measured distance once every timeBeforeRefresh
};

#endif