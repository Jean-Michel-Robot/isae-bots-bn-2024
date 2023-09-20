/*
  Allows for the creation of ultrasonic sensor objects
  Designed for UNDK 30U6103/S14 sensor
*/

#include "UltrasonicSensor.h"

// constructor of UltrasonicSensor class
UltrasonicSensor::UltrasonicSensor(String id, int pin, float base, float coeff, float timeBeforeRefresh)
{
    this->id = id;
    this->pin = pin;
    this->base = base;
    this->coeff = coeff;
    this->timeBeforeRefresh = timeBeforeRefresh;

    this->distance = 0;
}

// getter for distance variable
float UltrasonicSensor::getDistance()
{
    return distance;
}

// updates distance with the current measured distance
void UltrasonicSensor::updateDistance()
{
    distance = base + ((float)analogRead(pin)) / coeff;
}

void UltrasonicSensor::setup()
{
    pinMode(pin, INPUT);
    updateDistance();
}

// updates the measured distance once every timeBeforeRefresh
void UltrasonicSensor::loop()
{
    static unsigned long lastTimeRefresh = millis();

    if (millis() - lastTimeRefresh > timeBeforeRefresh * 1e3)
    {
        lastTimeRefresh = millis();
        updateDistance();
    }
}
