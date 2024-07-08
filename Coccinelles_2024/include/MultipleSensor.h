#ifndef MULTIPLESENSOR_H
#define MULTIPLESENSOR_H

#include <Wire.h>
#include <Arduino.h>
#include <SparkFun_VL53L5CX_Library.h>

class MultipleSensor
{

private:
    int m_r_pin;
    int m_l_pin;
    int m_center_pin;

public:
    void setup();
    void loop();
    MultipleSensor(int r_pin, int l_pin, int center_pin);
    SparkFun_VL53L5CX myImager;
    VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM
    volatile bool dataReady = false;      // Goes true when interrupt fires
    int imageResolution = 0;              // Used to pretty print output
    int imageWidth = 0;
};

#endif