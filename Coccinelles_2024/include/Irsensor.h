#include <Wire.h>
#include <Arduino.h>
#include <SparkFun_VL53L5CX_Library.h>
#ifndef IRSENSOR_H
#define IRSENSOR_H

class Irsensor
{

private:
    int m_IR_PIN;
    long dt = 10;

public:
    SparkFun_VL53L5CX myImager;
    VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM
    volatile bool dataReady = false;      // Goes true when interrupt fires
    int imageResolution = 0;              // Used to pretty print output
    int imageWidth = 0;                   // Used to pretty print output

    // Constructor
    Irsensor(int Ir_PIN);

    int m_minimum_distance = 1000; // Distance en mm
    long m_time;

    void interruptRoutine();
    void setup();
    void loop();
};

#endif