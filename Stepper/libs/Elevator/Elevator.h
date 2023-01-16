/*
  Allows for the creation of elevator objects, that is 4 elevator arms and a stepper
*/

#ifndef ELEVATOR_H
#define ELEVATOR_H

#include <Arduino.h>
#include "ElevatorArm.h"
#include "StepperKamasutra.h"

#define NB_ARMS 4

enum PositionId
{
    POS_HIGH = 0,
    POS_MID = 1,
    POS_LOW = 2
};

class Elevator
{
private:
    String m_id; // id of the elevator

    ElevatorArm *m_arms[NB_ARMS]; // array of elevator arms
    StepperKamasutra m_stepper;    // stepper of the elevator

public:
    Elevator(String id, float switchTau, float switchThreshold,
             String armIds[NB_ARMS], int pumpPins[NB_ARMS], int switchPins[NB_ARMS],
             String stepperId, int stepPin, int dirPin, int maxSpeed, int acceleration,
             int nbPos, int *positions); // constructor of Elevator class

    bool isSwitchBuoyPressed(int armId); // returns the state of the filtered switch of the given arm

    void switchPump(int armId, bool state); // switch the pump of the given arm

    void setPosition(PositionId positionId); // sets the position of the elevator

    void setup(); // initializes the Elevator and all of its components

    void loop();
};

#endif