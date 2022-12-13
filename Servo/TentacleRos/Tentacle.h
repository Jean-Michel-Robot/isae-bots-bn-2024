/*
  Allows for the creation of tentacle objects, that is the arms of the robot, with a servo, a pump,and a switch
*/

#ifndef TENTACLE_H
#define TENTACLE_H

#include <Arduino.h>
#include "ServoKamasutra.h"
#include "SwitchFiltered.h"

#define NB_POS 4

enum PositionId {
    POS_HIGH = 0,
    POS_MID = 1,
    POS_LOW = 2,
    POS_LOWEST = 3
};

class Tentacle {
private:
    String id; // id of the tentacle
    int pumpPin; // pin where the pump is connected

    SwitchFiltered switchBuoy; // switch to detect presence of a buoy
    ServoKamasutra servo; // servo that moves the arm

public:
    Tentacle(String id, int pumpPin,
        int switchPin, float switchTau, float switchThreshold,
        String servoId, int servoPin, int servoPositions[]); // constructor of Tentacle class

    float getSwitchBuoyFilterOutput(); // returns the switch buoy filter output value, only for testing purposes

    bool isSwitchBuoyPressed(); // reads the filtered bumper state in order to tell wether or not a buoy is present

    void switchPumpOnOrOff(bool state); // switch the pump on or off : true -> on, false -> off

    PositionId getServoCurrentPos(); // getter for servo's currentPos
    
    void setPosition(PositionId positionId); // sets the position of the tentacle among its servo listed positions

    void setup(); // initializes the Tentacle and all of its components

    void loop(); // updates the switch state
};

#endif
