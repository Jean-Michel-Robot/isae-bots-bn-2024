/*
    Allows for the creation of flagboner objects,
    that is the system to rise the flag at the end
        ( Savagely copied from tentacle.h )
*/

#ifndef FLAGBONER_H
#define FLAGBONER_H

// #include <Arduino.h>
#include "ServoKamasutra.h"

#define NB_POS 2

enum PositionId {
    POS_HIGH = 0,   // What value should be used ?
    POS_LOW = 2,    // ??
};

class Flagboner {
private:
    SwitchFiltered switchBuoy; // switch to detect presence of a buoy
    ServoKamasutra servo; // servo that moves the arm

public:
    Flagboner::Flagboner(int lowerSwitchPin, int higherSwitchPin, float switchTau,
        float switchThreshold, String servoId, int servoPin,
        int servoPositions[]); // constructor of flagboner classf

    PositionId getServoCurrentPos(); // getter for servo's currentPos
    
    void setPosition(PositionId positionId); // sets the position of the flagboner among its servo listed positions

    void setup(); // initializes the flagboner

    void loop(); // updates the switch state
};

#endif
