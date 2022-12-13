/*
  Allows for the creation of tentacle objects, that is the arms of the robot, with a servo, a pump, and a switch
*/

#include "Tentacle.h"

// constructor of Tentacle class
Tentacle::Tentacle(String id, int pumpPin,
        int switchPin, float switchTau, float switchThreshold,
        String servoId, int servoPin, int servoPositions[])

        : servo(servoId, servoPin, NB_POS, servoPositions),
        switchBuoy(switchPin, switchTau, switchThreshold) {
    this->id = id;
    this->pumpPin = pumpPin;
}

// returns the switch buoy filter output value, only for testing purposes
float Tentacle::getSwitchBuoyFilterOutput() {
    return switchBuoy.getFilterOutput();
}

// reads the filtered bumper state in order to tell wether or not a buoy is present
bool Tentacle::isSwitchBuoyPressed() {
    return switchBuoy.isSwitchPressed();
}

// switch the pump on or off : true -> on, false -> off
void Tentacle::switchPumpOnOrOff(bool state) {
    digitalWrite(pumpPin, state);
}

// getter for servo's currentPos
PositionId Tentacle::getServoCurrentPos() {
    return (PositionId) servo.getCurrentPos();
}

// sets the position of the tentacle among its servo listed positions
void Tentacle::setPosition(PositionId positionId) {
    servo.setPosition((int) positionId);
}

// initializes the Tentacle and all of its components
void Tentacle::setup() {
    pinMode(pumpPin, OUTPUT);
    switchPumpOnOrOff(LOW);

    servo.setup();
    switchBuoy.setup();
}

// updates the switch state
void Tentacle::loop() {
    switchBuoy.loop();
}