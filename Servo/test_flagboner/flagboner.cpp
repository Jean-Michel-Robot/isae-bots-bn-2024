/*
	Allows for the creation of flagboner objects,
	that is the system to rise the flag at the end
		( Savagely copied from tentacle.cpp )
*/

#include "flagboner.h"

// constructor of Tentacle class
Flagboner::Flagboner(int lowerSwitchPin, int higherSwitchPin, float switchTau,
		float switchThreshold, String servoId, int servoPin,
		int servoPositions[])

        : servo(servoId, servoPin, NB_POS, servoPositions),
        switchBuoy(switchPin, switchTau, switchThreshold) {
}
// getter for servo's currentPos
PositionId Flagboner::getServoCurrentPos() {
    return (PositionId) servo.getCurrentPos();	// Who is servo ?
}

// sets the position of the Flagboner among its servo
	// listed positions
void Flagboner::setPosition(PositionId positionId) {
    servo.setPosition((int) positionId);
}

// initializes the Flagboner and all of its components
void Flagboner::setup() {
    pinMode(servoPin, OUTPUT);
    pinMode(lower_switch, OUTPUT);
    pinMode(higher_switch, OUTPUT);

    servo.setup();
}

// updates the switch state
void Flagboner::loop() {
    switchBuoy.loop();
}