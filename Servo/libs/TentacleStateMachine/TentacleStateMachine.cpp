/*
  Allows for the creation of tentacle state machine objects
*/

#include "TentacleStateMachine.h"

// constructor of TentacleStateMachine class
TentacleStateMachine::TentacleStateMachine(String id, int pumpPin,
        int switchPin, float switchTau, float switchThreshold,
        String servoId, int servoPin, int servoPositions[],
        float timeInContact, float timeLostContact, float timeGrabbing,
        float timeLostContactBeforePumpOffReleasing,
        float timeBeforePumpOffReleasingFromMid, float timeReleasingFromMid,
        float timeBeforePumpOffReleasingFromHigh, float timeReleasingFromHigh)

        : tentacle(id, pumpPin,
            switchPin, switchTau, switchThreshold,
            servoId, servoPin, servoPositions),
        timerInContact(timeInContact),
        timerLostContact(timeLostContact),
        timerGrabbing(timeGrabbing),
        timerLostContactBeforePumpOffReleasing(timeLostContactBeforePumpOffReleasing),
        timerBeforePumpOffReleasing(timeBeforePumpOffReleasingFromMid),
        timerReleasing(timeReleasingFromMid) {

    this->timeBeforePumpOffReleasingFromMid = timeBeforePumpOffReleasingFromMid;
    this->timeReleasingFromMid = timeReleasingFromMid;

    this->timeBeforePumpOffReleasingFromHigh = timeBeforePumpOffReleasingFromHigh;
    this->timeReleasingFromHigh = timeReleasingFromHigh;

    this->currentState = STATE_SETUP;
}

// getter for currentState
State TentacleStateMachine::getCurrentState() {
    return currentState;
} 

// getter for tentacle's currentPos
PositionId TentacleStateMachine::getTentacleCurrentPos() {
    return tentacle.getServoCurrentPos();
}

// handles the actions associated with entering a state
void TentacleStateMachine::stateEnter(State state) {
    currentState = state;

    switch(state) {
        case STATE_SETUP:
            break;
        case STATE_IDLE:
            tentacle.switchPumpOnOrOff(LOW);
            tentacle.setPosition(POS_HIGH);
            break;
        case STATE_GRABBING:
            timerGrabbing.start(millis());
            tentacle.switchPumpOnOrOff(HIGH);
            tentacle.setPosition(POS_LOWEST);
            break;
        case STATE_CARRYING:
            timerBeforePumpOffReleasing.setLength(timeBeforePumpOffReleasingFromMid);
            timerReleasing.setLength(timeReleasingFromMid);
            tentacle.switchPumpOnOrOff(HIGH);
            tentacle.setPosition(POS_MID);
            break;
        case STATE_CARRYING_HIGH:
            timerBeforePumpOffReleasing.setLength(timeBeforePumpOffReleasingFromHigh);
            timerReleasing.setLength(timeReleasingFromHigh);
            tentacle.switchPumpOnOrOff(HIGH);
            tentacle.setPosition(POS_HIGH);
            break;
        case STATE_RELEASING:
            timerReleasing.start(millis());
            timerBeforePumpOffReleasing.start(millis());
            tentacle.setPosition(POS_LOW);
            break;
        case STATE_END:
            tentacle.switchPumpOnOrOff(LOW);
            break;
    }
}

// handles the actions associated with leaving a state
void TentacleStateMachine::stateExit() {
    switch(currentState) {
        case STATE_SETUP:
            break;
        case STATE_IDLE:
            break;
        case STATE_GRABBING:
            timerGrabbing.reset();
            timerInContact.reset();
            break;
        case STATE_CARRYING:
        case STATE_CARRYING_HIGH:
            timerLostContact.reset();
            break;
        case STATE_RELEASING:
            timerReleasing.reset();
            timerBeforePumpOffReleasing.reset();
            timerLostContactBeforePumpOffReleasing.reset();
            break;
        case STATE_END:
            break;
    }
}

// does the transition between current state and next state
void TentacleStateMachine::stateDoTransition(State nextState) {
    stateExit();
    stateEnter(nextState);
}

// check if a transition is due based on the currentState
void TentacleStateMachine::stateCheckTransition() {
    switch(currentState) {
        case STATE_SETUP:
            break;
        case STATE_IDLE:
            break;
        case STATE_GRABBING:
            if (timerInContact.isStarted() && timerInContact.isExpired(millis())) {
                stateDoTransition(STATE_CARRYING);
            }
            else if (!timerInContact.isStarted() && timerGrabbing.isExpired(millis())) {
                stateDoTransition(STATE_IDLE);
            }
            break;
        case STATE_CARRYING:
        case STATE_CARRYING_HIGH:
            if (timerLostContact.isStarted() && timerLostContact.isExpired(millis())) {
                stateDoTransition(STATE_IDLE);
            }
            break;
        case STATE_RELEASING:
            if (!timerBeforePumpOffReleasing.isExpired(millis())) {
                if (timerLostContactBeforePumpOffReleasing.isStarted() && timerLostContactBeforePumpOffReleasing.isExpired(millis())) {
                    // TODO: failure msg
                    Serial.print("currentState");
                    Serial.print(",");
                    Serial.print("tentacleCurrentPos");
                    Serial.print(",");
                    Serial.println("INVALID_DEPOSIT");
                    stateDoTransition(STATE_IDLE);
                }
            }
            else {
                if (!tentacle.isSwitchBuoyPressed()) {
                    // TODO: success msg
                    Serial.print("currentState");
                    Serial.print(",");
                    Serial.print("tentacleCurrentPos");
                    Serial.print(",");
                    Serial.println("SUCCESSFUL_DEPOSIT");
                    stateDoTransition(STATE_IDLE);
                }
                else if (timerReleasing.isExpired(millis())) {
                    stateDoTransition(STATE_CARRYING_HIGH);
                }
            }
            break;
        case STATE_END:
            break;
    }
}

// handles the actions associated with being in a state
void TentacleStateMachine::stateExecute() {
    switch(currentState) {
        case STATE_SETUP:
            break;
        case STATE_IDLE:
            break;
        case STATE_GRABBING:
            if (tentacle.isSwitchBuoyPressed()) {
                if (!timerInContact.isStarted()) {
                    timerInContact.start(millis());
                }
            }
            else if (timerInContact.isStarted()) {
                timerInContact.reset();
            }
            break;
        case STATE_CARRYING:
        case STATE_CARRYING_HIGH:
            if (!tentacle.isSwitchBuoyPressed()) {
                if (!timerLostContact.isStarted()) {
                    timerLostContact.start(millis());
                } 
            }
            else if (timerLostContact.isStarted()) {
                timerLostContact.reset();
            }
            break;
        case STATE_RELEASING:
            if (timerBeforePumpOffReleasing.isExpired(millis())) {
                tentacle.switchPumpOnOrOff(LOW);
            }
            else {
                if (!tentacle.isSwitchBuoyPressed()) {
                    if (!timerLostContactBeforePumpOffReleasing.isStarted()) {
                        timerLostContactBeforePumpOffReleasing.start(millis());
                    }
                }
                else if (timerLostContactBeforePumpOffReleasing.isStarted()) {
                    timerLostContactBeforePumpOffReleasing.reset();
                }
            }
            break;
        case STATE_END:
            break;
    }
}

void TentacleStateMachine::setup() {
    tentacle.setup();
    timerInContact.reset();

    timerLostContact.reset();
    
    timerGrabbing.reset();
    
    timerLostContactBeforePumpOffReleasing.reset();
    timerBeforePumpOffReleasing.reset();
    timerReleasing.reset();

    stateDoTransition(STATE_IDLE);
}

// ensures the state machine behaviour
void TentacleStateMachine::loop() {
    tentacle.loop();

    stateExecute();
    stateCheckTransition();
}
