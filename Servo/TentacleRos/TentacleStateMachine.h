/*
  Allows for the creation of tentacle state machine objects
*/

#ifndef TENTACLESTATEMACHINE_H
#define TENTACLESTATEMACHINE_H

#include <Arduino.h>
#include "Tentacle.h"
#include "Timer.h"

enum State {
    STATE_IDLE = 0,
    STATE_GRABBING = 1,
    STATE_CARRYING = 2,
    STATE_CARRYING_HIGH = 3,
    STATE_RELEASING = 4,

    STATE_SETUP = 8,
    STATE_END = 9
};

class TentacleRos;

class TentacleStateMachine {
private:
    Tentacle tentacle; // tentacle associated to the state machine
    Timer timerInContact; // timer for the time that the tentacle needs to stay in contact with the buoy before considering that it is grabbed
    Timer timerLostContact; // timer for the time that the tentacle needs to have lost contact with the buoy before considering that it is lost
    
    Timer timerGrabbing; // timer for grabbing a buoy
    
    Timer timerLostContactBeforePumpOffReleasing; // timer to ensure that we do not lose the buoy before the pump switches off while releasing
    Timer timerBeforePumpOffReleasing; // timer to let the servo go all the way down before switching the pump off
    Timer timerReleasing; // timer for releasing a buoy

    float timeBeforePumpOffReleasingFromMid;
    float timeReleasingFromMid;

    float timeBeforePumpOffReleasingFromHigh;
    float timeReleasingFromHigh;

    State currentState; // current state the state machine is in

    // Ros communication
    int tentacleId;
    TentacleRos *tentacleRos;

public:
    TentacleStateMachine(String id, int pumpPin,
        int switchPin, float switchTau, float switchThreshold,
        String servoId, int servoPin, int servoPositions[],
        float timeInContact, float timeLostContact,
        float timeGrabbing,
        float timeLostContactBeforePumpOffReleasing,
        float timeBeforePumpOffReleasingFromMid, float timeReleasingFromMid,
        float timeBeforePumpOffReleasingFromHigh, float timeReleasingFromHigh,
        int tentacleId); // constructor of TentacleStateMachine class

    State getCurrentState(); // getter for currentState

    PositionId getTentacleCurrentPos(); // getter for tentacle's currentPos

    int getTentacleId(); // getter for the tantacleId

    void setTentacleRos(TentacleRos *tentacleRos); // setter for tentacleRos object

    void stateEnter(State state); // handles the actions associated with entering a state

    void stateExit(); // handles the actions associated with leaving a state

    void stateDoTransition(State nextState); // does the transition between current state and next state

    void stateCheckTransition(); // check if a transition is due based on the currentState

    void stateExecute(); // handles the actions associated with being in a state

    void setup();

    void loop(); // ensures the state machine behaviour
};

#endif