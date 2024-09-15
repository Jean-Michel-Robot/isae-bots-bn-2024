
#include "state_machine/Ramp.hpp"
#include "state_machine/RampSM.hpp"
#include "logging.h"

Ramp::Ramp() {

    beginRampEvent.t0 = 0;
    goalSpeedChangeEvent.newSpeed = 0.0;
    updateEvent.currentTime = 0.0;

    // Don't put a logPrint in the constructor, it breaks the ramp behavior somehow

    RampSM::reset_and_start();
    // TEST_MESSAGE("Ramp started successfully");
}

void Ramp::beginRamp(uint32_t t0, float goalSpeed, float accelParam) {

    RampSM::setAccelParam(accelParam);

    // Verify that the Ramp SM is in IDLE
    if (RampSM::getCurrentState() != RampState::RAMP_IDLE)
    {
        //TODO error
        log(ERROR, "Tried to begin a ramp that is not in IDLE state");
        return;
    }

    // RampSM::setT0(t0);
    RampSM::setGoalSpeed(goalSpeed);

    // Start ramp by sending an updateEvent which sets t0 value
    beginRampEvent.t0 = t0;

    RampSM::dispatch(beginRampEvent);
}




float Ramp::updateRamp(uint32_t t) {

    // check if ramp is running
    //TODO


    // send UpdateEvent
    updateEvent.currentTime = t;
    RampSM::dispatch(updateEvent);

    // return output speed (that has been modified by the ramp SM)

    return RampSM::getCurrentSpeed();
}

void Ramp::endRamp() {
    RampSM::dispatch(endRampEvent);
}

bool Ramp::isRampIdle() {
    return (RampSM::getCurrentState() == RampState::RAMP_IDLE);
}

void Ramp::setToIdle() {
    SetRampToIdleEvent e;
    RampSM::dispatch(e);
}

void Ramp::changeGoalSpeed(float goalSpeed) {
    goalSpeedChangeEvent.newSpeed = goalSpeed;
    RampSM::dispatch(goalSpeedChangeEvent);
}

void Ramp::emergencyBrake() {
    RampSM::dispatch(emergencyBrakeEvent);
}