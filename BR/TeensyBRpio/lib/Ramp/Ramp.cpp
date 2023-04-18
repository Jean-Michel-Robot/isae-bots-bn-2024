
#include "Ramp.hpp"

// #include <ROS.hpp>
// #include "main_loop.hpp"


Ramp::Ramp(float accelParam = 0.0) {
    this->accelParam = accelParam;

    rampSM = RampSM();  // stack memory (instanciation without new keyword)


    // init accel param using set function
    rampSM.setAccelParam(accelParam);

    beginRampEvent.t0 = 0.0;
    goalSpeedChangeEvent.newSpeed = 0.0;
    updateEvent.currentTime = 0.0;

    rampSM.start();
}


void Ramp::beginRamp(uint32_t t0, float goalSpeed) {

    // Verify that the Ramp SM is in IDLE
    if (rampSM.getCurrentState() != RampState::RAMP_IDLE)
    {
        //TODO error
        Serial.println("ERROR : Tried to begin a ramp that is not in IDLE state");
        // p_ros->logPrint(LogType::ERROR, "Tried to begin a ramp that is not in IDLE state");
        return;
    }

    // rampSM.setT0(t0);
    rampSM.setGoalSpeed(goalSpeed);

    // Start ramp by sending an updateEvent which sets t0 value
    beginRampEvent.t0 = t0;

    rampSM.send_event(beginRampEvent);
}




float Ramp::updateRamp(uint32_t t) {

    // check if ramp is running
    //TODO

    // send UpdateEvent
    updateEvent.currentTime = t;
    rampSM.send_event(updateEvent);

    // return output speed (that has been modified by the ramp SM)

    return rampSM.getCurrentSpeed();
}

void Ramp::endRamp() {
    rampSM.send_event(endRampEvent);
}


void Ramp::changeGoalSpeed(float goalSpeed) {
    goalSpeedChangeEvent.newSpeed = goalSpeed;
    rampSM.send_event(goalSpeedChangeEvent);
}

void Ramp::emergencyBrake() {
    rampSM.send_event(emergencyBrakeEvent);
}