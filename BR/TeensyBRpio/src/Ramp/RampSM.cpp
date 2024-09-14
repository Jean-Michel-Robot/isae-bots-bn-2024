
#include "Ramp/RampSM.hpp"

#include <Arduino.h>

#include "ROS.hpp"
#include "main_loop.hpp"

#include <RampDefines.hpp>
#include <defines.hpp>


// define string names for states
static const char *RampStateStr[] = {
    FOREACH_RAMPSTATE(GENERATE_STRING)
};

// Constructor
void RampSM::reset_and_start() {
  // init state var
  currentState = RampState::RAMP_IDLE;

  // this->t_current = 0.0;
  t_start_slope = 0.0; t0 = 0.0;
  V_start_slope = 0.0;
  currentSpeed = 0.0;
  accelParam = 0.0;

  start();
}

void RampSM::setAccelParam(float accelParam) {
  RampSM::accelParam = accelParam;
}

void RampSM::setGoalSpeed(float goalSpeed) {
  RampSM::goalSpeed = goalSpeed;
}


// void RampSM::setT0(float t0) {
//     this->t0 = t0;
// }

RampState RampSM::getCurrentState() {
  return currentState;
}

String RampSM::getCurrentStateStr() {
  return RampStateStr[currentState];
}


float RampSM::getCurrentSpeed() {
  return currentSpeed;
}


// Forward declarations
class Constant;
class RampEnd;
class Brake;




// ----------------------------------------------------------------------------
// Transition functions
//






// ----------------------------------------------------------------------------
// State: Slope
//
class Slope
: public RampSM
{
  void entry() override {
    currentState = RampState::RAMP_SLOPE;

    t_start_slope = micros();
    V_start_slope = currentSpeed;
  }

  void react(UpdateEvent const & e) override {

    // float dt = e.currentTime - t_current;
    // t_current = e.currentTime;

    if (e.currentTime - t_start_slope < 0) {
      p_ros->logPrint(ERROR, "Ecart de temps negatif"); //TOTEST pas de valeurs négatives
    }

    // update currentSpeed (no need for previous currentSpeed)

    if (V_start_slope < goalSpeed - RAMP_EPSILON) {
      currentSpeed = V_start_slope + accelParam * (e.currentTime - t_start_slope)*0.000001;  // en ASC
    
      if (currentSpeed > goalSpeed - RAMP_EPSILON) {
        // p_ros->logPrint(INFO, "Reached constant part of upwards slope");
        currentSpeed = goalSpeed;
        transit<Constant>();
      }
    }

    else if (V_start_slope > goalSpeed + RAMP_EPSILON) {
      currentSpeed = V_start_slope - accelParam * (e.currentTime - t_start_slope)*0.000001;  // en DESC
    
      if (currentSpeed < goalSpeed + RAMP_EPSILON) {
        // p_ros->logPrint(INFO, "Reached constant part of downwards slope");
        currentSpeed = goalSpeed;
        transit<Constant>();
      }
    }

    else {
        // Case where we have exactly the same value (to a given epsilon)
        // We just transit to constant
        currentSpeed = goalSpeed;
        transit<Constant>();
    }


  }

  // Transition de fin
  void react(EndRampEvent const & e) override {
    
    // transition
    // p_ros->logPrint(INFO, "Ending ramp from slope");
    transit<RampEnd>();
  }


  void react(GoalSpeedChangeEvent const & e) override {

    setGoalSpeed(e.newSpeed);
    // p_ros->logPrint(INFO, "Goal speed changed in ramp slope");

    transit<Slope>();  //TOTEST est ce que avec le UpdateEvent on passe bien qu'une fois dans entry ?
  };

  void react(EmergencyBrakeEvent const & e) override {

    p_ros->logPrint(WARN, "Emergency brake in ramp slope");
    transit<Brake>();
  };
};



// ----------------------------------------------------------------------------
// State: Constant
//
class Constant
: public RampSM
{
  void entry() override {
    currentState = RampState::RAMP_CONSTANT;
  }

  void react(UpdateEvent const & e) override {
    //  Nothing to do here
  }

  // Transition de fin
  void react(EndRampEvent const & e) override {
    
    // transition
    // p_ros->logPrint(INFO, "Ending ramp from constant");
    transit<RampEnd>();
  }

  void react(GoalSpeedChangeEvent const & e) override {

    setGoalSpeed(e.newSpeed);
    // p_ros->logPrint(INFO, "Goal speed changed in ramp constant");

    transit<Slope>();  // go back to slope if the goal speed is changed
  };

  void react(EmergencyBrakeEvent const & e) override {

    p_ros->logPrint(WARN, "Emergency brake in ramp constant");
    transit<Brake>();
  };
};


// ----------------------------------------------------------------------------
// State: RampIdle
//
class RampIdle
: public RampSM
{
  void entry() override {
    currentState = RampState::RAMP_IDLE;

    currentSpeed = 0.0;
  }

  void react(BeginRampEvent const & e) override {  // on commence juste la rampe
    
    // p_ros->logPrint(INFO, "Starting new ramp");
    t0 = e.t0;  // set de t0
    transit<Slope>();
  }

};

// ----------------------------------------------------------------------------
// State: RampEnd
//
class RampEnd
: public RampSM
{
  void entry() override {
    currentState = RampState::RAMP_END;

    t_start_slope = micros();
    V_start_slope = currentSpeed;
  }

  void react(UpdateEvent const & e) override {

    // ici on est forcément en DESC vers 0.0
    currentSpeed = V_start_slope - accelParam * (e.currentTime - t_start_slope)*0.000001;  // en DESC
  
    if (currentSpeed < 0.0 + RAMP_EPSILON) {

      // p_ros->logPrint(INFO, "Ramp finished ending");
      currentSpeed = 0.0;
      transit<RampIdle>();
    }
  }

};


// ----------------------------------------------------------------------------
// State: Brake
//
class Brake
: public RampSM
{
  void entry() override {
    currentState = RampState::RAMP_BRAKE;

    t_start_slope = micros();
    V_start_slope = currentSpeed;
  }

  void react(UpdateEvent const & e) override {

    // direction is down
    currentSpeed = V_start_slope - ACCEL_BRAKE * (e.currentTime - t_start_slope)*0.000001;

    // transition to RampIdle
    if (currentSpeed < 0.0 + RAMP_EPSILON) {
      currentSpeed = 0.0;
      transit<RampIdle>();
    }
  }

  // No react to a goalSpeed change event here

};





// ----------------------------------------------------------------------------
// Base state: default implementations (need all events here)
//

void RampSM::react(GoalSpeedChangeEvent const &) {
}

void RampSM::react(UpdateEvent const &) {
}

void RampSM::react(BeginRampEvent const &) {
}

void RampSM::react(EndRampEvent const &) {
}

void RampSM::react(EmergencyBrakeEvent const &) {
}

void RampSM::react(SetRampToIdleEvent const &) {
  transit<RampIdle>();
}

// Variable initializations (so that every state knows what it is)

uint32_t RampSM::t0 = 0;
uint32_t RampSM::t_start_slope = 0;
float RampSM::V_start_slope = 0.0;
float RampSM::goalSpeed = 0.0;
float RampSM::accelParam = 0.0;
float RampSM::currentSpeed = 0.0;
RampState RampSM::currentState = RampState::RAMP_UNDEF;

// ----------------------------------------------------------------------------
// Initial state definition
//
FSM_INITIAL_STATE(RampSM, RampIdle)