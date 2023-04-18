
#include "RampSM.hpp"

// #include <ROS.hpp>
// #include "main_loop.hpp"

#include <Arduino.h>

#include "RampDefines.hpp"

// Constructor
RampSM::RampSM() {
    this->accelParam = 0;
    this->currentSpeed = 0;

    // init state var
    this->currentState = RampState::RAMP_IDLE;

    // this->t_current = 0.0;
    this->t_start_slope = 0.0;
}

void RampSM::setAccelParam(float accelParam) {
    this->accelParam = accelParam;
}

void RampSM::setGoalSpeed(float goalSpeed) {
    this->goalSpeed = goalSpeed;
}


// void RampSM::setT0(float t0) {
//     this->t0 = t0;
// }

RampState RampSM::getCurrentState() {
    return currentState;
}

float RampSM::getCurrentSpeed() {
    return currentSpeed;
}

void RampSM::setCurrentState(RampState rampState) {
  currentState = rampState;
}

// Forward declarations
class Constant;
class RampEnd;
class Brake;



// RampSM::t_start_slope = 0.0;


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
    // currentState = RampState::SLOPE;
    setCurrentState(RampState::SLOPE);

    t_start_slope = micros();
    V_start_slope = currentSpeed;
  }

  void react(UpdateEvent const & e) override {

    // float dt = e.currentTime - t_current;
    // t_current = e.currentTime;

    if (e.currentTime - t_start_slope < 0) {
      Serial.println("Ecart de temps negatif");
      // p_ros->logPrint(LogType::ERROR, "Ecart de temps negatif"); //TOTEST pas de valeurs négatives
    }

    // update currentSpeed (no need for previous currentSpeed)

    if (V_start_slope < goalSpeed - RAMP_EPSILON) {
      currentSpeed = V_start_slope + accelParam * (e.currentTime - t_start_slope)*0.000001;  // en ASC
    
      if (currentSpeed > goalSpeed - RAMP_EPSILON) {
        Serial.println("Reached constant part of upwards slope");
        // p_ros->logPrint(LogType::INFO, "Reached constant part of upwards slope");
        currentSpeed = goalSpeed;
        transit<Constant>();
      }
    }

    else if (V_start_slope > goalSpeed + RAMP_EPSILON) {
      currentSpeed = V_start_slope - accelParam * (e.currentTime - t_start_slope)*0.000001;  // en DESC
    
      if (currentSpeed < goalSpeed + RAMP_EPSILON) {
        Serial.println("Reached constant part of downwards slope");
        // p_ros->logPrint(LogType::INFO, "Reached constant part of downwards slope");
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
    Serial.println("Ending ramp from slope");
    // p_ros->logPrint(LogType::INFO, "Ending ramp from slope");
    transit<RampEnd>();
  }


  void react(GoalSpeedChangeEvent const & e) override {

    setGoalSpeed(e.newSpeed);
    Serial.println("Goal speed changed in ramp slope");
    // p_ros->logPrint(LogType::INFO, "Goal speed changed in ramp slope");

    transit<Slope>();  //TOTEST est ce que avec le UpdateEvent on passe bien qu'une fois dans entry ?
  };

  void react(EmergencyBrakeEvent const & e) override {

    Serial.println("Emergency brake in ramp slope");
    // p_ros->logPrint(LogType::WARN, "Emergency brake in ramp slope");
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
    // currentState = RampState::CONSTANT;
    setCurrentState(RampState::CONSTANT);
  }

  void react(UpdateEvent const & e) override {
    //  Nothing to do here
  }

  // Transition de fin
  void react(EndRampEvent const & e) override {
    
    // transition
    Serial.println("Ending ramp from constant");
    // p_ros->logPrint(LogType::INFO, "Ending ramp from constant");
    transit<RampEnd>();
  }

  void react(GoalSpeedChangeEvent const & e) override {

    setGoalSpeed(e.newSpeed);
    Serial.println("Goal speed changed in ramp constant");
    // p_ros->logPrint(LogType::INFO, "Goal speed changed in ramp constant");

    transit<Slope>();  // go back to slope if the goal speed is changed
  };

  void react(EmergencyBrakeEvent const & e) override {

    Serial.println("Emergency brake in ramp constant");
    // p_ros->logPrint(LogType::WARN, "Emergency brake in ramp constant");
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


    Serial.println("ICI dans entry");
    delay(100);
    // currentState = RampState::RAMP_IDLE;
    // setCurrentState(RampState::RAMP_IDLE);
  }

  void react(BeginRampEvent const & e) override {  // on commence juste la rampe
    
    Serial.println("Starting new ramp");
    // p_ros->logPrint(LogType::INFO, "Starting new ramp");
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
    // currentState = RampState::RAMP_END;
    setCurrentState(RampState::RAMP_END);

    t_start_slope = micros();
    V_start_slope = currentSpeed;
  }

  void react(UpdateEvent const & e) override {

    // ici on est forcément en DESC vers 0.0
    currentSpeed = V_start_slope - accelParam * (e.currentTime - t_start_slope)*0.000001;  // en DESC
  
    if (currentSpeed < 0.0 + RAMP_EPSILON) {

      Serial.println("Ramp finished ending");
      // p_ros->logPrint(LogType::INFO, "Ramp finished ending");
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
    currentState = RampState::BRAKE;
    // setCurrentState(RampState::BRAKE);

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


// Variable initializations (so that every state knows what it is)

float RampSM::t0 = 0.0;
float RampSM::t_start_slope = 0.0;
float RampSM::V_start_slope = 0.0;
float RampSM::goalSpeed = 0.0;
float RampSM::accelParam = 0.0;
float RampSM::currentSpeed = 0.0;
RampState RampSM::currentState = RampState::RAMP_IDLE;

// ----------------------------------------------------------------------------
// Initial state definition
//
FSM_INITIAL_STATE(RampSM, RampIdle)