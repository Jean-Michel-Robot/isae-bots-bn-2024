
#include "RampSM.hpp"

#include <ROS.hpp>
#include "main_loop.hpp"

#include "defines.hpp"

// Constructor
RampSM::RampSM() {
    this->accelParam = 0;
    this->currentSpeed = 0;

    // init state var
    this->currentState = RampState::IDLE;

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

float RampSM::getCurrentSpeed() {
    return currentSpeed;
}





// ----------------------------------------------------------------------------
// Transition functions
//



// ----------------------------------------------------------------------------
// State: Idle
//
class Idle
: public RampSM
{
  void entry() override {
    currentState = RampState::IDLE;
  }

  void react(BeginRampEvent const & e) override {  // on commence juste la rampe
    
    p_ros->logPrint(LogType::INFO, "Starting new ramp");
    t0 = e.t0;  // set de t0
    transit<Slope>();
  }

};


// ----------------------------------------------------------------------------
// State: Slope
//
class Slope
: public RampSM
{
  void entry() override {
    currentState = RampState::SLOPE;

    t_start_slope = micros();
    V_start_slope = currentSpeed;
  }

  void react(UpdateEvent const & e) override {

    // float dt = e.currentTime - t_current;
    // t_current = e.currentTime;

    if (e.currentTime - t_start_slope < 0) {
      p_ros->logPrint(LogType::ERROR, "Ecart de temps negatif"); //TOTEST
    }

    // update currentSpeed (no need for previous currentSpeed)

    if (V_start_slope < goalSpeed - RAMP_EPSILON) {
      currentSpeed = V_start_slope + accelParam * (e.currentTime - t_start_slope);  // en ASC
    
      if (currentSpeed > goalSpeed - RAMP_EPSILON) {
        p_ros->logPrint(LogType::INFO, "Reached constant part of upwards slope");
        currentSpeed = goalSpeed;
        transit<Constant>();
      }
    }

    else if (V_start_slope > goalSpeed + RAMP_EPSILON) {
      currentSpeed = V_start_slope - accelParam * (e.currentTime - t_start_slope);  // en DESC
    
      if (currentSpeed < goalSpeed + RAMP_EPSILON) {
        p_ros->logPrint(LogType::INFO, "Reached constant part of downwards slope");
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
    p_ros->logPrint(LogType::INFO, "Ending ramp from slope");
    transit<RampEnd>();
  }


  void react(GoalSpeedChangeEvent const & e) override {

    setGoalSpeed(e.newSpeed);
    p_ros->logPrint(LogType::INFO, "Goal speed changed in ramp slope");

    transit<Slope>();  //TOTEST est ce que avec le UpdateEvent on passe bien qu'une fois dans entry ?
  };

  void react(EmergencyBrakeEvent const & e) override {

    p_ros->logPrint(LogType::WARN, "Emergency brake in ramp slope");
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
    currentState = RampState::CONSTANT;
  }

  void react(UpdateEvent const & e) override {
    //  Nothing to do here
  }

  // Transition de fin
  void react(EndRampEvent const & e) override {
    
    // transition
    p_ros->logPrint(LogType::INFO, "Ending ramp from constant");
    transit<RampEnd>();
  }

  void react(GoalSpeedChangeEvent const & e) override {

    setGoalSpeed(e.newSpeed);
    p_ros->logPrint(LogType::INFO, "Goal speed changed in ramp constant");

    transit<Slope>();  // go back to slope if the goal speed is changed
  };

  void react(EmergencyBrakeEvent const & e) override {

    p_ros->logPrint(LogType::WARN, "Emergency brake in ramp constant");
    transit<Brake>();
  };
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
    currentSpeed = V_start_slope - accelParam * (e.currentTime - t_start_slope);  // en DESC
  
    if (currentSpeed < 0.0 + RAMP_EPSILON) {
      p_ros->logPrint(LogType::INFO, "Ramp finished ending");
      currentSpeed = 0.0;
      transit<Idle>();
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

    t_start_slope = micros();
    V_start_slope = currentSpeed;
  }

  void react(UpdateEvent const & e) override {

    // direction is down
    currentSpeed = V_start_slope - ACCEL_BRAKE * (e.currentTime - t_start_slope);

    // transition to Idle
    if (currentSpeed <= 0) {
      currentSpeed = 0;
      transit<Idle>();
    }
  }

  // No react to a goalSpeed change event here

};







// ----------------------------------------------------------------------------
// Base state: default implementations
//

void RampSM::react(GoalSpeedChangeEvent const &) {
  // std::cout << "Order event ignored" << std::endl;
}

void RampSM::react(UpdateEvent const &) {
    
}


RampState RampSM::getCurrentState() {
  return currentState;
}



// ----------------------------------------------------------------------------
// Initial state definition
//
FSM_INITIAL_STATE(RampSM, Idle)