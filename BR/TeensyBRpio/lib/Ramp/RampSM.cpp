
#include "RampSM.hpp"

#include <ROS.hpp>
#include "main_loop.hpp"

// Constructor
RampSM::RampSM() {
    this->accelParam = 0;
    this->currentSpeed = 0;

    // init state var
    this->currentState = RampState::IDLE;

    this->t_current = 0.0;
    this->t_start_slope = 0.0;
}

void RampSM::setAccelParam(float accelParam) {
    this->accelParam = accelParam;
}

void RampSM::setGoalSpeed(float goalSpeed) {
    this->goalSpeed = goalSpeed;
}

void RampSM::setT0(float t0) {
    this->t0 = t0;
}

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

  void react(UpdateEvent const & e) override {
    
    transit<Slope>();
  }

  void react(GoalSpeedChangeEvent const & e) override {

    float newSpeed = e.newSpeed;

    transit<Slope>();
  };
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

    float dt = e.currentTime - t_current;
    t_current = e.currentTime;

    // update V (no need for previous V)
    if (goalSpeed > currentSpeed) {
        currentSpeed = V_start_slope + accelParam * (t_current - t_start_slope);  // en ASC
    }
    else if (goalSpeed < currentSpeed) {
        currentSpeed = V_start_slope - accelParam * (t_current - t_start_slope);  // en ASC
    }
    else {
        //TODO : edge case
    }

    // TODO : condition d'arrêt et transition de fin
    
  }

  void react(GoalSpeedChangeEvent const & e) override {

    setGoalSpeed(e.newSpeed);
    p_ros->logPrint(LogType::DEBUG, "Goal speed changed in ramp slope");
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

  void react(GoalSpeedChangeEvent const & e) override {

    setGoalSpeed(e.newSpeed);
    p_ros->logPrint(LogType::DEBUG, "Goal speed changed in ramp constant");

    transit<Slope>();  // go back to slope if the goal speed is changed
  };
};


// ----------------------------------------------------------------------------
// State: Brake
//

class Constant
: public RampSM
{
  void entry() override {
    currentState = RampState::BRAKE;
  }

  void react(UpdateEvent const & e) override {
    
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