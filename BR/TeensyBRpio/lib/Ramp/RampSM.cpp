
#include "RampSM.hpp"


// Constructor
RampSM::RampSM() {
    this->accelParam = 0;
    this->currentSpeed = 0;

    // init state var
    this->currentState = RampState::STILL;
}

void RampSM::setAccelParam(float accelParam) {
    this->accelParam = accelParam;
}

float RampSM::getCurrentSpeed() {
    return currentSpeed;
}



// ----------------------------------------------------------------------------
// Transition functions
//



// ----------------------------------------------------------------------------
// State: Still
//
class Still
: public RampSM
{
  void entry() override {
    currentState = RampState::STILL;
  }

  void react(UpdateEvent const & e) override {
    if (e.currentTime > 0.2) {

    }
  }

  void react(GoalSpeedChangeEvent const & e) override {

    float newSpeed = e.newSpeed;

    transit<Accel>();
  };
};


// ----------------------------------------------------------------------------
// State: Accel
//

class Accel
: public RampSM
{
  void entry() override {
    currentState = RampState::ACCEL;
  }

  void react(UpdateEvent const & e) override {
    if (e.currentTime > 0.2) {

    }
  }

  void react(GoalSpeedChangeEvent const & e) override {

    float newSpeed = e.newSpeed;

    transit<Accel>();
  };
};





// ----------------------------------------------------------------------------
// Base state: default implementations
//

void RampSM::react(GoalSpeedChangeEvent const &) {
  // std::cout << "Order event ignored" << std::endl;
}


RampState RampSM::getCurrentState() {
  return currentState;
}



// ----------------------------------------------------------------------------
// Initial state definition
//
FSM_INITIAL_STATE(RampSM, Still)