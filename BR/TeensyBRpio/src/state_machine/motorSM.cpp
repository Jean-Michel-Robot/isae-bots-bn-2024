#include "tinyfsm.hpp"
#include "state_machine/motorSM.hpp"


// ----------------------------------------------------------------------------
// Motor states
//

class Motor_Idle
: public MotorSM
{
  void entry() override {

  };
};

class Motor_Calib
: public MotorSM
{
  void entry() override {

  };
};

class Motor_Running
: public MotorSM
{
  void entry() override {

  };
};

class Motor_Stopped
: public MotorSM
{
  void entry() override {

  };
};


// ----------------------------------------------------------------------------
// Base State: default implementations
//

void MotorSM::react(CalibrationEvent const &) {
//   transit<Stopped>();
}

void MotorSM::react(StopEvent const &) {
//   transit<Up>();
}


int MotorSM::axisState{0};


// ----------------------------------------------------------------------------
// Initial state definition
//
FSM_INITIAL_STATE(MotorSM, Motor_Idle)
