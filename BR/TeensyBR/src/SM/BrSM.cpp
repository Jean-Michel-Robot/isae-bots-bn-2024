#include "tinyfsm/tinyfsm.hpp"

#include "BrSM.hpp"
#include "fsmlist.hpp"

//TODO : for debug
#include <Arduino.h>


class Idle; // forward declaration


// ----------------------------------------------------------------------------
// Transition functions
//

// static void CallMaintenance() {
//   std::cout << "*** calling maintenance ***" << std::endl;
// }



// ----------------------------------------------------------------------------
// State: Arrived
//

class Arrived
: public BrSM
{
  void entry() override {
    // send_event(MotorStop());
	}
};


// ----------------------------------------------------------------------------
// State: InitRot
//

class InitRot
: public BrSM
{
  void react(GoalReachedEvent const & e) override {

    switch (e.goalType) {

      case GoalType::ORIENT :
      {
        Serial.println("Transition : InitRot -> Arrived");
        transit<Arrived>();
      }

      case GoalType::TRANS :
      {
        Serial.println("Transition : InitRot -> Forward");
        transit<Forward>();
      }

      default :
      {
        // error
        Serial.println("Wrong goal type in state InitRot");
      }
    }

  };
};


// ----------------------------------------------------------------------------
// State: Forward
//

class Forward
: public BrSM
{
  void react(GoalReachedEvent const & e) override {

    switch (e.goalType) {

      case GoalType::TRANS :
      {
        Serial.println("Transition : Forward -> Arrived");
        transit<Arrived>();
      }

      case GoalType::FINAL :
      {
        Serial.println("Transition : Forward -> FinalRot");
        transit<FinalRot>();
      }

      default :
      {
        // error
        Serial.println("Wrong goal type in state Forward");
      }
    }

  };
};

// ----------------------------------------------------------------------------
// State: FinalRot
//

class FinalRot
: public BrSM
{
  void react(GoalReachedEvent const & e) override {

    switch (e.goalType) {

      case GoalType::FINAL :
      {
        Serial.println("Transition : FinalRot -> Arrived");
        transit<Arrived>();
      }

      default :
      {
        // error
        Serial.println("Wrong goal type in state FinalRot");
      }
    }

  };
};


// ----------------------------------------------------------------------------
// State: Idle
//

class Idle
: public BrSM
{
  void entry() override {

  }

  void react(OrderEvent const & e) override {

    
    // store order
    currentOrder = e.order;

    Serial.println("Transition : Idle -> InitRot");
  }
};


// ----------------------------------------------------------------------------
// Base state: default implementations
//

void BrSM::react(OrderEvent const &) {
  // std::cout << "Order event ignored" << std::endl;
}

void BrSM::react(GoalReachedEvent const &) {
  // std::cout << "Objective reached event ignored" << std::endl;
}

void BrSM::react(ErrorEvent const &) {
  // std::cout << "Error event ignored" << std::endl;
}


// Variable initializations
AxisStates BrSM::axisStates = {0};

// ----------------------------------------------------------------------------
// Initial state definition
//
FSM_INITIAL_STATE(BrSM, Idle)
