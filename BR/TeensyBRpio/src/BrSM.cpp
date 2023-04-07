// #include "tinyfsm/tinyfsm.hpp"

#include "BrSM.hpp"
// #include "fsmlist.hpp"

#include <ROS.hpp>

#include "main_loop.hpp"


// for prints (peut etre pas que)

// forward declarations
class Forward;
class FinalRot;

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
    currentState = BRState::ARRIVED;

    // send_event(MotorStop());
	}
};


// ----------------------------------------------------------------------------
// State: InitRot
//

class InitRot
: public BrSM
{
  void entry() override {
    currentState = BRState::INITROT;
  }

  void react(GoalReachedEvent const & e) override {

    switch (e.goalType) {

      case GoalType::ORIENT :
      {
        p_ros->logPrint(LogType::INFO, "Transition : InitRot -> Arrived");
        
        transit<Arrived>();
        
      }

      case GoalType::TRANS :
      {
        p_ros->logPrint(LogType::INFO, "Transition : InitRot -> Forward");

        transit<Forward>();
      }

      default :
      {
        // error
        p_ros->logPrint(LogType::ERROR, "Wrong goal type in state InitRot");
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
  void entry() override {
    currentState = BRState::FORWARD;
  }

  void react(GoalReachedEvent const & e) override {

    switch (e.goalType) {

      case GoalType::TRANS :
      {
        p_ros->logPrint(LogType::INFO, "Transition : Forward -> Arrived");

        transit<Arrived>();
      }

      case GoalType::FINAL :
      {
        p_ros->logPrint(LogType::INFO, "Transition : Forward -> FinalRot");

        transit<FinalRot>();
      }

      default :
      {
        // error
        p_ros->logPrint(LogType::ERROR, "Wrong goal type in state Forward");
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
  void entry() override {
    currentState = BRState::FINALROT;
  }


  void react(GoalReachedEvent const & e) override {

    switch (e.goalType) {

      case GoalType::FINAL :
      {
        p_ros->logPrint(LogType::INFO, "Transition : FinalRot -> Arrived");

        transit<Arrived>();
      }

      default :
      {
        // error
        p_ros->logPrint(LogType::ERROR, "Wrong goal type in state FinalRot");
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
    currentState = BRState::IDLE;
  }

  void react(OrderEvent const & e) override {

    
    // store order
    this->currentOrder = e.order;

    p_ros->logPrint(LogType::INFO, "Transition : Idle -> InitRot");
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

BRState BrSM::getCurrentState() {
  return currentState;
}

// Variable initializations
AxisStates BrSM::axisStates = {0};
OrderType BrSM::currentOrder = {0};

// BrSM::current_state_ptr

// ----------------------------------------------------------------------------
// Initial state definition
//
BRState BrSM::currentState = BRState::UNDEF;
FSM_INITIAL_STATE(BrSM, Idle)
