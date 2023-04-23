// #include "tinyfsm/tinyfsm.hpp"

#include "BrSM.hpp"
// #include "fsmlist.hpp"

#include "ROS.hpp"

#include "main_loop.hpp"
#include <Arduino.h>
#include <LinearTrajectory.hpp>
#include <RotationTrajectory.hpp>
#include <Trajectory.hpp>
#include "OdosPosition.hpp"

#include "Asserv.hpp"


// define string names for states
static const char *BrStateStr[] = {
    FOREACH_BRSTATE(GENERATE_STRING)
};

// forward declarations
class Forward;
class FinalRot;
class InitRot;

// ----------------------------------------------------------------------------
// Transition functions
//

// static void CallMaintenance() {
//   std::cout << "*** calling maintenance ***" << std::endl;
// }



// Helper function to setup a trajectory
void BrSM::setupTrajectory() {

  // RAZ des variables
  //TODO

  // Set robot position
  currentTrajectory->setRobotPos( p_odos->getRobotPosition() );

  // Set destination using order info
  currentTrajectory->setDest( currentOrder );

  // Begin trajectory
  currentTrajectory->beginTrajectory( micros() );
}


// ----------------------------------------------------------------------------
// State: Arrived
//

class Arrived
: public BrSM
{
  void entry() override {
    currentState = BRState::BR_ARRIVED;
  }

  void react(BrUpdateEvent const & e) override {
    //TODO bloquer les moteurs avec l'asserv (a la difference de IDLE)

  }

  void react(OrderEvent const & e) override {

    // store order
    this->currentOrder = e.order;

    p_ros->logPrint(LogType::INFO, "Transition : Arrived -> InitRot");

    //TODO : check if both axis are running (closed loop state)

    transit<Forward>();  //FORTEST remettre initRot
  }
};


// ----------------------------------------------------------------------------
// State: InitRot
//

class InitRot
: public BrSM
{
  void entry() override {
    currentState = BRState::BR_INITROT;

    // Changement de trajectoire en rotation
    currentTrajectory = p_rotationTrajectory;

    setupTrajectory();

    // Démarrage de l'asserv
    //TODO
  }


  void react(GoalReachedEvent const & e) override {

    switch (e.goalType) {

      case GoalType::ORIENT :
        p_ros->logPrint(LogType::INFO, "Transition : InitRot -> Arrived");
        
        transit<Arrived>();
      break;

      case GoalType::TRANS :
        p_ros->logPrint(LogType::INFO, "Transition : InitRot -> Forward");

        transit<Forward>();
      break;

      default :
        // error
        p_ros->logPrint(LogType::ERROR, "Wrong goal type in state InitRot");
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
    currentState = BRState::BR_FORWARD;

    // Changement de trajectoire en rotation
    currentTrajectory = p_linearTrajectory;

    setupTrajectory();
  }

  void react(GoalReachedEvent const & e) override {

    switch (e.goalType) {

      case GoalType::TRANS :
        p_ros->logPrint(LogType::INFO, "Transition : Forward -> Arrived");

        transit<Arrived>();
      break;

      case GoalType::FINAL :
        p_ros->logPrint(LogType::INFO, "Transition : Forward -> FinalRot");

        transit<FinalRot>();
      break;

      default :
        // error
        p_ros->logPrint(LogType::ERROR, "Wrong goal type in state Forward");
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
    currentState = BRState::BR_FINALROT;
  }


  void react(GoalReachedEvent const & e) override {

    switch (e.goalType) {

      case GoalType::FINAL :
        p_ros->logPrint(LogType::INFO, "Transition : FinalRot -> Arrived");

        transit<Arrived>();
      break;

      default :
        // error
        p_ros->logPrint(LogType::ERROR, "Wrong goal type in state FinalRot");
    }

  };
};


// ----------------------------------------------------------------------------
// State: BR_Idle
//

class BR_Idle
: public BrSM
{
  void entry() override {
        digitalWrite(13, 1);

    currentState = BRState::BR_IDLE;
  }

  void react(BrUpdateEvent const & e) override {
    // ne rien faire en Idle
  }

  void react(OrderEvent const & e) override {

    // store order
    this->currentOrder = e.order;

    p_ros->logPrint(LogType::INFO, "Transition : Idle -> InitRot");

    //TODO : check if both axis are running (closed loop state)

    transit<Forward>();  // mettre une fonctions dans transit fait qu'elle est exécutée
                         // après le exit() de l'état
                         //FORTEST remttre a InitRot
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

void BrSM::react(BrUpdateEvent const & e) {

  uint32_t t = e.currentTime;

  if (currentTrajectory == NULL) {
    //Serial.println("ERROR : pointer to current trajectory is NULL");
    return;
  }

  currentTrajectory->updateTrajectory(t);

  p_asserv->updateError( currentTrajectory->getTrajectoryPoint() );

  p_asserv->updateCommand(
    currentTrajectory->getTrajectoryLinearSpeed(),
    currentTrajectory->getTrajectoryAngularSpeed());
  //TODO update la commande des moteurs directement dans l'asserv

  

  if ( !currentTrajectory->isTrajectoryActive() ) {

    // Can mean that the trajectory is done
    if (p_asserv->isAtObjectivePoint(false) || true) {  //TODO checkangle ??

      //Serial.println("Send goal reached event");

      GoalReachedEvent e;
      e.goalType = currentOrder.goalType;
      send_event(e);
    }
  }
}


BRState BrSM::getCurrentState() {
  return currentState;
}

String BrSM::getCurrentStateStr() {
  return BrStateStr[currentState];
}

float BrSM::getCurrentTargetSpeed() {
  if (currentTrajectory->trajectoryType == TrajectoryType::TRAJ_LINEAR) {
    return currentTrajectory->getTrajectoryLinearSpeed();
  }
  else if (currentTrajectory->trajectoryType == TrajectoryType::TRAJ_ROTATION) {
    return currentTrajectory->getTrajectoryAngularSpeed();
  }
  else {
    p_ros->logPrint(LogType::ERROR, "ERROR : Unhandled trajectory type");
    return 0.0;
  }
}

// Variable initializations
AxisStates BrSM::axisStates = {0};
OrderType BrSM::currentOrder = {0};
Trajectory* BrSM::currentTrajectory = NULL;

// BrSM::current_state_ptr

// ----------------------------------------------------------------------------
// Initial state definition
//
BRState BrSM::currentState = BRState::BR_IDLE;
FSM_INITIAL_STATE(BrSM, BR_Idle)
