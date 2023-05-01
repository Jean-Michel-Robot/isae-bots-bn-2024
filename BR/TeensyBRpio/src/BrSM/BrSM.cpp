// #include "tinyfsm/tinyfsm.hpp"

#include "BrSM/BrSM.hpp"
// #include "fsmlist.hpp"


#include <Arduino.h>
#include "Trajectories/Trajectory.hpp"
#include "Trajectories/LinearTrajectory.hpp"
#include "Trajectories/RotationTrajectory.hpp"
#include "OdosPosition.hpp"
#include <Motors.hpp>

#include "Asserv.hpp"

#include "ROS.hpp"
#include "main_loop.hpp"


// define string names for states
static const char *BrStateStr[] = {
    FOREACH_BRSTATE(GENERATE_STRING)
};

// forward declarations
class Forward;
class FinalRot;
class InitRot;
class BR_Recal;
class BR_Idle;

// constructor
BrSM::BrSM() {


  // set timer lengths
  recalTimer.setLength(RECAL_TIMER_LENGTH);

  // initialize switches
  m_switches[BR_RIGHT] = new SwitchFiltered(BUMPER_RIGHT_PIN);
  m_switches[BR_LEFT] = new SwitchFiltered(BUMPER_LEFT_PIN);
}

// ----------------------------------------------------------------------------
// Transition functions
//

// static void CallMaintenance() {
//   std::cout << "*** calling maintenance ***" << std::endl;
// }



// Helper function to setup a trajectory
void BrSM::setupTrajectory() {

  //TODO RAZ des variables

  // Set robot position
  currentTrajectory->setRobotPos( p_odos->getRobotPosition() );

  // Set destination using order info
  currentTrajectory->setDest( currentOrder );

  // Begin trajectory
  currentTrajectory->beginTrajectory( micros() );
}


// ----------------------------------------------------------------------------
// State: Ready
//

class Ready
: public BrSM
{
  void entry() override {
    currentState = BR_READY;

    // send callback to HN
    p_ros->sendCallback(OK_POS);
  }

  void react(BrUpdateEvent const & e) override {
    //TODO bloquer les moteurs avec l'asserv (a la difference de IDLE)

  }

  void react(OrderEvent const & e) override {

    //TODO : check if both axis are running (closed loop state)

    // store order
    currentOrder = e.order;
    p_ros->logPrint(INFO, "Order received : ("+String(currentOrder.x)+", "+
                          String(currentOrder.y)+", "+String(currentOrder.theta)+
                          ") with goalType "+String(currentOrder.goalType));


    // Transition depending on the order goal type
    switch (currentOrder.goalType) {

      //TOTEST les switch cases
      case ORIENT:
      case TRANS:
      case FINAL:
        p_ros->logPrint(INFO, "BR Transition : Ready -> InitRot");
        transit<Forward>();  //FORTEST remettre initRot
      break;

      case RECAL_FRONT:
      case RECAL_BACK:
        p_ros->logPrint(INFO, "BR Transition : Ready -> Recal");
        transit<BR_Recal>();
      break;

      default:
        p_ros->logPrint(ERROR, "Order ignore because not recognized");
    }
  }


  void react(BrSetToIdleEvent const & e) override {

    //TODO set motor speed to 0
    //TODO set motor state to Idle

    p_ros->logPrint(INFO, "BR Transition : Ready -> Idle");
    transit<BR_Idle>();
  }
};


// ----------------------------------------------------------------------------
// State: InitRot
//

class InitRot
: public BrSM
{
  void entry() override {
    currentState = BR_INITROT;

    // Changement de trajectoire en rotation
    currentTrajectory = p_rotationTrajectory;

    setupTrajectory();
  }


  void react(GoalReachedEvent const & e) override {

    switch (e.goalType) {

      case GoalType::ORIENT :
        p_ros->logPrint(INFO, "BR Transition : InitRot -> Ready");
        
        transit<Ready>();
      break;

      case GoalType::TRANS :
        p_ros->logPrint(INFO, "BR Transition : InitRot -> Forward");

        transit<Forward>();
      break;

      default :
        // error
        p_ros->logPrint(ERROR, "Wrong goal type in state InitRot");
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
    currentState = BR_FORWARD;

    // Changement de trajectoire en linéaire
    currentTrajectory = p_linearTrajectory;

    setupTrajectory();
  }

  void react(GoalReachedEvent const & e) override {

    switch (e.goalType) {

      case GoalType::TRANS :
        p_ros->logPrint(INFO, "BR Transition : Forward -> Ready");

        transit<Ready>();
      break;

      case GoalType::FINAL :
        p_ros->logPrint(INFO, "BR Transition : Forward -> FinalRot");

        transit<FinalRot>();
      break;

      default :
        // error
        p_ros->logPrint(ERROR, "Wrong goal type in state Forward");
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
    currentState = BR_FINALROT;

    // Changement de trajectoire en rotation
    currentTrajectory = p_rotationTrajectory;

    setupTrajectory();
  }


  void react(GoalReachedEvent const & e) override {

    switch (e.goalType) {

      case GoalType::FINAL :
        p_ros->logPrint(INFO, "BR Transition : FinalRot -> Ready");

        transit<Ready>();
      break;

      default :
        // error
        p_ros->logPrint(ERROR, "Wrong goal type in state FinalRot");
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
    currentState = BR_IDLE;
  }

  void react(BrUpdateEvent const & e) override {
    // ne rien faire en Idle
  }

  void react(BrGetReadyEvent const & e) override {

    //TODO check if there is no motor error
    //TODO Check is calibration is needed or not
    p_ros->logPrint(INFO, "BR Transition : Idle -> Ready");

    transit<Ready>();
  }

  // // We transition to the Ready state when notified by a BrGetReadyEvent
  // void react(OrderEvent const & e) override {

  //   // store order
  //   this->currentOrder = e.order;

  //   p_ros->logPrint(INFO, "Transition : Idle -> InitRot");

  //   //TODO : check if both axis are running (closed loop state)

  //   transit<Forward>();  // mettre une fonctions dans transit fait qu'elle est exécutée
  //                        // après le exit() de l'état
  //                        //FORTEST remttre a InitRot
  // }
};


// ----------------------------------------------------------------------------
// State: BR_Recal
//

class BR_Recal
: public BrSM
{
  void entry() override {
    currentState = BR_RECAL;

    // make sure we start using the asserv
    isRecalInAsservPhase = true;

    recalTimer.start( millis() );
  }

  void react(BrUpdateEvent const & e) override {
    /*
    On avance/recule en asserv pendant un certain temps lié à la distance 
    au mur à laquelle on pense être
    Puis on recule en commande en bypassant l'asserv
    Pendant tout le temps on surveille les bumpers :
    - si un bumper s'active pendant la phase asserv on passe en phase commande
    - si un bumper s'active en phase commande on réduit la commande de vitesse
    de ce côté
    - quand les deux bumpers sont activés on attend un peu avant de reset la 
    position
    */

    if (isRecalInAsservPhase &&
      (recalTimer.isExpired( millis() ) ||
      m_switches[BR_RIGHT]->isSwitchPressed() ||
      m_switches[BR_LEFT]->isSwitchPressed()) ) {  // condition pour passer en recul commande

        isRecalInAsservPhase = false;
    }

    if (isRecalInAsservPhase) {
      // TODO reculer en asserv
    }


    // sinon on recule en commande et on analyse les bumpers
    float cmd_right = RECAL_SPEED;
    float cmd_left = RECAL_SPEED;

    if (m_switches[BR_RIGHT]->isSwitchPressed()) {
        cmd_right = RECAL_SPEED / 3;
    }
    if (m_switches[BR_LEFT]->isSwitchPressed()) {
        cmd_left = RECAL_SPEED / 3;
    }

    // if both bumpers are pressed we wait for a while and then transition
    if (m_switches[BR_RIGHT]->isSwitchPressed() && 
        m_switches[BR_LEFT]->isSwitchPressed()) {

        //TODO

        // stop motors (send command 0)
        sendMotorCommand(BR_RIGHT, 0);
        sendMotorCommand(BR_LEFT, 0);

        //TODO reset position on axis

        transit<Ready>();

    // Send motor commands
    sendMotorCommand(BR_RIGHT, cmd_right);
    sendMotorCommand(BR_LEFT, cmd_left);
  }
};

// ----------------------------------------------------------------------------
// Base state: default implementations
//

void BrSM::react(OrderEvent const &) {
  p_ros->logPrint(DEBUG, "OrderEvent ignored");
}

void BrSM::react(GoalReachedEvent const &) {
  p_ros->logPrint(DEBUG, "GoalReachedEvent ignored");
}

void BrSM::react(ErrorEvent const &) {
  p_ros->logPrint(DEBUG, "ErrorEvent ignored");
}

void BrSM::react(BrGetReadyEvent const &) {
  p_ros->logPrint(DEBUG, "BrGetReadyEvent ignored");
}

void BrSM::react(BrSetToIdleEvent const &) {
  p_ros->logPrint(DEBUG, "BrSetToIdleEvent ignored");
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

    // Can mean that the trajectory is done if the asserv agrees
    if (p_asserv->isAtObjectivePoint(false) || true) {  //TODO checkangle ?? //FORTEST

      //Serial.println("Send goal reached event");

      GoalReachedEvent e;
      e.goalType = currentOrder.goalType;
      send_event(e);
    }

    // otherwise we let the asserv stabilize close to the end point
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
    p_ros->logPrint(ERROR, "ERROR : Unhandled trajectory type");
    return 0.0;
  }
}

// Variable initializations
AxisStates BrSM::axisStates = {0};
OrderType BrSM::currentOrder = {0};
Trajectory* BrSM::currentTrajectory = NULL;
Timer BrSM::recalTimer = Timer( millis() );
bool BrSM::isRecalInAsservPhase = true;
SwitchFiltered* BrSM::m_switches[2] = {NULL};

// BrSM::current_state_ptr

// ----------------------------------------------------------------------------
// Initial state definition
//
BRState BrSM::currentState = BR_IDLE;
FSM_INITIAL_STATE(BrSM, BR_Idle)
