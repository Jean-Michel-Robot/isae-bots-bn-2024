#include <stdlib.h>

#include "state_machine/BrSM.hpp"
#include "state_machine/RampSM.hpp"
#include "utils/clock.h"
#include "logging.h"
#include "state_machine/Callback.hpp"

#include "geometry/GeometricTools.hpp"
#include "trajectories/Trajectory.hpp"
#include "trajectories/LinearTrajectory.hpp"
#include "trajectories/RotationTrajectory.hpp"

#include "feedback/PositionFeedback.hpp"

#include "motors.hpp"

#include "Asserv.hpp"

// define string names for states
static const char *BrStateStr[] = {
    FOREACH_BRSTATE(GENERATE_STRING)};

// forward declarations (all of them to have the list of states here)$
class Ready;
class InitRot;
class Forward;
class FinalRot;
class BR_Idle;
class BR_RecalAsserv;
class BR_RecalDetect;
class BR_EmergencyStop;

// constructor
void BrSM::setup()
{
  // set timer lengths
  recalAsservTimer.setLength(RECAL_ASSERV_TIMEOUT);
  waitTimer.setLength(BR_WAIT_TIMER);

  // initialize switches
  m_switches[BR_RIGHT] = new SwitchFiltered(BUMPER_RIGHT_PIN);
  m_switches[BR_LEFT] = new SwitchFiltered(BUMPER_LEFT_PIN);
}

// ----------------------------------------------------------------------------
// Transition functions
//

// ----------------------------------------------------------------------------
// Helper functions
//

// Helper function to setup a trajectory
void BrSM::setupTrajectory()
{
  // TODO RAZ des variables

  // Set robot position
  Position2D robotPos = PositionFeedback::instance().getRobotPosition();
  currentTrajectory->setRobotPos(robotPos);

  // Set destination using order info
  // NOTE depends on the current state and the goal type
  switch (currentState)
  {

  case BR_INITROT:
  {
    float thetaDest = 0.0;

    switch (currentOrder.goalType)
    {

    case ORIENT:
      thetaDest = currentOrder.theta;
      break;

    case TRANS:
    case FINAL:
      thetaDest = Position2D<Meter>::s_angleBetweenTwoPoints(robotPos, currentOrder);
      break;
    }
    currentTrajectory->setDest(Position2D<Meter>(0.0, 0.0, thetaDest));
    break;
  }

  case BR_FORWARD:
  case REVERSE:
  {
    currentTrajectory->setDest(Position2D<Meter>(currentOrder.x, currentOrder.y, 0.0));
    break;
  }

  case BR_FINALROT:
  {
    currentTrajectory->setDest(Position2D<Meter>(0.0, 0.0, currentOrder.theta));
    break;
  }

  default:
  {
    log(ERROR, "Current state not handled in setDest");
    break;
  }
  }
  
  // si Dtotale vaut 0 (ou est proche de 0) c'est que l'ordre peut être ignoré
  // (pas besoin de se déplacer pour une trajectoire super courte)
  // On ignore alors l'étape et on passe à la suite en envoyant un GoalReachedEvent
  if (currentTrajectory->Dtotale < EPSILON_DTOTALE)
  {
    log(WARN, "Trajectory ignored because Dtotale is too small");
    // GoalReachedEvent goalReachedEvent;
    // goalReachedEvent.goalType = currentOrder.goalType;
    // send_event(goalReachedEvent);
    return; // on ne lance pas la trajectoire
  }

  // Begin trajectory
  currentTrajectory->beginTrajectory(micros());
}

// ----------------------------------------------------------------------------
// State: Ready
//

class Ready
    : public BrSM
{
  void entry() override
  {

    // send callback to HN depending of the previous state
    // TOTEST
    switch (currentState) {

      case BR_IDLE: // callback to confirm ready state
        sendCallback(OK_READY);
        break;

      // case BR_INITROT:
      // case BR_FINALROT:
      //   sendCallback(OK_TURN);
      //   break;

      // case BR_FORWARD:
      //   sendCallback(OK_POS);
      //   break;

      // case BR_RECAL_DETECT:
      //   sendCallback(OK_RECAL);
      //   break;

      default:
        break;
    }

    currentState = BR_READY;

    // need to have an error of 0 when transitioning to Ready before any trajectory
    // if (currentTrajectory == NULL) {
    //   currentGoalPos = p_odos->getRobotPosition();
    // }
  }

  void react(BrUpdateEvent const &e) override
  {

    // check if we have to transit to Idle
    if (isSupposedToBeIdle)
    {

      int motor_states[2];
      getCurrentMotorStates(motor_states);      
      if (true || (motor_states[0] == AXIS_STATE_IDLE && motor_states[1] == AXIS_STATE_IDLE))  //FORTEST
      {

        log(INFO, "BR Transition : Ready -> Idle");
        transit<BR_Idle>();

        return;
      }
    }

    // On bloque les moteurs avec l'asserv (a la difference de IDLE)

    // if (currentTrajectory == NULL)
    // { // should only be the case at the start before the first order

    //   // Position2D startPos = Position2D(0.0, 0.0, 0.0);
    //   // p_odos->setPosition(startPos);
    //   // currentGoalPos = startPos;
    // }
    // else  // Cas ou on vient de sortir d'une trajectoire, on se
    // {
    //   currentGoalPos = currentTrajectory->getGoalPoint();
    // }

    Asserv::instance().updateError(toAsservPointFrame(currentGoalPos));

    float nullSpeed[2] = {0.0, 0.0};

    Asserv::instance().updateCommand_2(nullSpeed);
  }

  void react(OrderEvent const &e) override
  {

    // TODO : check if both axis are running (closed loop state)

    // store order
    currentOrder = e.order;
    log(INFO, "Order received : (" + ToString(currentOrder.x) + ", " +
                              ToString(currentOrder.y) + ", " + ToString(currentOrder.theta) +
                              ") with goalType " + ToString(currentOrder.goalType));

    // Transition depending on the order goal type
    switch (currentOrder.goalType)
    {

    case ORIENT:
    case TRANS:
    case FINAL:
      log(INFO, "BR Transition : Ready -> InitRot");
      transit<InitRot>();
      break;

    case REVERSE:
      log(INFO, "BR Transition : Ready -> Forward");
      transit<Forward>();
      break;      

    case RECAL_FRONT:
    case RECAL_BACK:
      log(INFO, "BR Transition : Ready -> RecalAsserv");
      transit<BR_RecalAsserv>();
      break;

    default:
      log(ERROR, "Order ignored because not recognized");
    }
  }

  void react(BrSetToIdleEvent const &e) override
  {

    // Set motor speed to 0  //TODO put elsewhere ?
    sendMotorCommand(BR_RIGHT, 0.0);
    sendMotorCommand(BR_LEFT, 0.0);

    // Send request for motors state to be Idle
    setMotorsToIdle(); // NOTE without waiting

    // will trigger the transition when the Odrive response will be received
    isSupposedToBeIdle = true;
  }
};

// ----------------------------------------------------------------------------
// State: InitRot
//

class InitRot
  : public BrSM
{
  void entry() override
  {
    currentState = BR_INITROT;

    // Changement de trajectoire en rotation
    currentTrajectory = std::make_unique<RotationTrajectory>();

    setupTrajectory();
  }

  void react(GoalReachedEvent const &e) override
  {

    // Check the goalType
    if (e.goalType != currentOrder.goalType)
    {
      log(ERROR, "Goal type does not match in state InitRot");
    }

    switch (currentOrder.goalType)
    {

    case GoalType::ORIENT:
      log(INFO, "BR Transition : InitRot -> Ready");

      // requestedState = BR_READY;
      // waitTimer.start( millis() );
      sendCallback(OK_TURN);
      transit<Ready>();
      break;

    case GoalType::TRANS:
    case GoalType::FINAL:
      log(INFO, "BR Transition : InitRot -> Forward");

      // requestedState = BR_FORWARD;
      // waitTimer.start( millis() );
      sendCallback(OK_TURN);
      transit<Forward>();
      break;

    default:
      // error
      log(ERROR, "Wrong goal type in state InitRot");
    }
  }

  void react(BrEmergencyBrakeEvent const &) override
  {
    // dont react to an emergency brake event in Idle
  }
};

// ----------------------------------------------------------------------------
// State: Forward
//

class Forward
    : public BrSM
{
  void entry() override
  {
    currentState = BR_FORWARD;

    // Changement de trajectoire en linéaire
    currentTrajectory =  std::make_unique<LinearTrajectory>();

    setupTrajectory();
  }

  void react(GoalReachedEvent const &e) override
  {

    // Check the goalType
    if (e.goalType != currentOrder.goalType)
    {
      log(ERROR, "Goal type does not match in state Forward");
    }

    switch (currentOrder.goalType)
    {

    case GoalType::TRANS:
      log(INFO, "BR Transition : Forward -> Ready");

      // requestedState = BR_READY;
      // waitTimer.start( millis() );
      sendCallback(OK_POS);
      transit<Ready>();
      break;

    case GoalType::REVERSE:
      log(INFO, "BR Transition : Forward -> Ready");
      sendCallback(OK_POS);
      transit<Ready>();
      break;

    case GoalType::FINAL:
      log(INFO, "BR Transition : Forward -> FinalRot");

      // requestedState = BR_FINALROT;
      // waitTimer.start( millis() );
      sendCallback(OK_POS);
      transit<FinalRot>();
      break;

    default:
      // error
      log(ERROR, "Wrong goal type in state Forward");
    }
  };
};

// ----------------------------------------------------------------------------
// State: FinalRot
//

class FinalRot
    : public BrSM
{
  void entry() override
  {
    currentState = BR_FINALROT;

    // Changement de trajectoire en rotation
    currentTrajectory = std::make_unique<RotationTrajectory>();

    setupTrajectory();
  }

  void react(GoalReachedEvent const &e) override
  {

    // Check the goalType
    if (e.goalType != currentOrder.goalType)
    {
      log(ERROR, "Goal type does not match in state FinalRot");
    }

    switch (currentOrder.goalType)
    {

    case GoalType::FINAL:
      log(INFO, "BR Transition : FinalRot -> Ready");

      // requestedState = BR_READY;
      // waitTimer.start( millis() );
      sendCallback(OK_TURN);
      transit<Ready>();
      break;

    default:
      // error
      log(ERROR, "Wrong goal type in state FinalRot");
    }
  };
};

// ----------------------------------------------------------------------------
// State: BR_Idle
//

class BR_Idle
    : public BrSM
{
  void entry() override
  {

    log(INFO, "Entered BR_IDLE from state" + getCurrentStateStr());
    sendCallback(OK_IDLE);

    currentState = BR_IDLE;
  }

  void react(BrUpdateEvent const &e) override
  {

    // check if we have to transit to Ready
    if (!isSupposedToBeIdle)
    {

      int motor_states[2];
      getCurrentMotorStates(motor_states);
      if (true || (motor_states[0] == AXIS_STATE_CLOSED_LOOP_CONTROL && motor_states[1] == AXIS_STATE_CLOSED_LOOP_CONTROL))  //FORTEST
      {

        log(INFO, "BR Transition : Idle -> Ready");
        transit<Ready>();

        currentGoalPos = PositionFeedback::instance().getRobotPosition();

        return;
      }
    }
  }

  void react(BrGetReadyEvent const &) override
  {

    // TODO check if there is no motor error
    // TODO check is calibration is needed or not

    // Send request for motors state to be Idle
    setMotorsToClosedLoop(); // NOTE without waiting

    // will trigger the transition when the Odrive response will be received
    isSupposedToBeIdle = false;
  }
};

// ----------------------------------------------------------------------------
// State: BR_RecalAsserv
//

class BR_RecalAsserv
    : public BrSM
{
  void entry() override
  {
    currentState = BR_RECAL_ASSERV;

    recalAsservTimer.start(millis());

    // Changement de trajectoire en linéaire
    currentTrajectory = std::make_unique<LinearTrajectory>();

    // Changement de la vitesse en vitesse de recalage
    currentTrajectory->setGoalSpeed(RECAL_SPEED);

    setupTrajectory();
  }

  void react(BrUpdateEvent const &e) override
  {
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

    // Timeout
    if (recalAsservTimer.isExpired(millis()))
    {
      log(ERROR, "Timeout for Recal, transit to Ready");
      transit<Ready>(); // TOTEST cette transition
      return;
    }

    Position2D robot_pos = PositionFeedback::instance().getRobotPosition();
    float d = (robot_pos - currentOrder).norm();

    if (d < RECAL_DISTANCE || // distance to destination is short enough
        m_switches[BR_RIGHT]->isSwitchPressed() ||
        m_switches[BR_LEFT]->isSwitchPressed())
    { // condition pour passer en recul commande

      transit<BR_RecalDetect>();
      return;
    }

    // else we follow a linear trajectory backwards (theta is towards the rear)

    // TODO factoriser le code dans une fct (commun avec le default updateEvent)
    if (!currentTrajectory)
    {
      log(FATAL, "Pointer to current trajectory is NULL in BR update function");
      return;
    }

    currentTrajectory->updateTrajectory(e.currentTime);
    currentGoalPos = currentTrajectory->getGoalPoint();

    Asserv::instance().updateError(toAsservPointFrame(currentGoalPos));

    Asserv::instance().updateCommand_2(
        currentTrajectory->getTrajectoryAbsoluteSpeed());
  }
};

// ----------------------------------------------------------------------------
// State: BR_RecalDetect
//

class BR_RecalDetect
    : public BrSM
{
  void entry() override
  {
    currentState = BR_RECAL_DETECT;
  }

  void react(BrUpdateEvent const &e) override
  {

    // On recule en commande et on analyse les bumpers
    float cmd_right = RECAL_SPEED;
    float cmd_left = RECAL_SPEED;

    if (m_switches[BR_RIGHT]->isSwitchPressed())
    {
      cmd_right = RECAL_SPEED / 3;
    }
    if (m_switches[BR_LEFT]->isSwitchPressed())
    {
      cmd_left = RECAL_SPEED / 3;
    }

    // if both bumpers are pressed we wait for a while and then transition
    if (m_switches[BR_RIGHT]->isSwitchPressed() &&
        m_switches[BR_LEFT]->isSwitchPressed())
    {

      // TODO

      // TODO stop motors (send command 0)
      // Asserv::instance().updateCommand_2(0.0, 0.0, ASSERV_BYPASSED);

      // TODO reset position on axis if needed

      transit<Ready>();
      return;

      // Send motor commands directly (right motor and left motor)
      sendMotorCommand(BR_RIGHT, cmd_right);
      sendMotorCommand(BR_LEFT, cmd_left);
    }
  }
};


// ----------------------------------------------------------------------------
// State: BR_EmergencyStop
//
class BR_EmergencyStop
    : public BrSM
{
  void entry() override
  {
    currentState = BR_EMERGENCYSTOP;
  }

  void react(BrUpdateEvent const &e) override
  {
    sendMotorCommand(BR_RIGHT, 0.0);
    sendMotorCommand(BR_LEFT, 0.0);
    transit<Ready>();
  }
};


// ----------------------------------------------------------------------------
// Base state: default implementations
//

void BrSM::react(OrderEvent const &)
{
  log(DEBUG, "OrderEvent ignored");
}

void BrSM::react(GoalReachedEvent const &)
{
  log(DEBUG, "GoalReachedEvent ignored");
}

void BrSM::react(ErrorEvent const &)
{
  log(DEBUG, "ErrorEvent ignored");
}

void BrSM::react(BrGetReadyEvent const &)
{
  log(DEBUG, "BrGetReadyEvent ignored");
}

void BrSM::react(BrSetToIdleEvent const &)
{
  log(DEBUG, "BrSetToIdleEvent ignored");
}

// We always take the reset position event
void BrSM::react(ResetPosEvent const &e)
{
  log(INFO, "Received reset position event to (" + ToString(e.x) + ", " +
                          ToString(e.y) + ", " + ToString(e.theta)+")");

  PositionFeedback::instance().resetPosition((Position2D<Millimeter>) e);

  // We also reset the currentGoalPos to make sure the asserv doesnt do madness
  currentGoalPos = convert((Position2D<Millimeter>) e);

  // RAZ l'intégrateur ?
  Asserv::instance().RAZIntegral();
}

void BrSM::react(BrEmergencyBrakeEvent const &)
{

  log(WARN, "Received emergency brake signal, stopping");

  // send signal to rampSM //TODO make it the same signal
  EmergencyBrakeEvent emergencyBrakeEvent;
  RampSM::dispatch(emergencyBrakeEvent);

  transit<BR_EmergencyStop>();
}

void BrSM::react(BrUpdateEvent const &e)
{

  if (!currentTrajectory)
  {
    log(FATAL, "Pointer to current trajectory is NULL in BR update function");
    return;
  }

  // if requested state not undef, means we have a pending transition
  // if (requestedState != BR_UNDEF) {

  //   if (waitTimer.isExpired( millis() )) {

  //     requestedState = BR_UNDEF;

  //     switch (requestedState) {
  //       case BR_FORWARD:
  //         transit<Forward>();
  //       case BR_FINALROT:
  //         transit<FinalRot>();
  //       case BR_READY:
  //         transit<Ready>();
  //       default:
  //         log(ERROR, "Requested state not handled");
  //     }
  //   }
  // }

  currentTrajectory->updateTrajectory(e.currentTime);
  currentGoalPos = currentTrajectory->getGoalPoint();

  // FORTEST caler le robot parfaitement sur la goalPos, si l'asserv est bypassed
  //  p_odos->setPosition(currentGoalPos);

  Asserv::instance().updateError(toAsservPointFrame(currentGoalPos));

  Asserv::instance().updateCommand_2(
      currentTrajectory->getTrajectoryAbsoluteSpeed());

  if (!currentTrajectory->isTrajectoryActive())
  {

    // Can mean that the trajectory is done if the asserv agrees
    if (Asserv::instance().isAtObjectivePoint(false) /* || true*/)
    { // TODO checkangle ?? //FORTEST

      // Serial.println("Send goal reached event");

      GoalReachedEvent e;
      e.goalType = currentOrder.goalType;
      dispatch(e);
    }

    // otherwise we let the asserv stabilize close to the end point
  }
}


// ----------------------------------------------------------------------------
// Getters
//

BRState BrSM::getCurrentState()
{
  return currentState;
}

string_t BrSM::getCurrentStateStr()
{
  return BrStateStr[currentState];
}

Position2D<Meter> BrSM::getCurrentGoalPos()
{
  return currentGoalPos;
}

float BrSM::getCurrentTargetSpeed()
{
  if (currentTrajectory->trajectoryType == TrajectoryType::TRAJ_LINEAR)
  {
    return currentTrajectory->getTrajectoryLinearSpeed();
  }
  else if (currentTrajectory->trajectoryType == TrajectoryType::TRAJ_ROTATION)
  {
    return currentTrajectory->getTrajectoryAngularSpeed();
  }
  else
  {
    // log(ERROR, "ERROR : Unhandled trajectory type");
    return 0.0;
  }
}

// Variable initializations
AxisStates BrSM::axisStates = {0};
OrderType BrSM::currentOrder = OrderType();
Position2D<Meter> BrSM::currentGoalPos = Position2D<Meter>();
std::unique_ptr<Trajectory> BrSM::currentTrajectory = std::unique_ptr<Trajectory>();
Timer BrSM::recalAsservTimer = Timer(millis());
Timer BrSM::waitTimer = Timer(millis());
SwitchFiltered *BrSM::m_switches[2] = {NULL};
bool BrSM::isSupposedToBeIdle = true;

// BrSM::current_state_ptr

// ----------------------------------------------------------------------------
// Initial state definition
//
BRState BrSM::currentState = BR_IDLE;
BRState	BrSM::requestedState = BR_UNDEF;


FSM_INITIAL_STATE(BrSM, BR_Idle)
