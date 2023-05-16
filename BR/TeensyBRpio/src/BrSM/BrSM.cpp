// #include "tinyfsm/tinyfsm.hpp"

#include "BrSM/BrSM.hpp"
// #include "fsmlist.hpp"

#include <Arduino.h>
#include <GeometricTools.hpp>
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
    FOREACH_BRSTATE(GENERATE_STRING)};

// forward declarations (all of them to have the list of states here)$
class Wait;
class Ready;
class InitRot;
class Forward;
class FinalRot;
class BR_Idle;
class BR_RecalAsserv;
class BR_RecalDetect;
class BR_EmergencyStop;

// constructor
BrSM::BrSM()
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
  Position2D robotPos = p_odos->getRobotPosition();
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
      thetaDest = atan2(currentOrder.y - robotPos.y, currentOrder.x - robotPos.x);
      break;
    }
    currentTrajectory->setDest(Position2D(0.0, 0.0, thetaDest));
    break;
  }

  case BR_FORWARD:
  {
    currentTrajectory->setDest(Position2D(currentOrder.x, currentOrder.y, 0.0));
    break;
  }

  case BR_FINALROT:
  {
    currentTrajectory->setDest(Position2D(0.0, 0.0, currentOrder.theta));
    break;
  }

  default:
  {
    p_ros->logPrint(ERROR, "Current state not handled in setDest");
    break;
  }
  }

  // si Dtotale vaut 0 (ou est proche de 0) c'est que l'ordre peut être ignoré
  // (pas besoin de se déplacer pour une trajectoire super courte)
  // On ignore alors l'étape et on passe à la suite en envoyant un GoalReachedEvent
  if (currentTrajectory->Dtotale < EPSILON_DTOTALE)
  {
    p_ros->logPrint(WARN, "Trajectory ignored because Dtotale is too small");
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
        p_ros->sendCallback(OK_READY);
        break;

      case BR_INITROT:
      case BR_FINALROT:
        p_ros->sendCallback(OK_TURN);
        break;

      case BR_FORWARD:
        p_ros->sendCallback(OK_POS);
        break;

      case BR_RECAL_DETECT:
        p_ros->sendCallback(OK_RECAL);
        break;

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

        p_ros->logPrint(INFO, "BR Transition : Ready -> Idle");
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

    p_asserv->updateError(toAsservPointFrame(currentGoalPos));

    float nullSpeed[2] = {0.0, 0.0};

    p_asserv->updateCommand_2(nullSpeed);
  }

  void react(OrderEvent const &e) override
  {

    // TODO : check if both axis are running (closed loop state)

    // store order
    currentOrder = e.order;
    p_ros->logPrint(INFO, "Order received : (" + String(currentOrder.x) + ", " +
                              String(currentOrder.y) + ", " + String(currentOrder.theta) +
                              ") with goalType " + String(currentOrder.goalType));

    // Transition depending on the order goal type
    switch (currentOrder.goalType)
    {

    case ORIENT:
    case TRANS:
    case FINAL:
      p_ros->logPrint(INFO, "BR Transition : Ready -> InitRot");
      transit<InitRot>();
      break;

    case RECAL_FRONT:
    case RECAL_BACK:
      p_ros->logPrint(INFO, "BR Transition : Ready -> RecalAsserv");
      transit<BR_RecalAsserv>();
      break;

    default:
      p_ros->logPrint(ERROR, "Order ignored because not recognized");
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
    currentTrajectory = p_rotationTrajectory;

    setupTrajectory();
  }

  void react(GoalReachedEvent const &e) override
  {

    // Check the goalType
    if (e.goalType != currentOrder.goalType)
    {
      p_ros->logPrint(ERROR, "Goal type does not match in state InitRot");
    }

    switch (currentOrder.goalType)
    {

    case GoalType::ORIENT:
      p_ros->logPrint(INFO, "BR Transition : InitRot -> Ready");

      requestedState = BR_READY;
      waitTimer.start( millis() );
      // transit<Ready>();
      break;

    case GoalType::TRANS:
    case GoalType::FINAL:
      p_ros->logPrint(INFO, "BR Transition : InitRot -> Forward");

      requestedState = BR_FORWARD;
      waitTimer.start( millis() );
      // transit<Forward>();
      break;

    default:
      // error
      p_ros->logPrint(ERROR, "Wrong goal type in state InitRot");
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
    currentTrajectory = p_linearTrajectory;

    setupTrajectory();
  }

  void react(GoalReachedEvent const &e) override
  {

    // Check the goalType
    if (e.goalType != currentOrder.goalType)
    {
      p_ros->logPrint(ERROR, "Goal type does not match in state Forward");
    }

    switch (currentOrder.goalType)
    {

    case GoalType::TRANS:
      p_ros->logPrint(INFO, "BR Transition : Forward -> Ready");

      requestedState = BR_READY;
      waitTimer.start( millis() );
      // transit<Ready>();
      break;

    case GoalType::FINAL:
      p_ros->logPrint(INFO, "BR Transition : Forward -> FinalRot");

      requestedState = BR_FINALROT;
      waitTimer.start( millis() );
      // transit<FinalRot>();
      break;

    default:
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
  void entry() override
  {
    currentState = BR_FINALROT;

    // Changement de trajectoire en rotation
    currentTrajectory = p_rotationTrajectory;

    setupTrajectory();
  }

  void react(GoalReachedEvent const &e) override
  {

    // Check the goalType
    if (e.goalType != currentOrder.goalType)
    {
      p_ros->logPrint(ERROR, "Goal type does not match in state FinalRot");
    }

    switch (currentOrder.goalType)
    {

    case GoalType::FINAL:
      p_ros->logPrint(INFO, "BR Transition : FinalRot -> Ready");

      requestedState = BR_READY;
      waitTimer.start( millis() );
      // transit<Ready>();
      break;

    default:
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
  void entry() override
  {

    p_ros->logPrint(INFO, "Entered BR_IDLE from state" + String(getCurrentStateStr()));
    p_ros->sendCallback(OK_IDLE);

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

        p_ros->logPrint(INFO, "BR Transition : Idle -> Ready");
        transit<Ready>();

        currentGoalPos = p_odos->getRobotPosition();

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
    currentTrajectory = p_linearTrajectory;

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
      p_ros->logPrint(ERROR, "Timeout for Recal, transit to Ready");
      transit<Ready>(); // TOTEST cette transition
      return;
    }

    Position2D robot_pos = p_odos->getRobotPosition();
    float d = sqrt((robot_pos.x - currentOrder.x) * (robot_pos.x - currentOrder.x) + (robot_pos.y - currentOrder.y) * (robot_pos.y - currentOrder.y));

    if (d < RECAL_DISTANCE || // distance to destination is short enough
        m_switches[BR_RIGHT]->isSwitchPressed() ||
        m_switches[BR_LEFT]->isSwitchPressed())
    { // condition pour passer en recul commande

      transit<BR_RecalDetect>();
      return;
    }

    // else we follow a linear trajectory backwards (theta is towards the rear)

    // TODO factoriser le code dans une fct (commun avec le default updateEvent)
    if (currentTrajectory == NULL)
    {
      p_ros->logPrint(FATAL, "Pointer to current trajectory is NULL in BR update function");
      return;
    }

    currentTrajectory->updateTrajectory(e.currentTime);
    currentGoalPos = currentTrajectory->getGoalPoint();

    p_asserv->updateError(toAsservPointFrame(currentGoalPos));

    p_asserv->updateCommand_2(
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
      // p_asserv->updateCommand_2(0.0, 0.0, ASSERV_BYPASSED);

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

    // TODO
  }
};


// ----------------------------------------------------------------------------
// Base state: default implementations
//

void BrSM::react(OrderEvent const &)
{
  p_ros->logPrint(DEBUG, "OrderEvent ignored");
}

void BrSM::react(GoalReachedEvent const &)
{
  p_ros->logPrint(DEBUG, "GoalReachedEvent ignored");
}

void BrSM::react(ErrorEvent const &)
{
  p_ros->logPrint(DEBUG, "ErrorEvent ignored");
}

void BrSM::react(BrGetReadyEvent const &)
{
  p_ros->logPrint(DEBUG, "BrGetReadyEvent ignored");
}

void BrSM::react(BrSetToIdleEvent const &)
{
  p_ros->logPrint(DEBUG, "BrSetToIdleEvent ignored");
}

// We always take the reset position event
void BrSM::react(ResetPosEvent const &e)
{
  p_ros->logPrint(INFO, "Received reset position event to (" + String(e.x) + ", " +
                          String(e.y) + ", " + String(e.theta)+")");

  p_odos->setPosition(Position2D(e.x, e.y, e.theta));

  // We also reset the currentGoalPos to make sure the asserv doesnt do madness
  currentGoalPos.x = e.x;
  currentGoalPos.y = e.y;
  currentGoalPos.theta = e.theta;

  // RAZ l'intégrateur ?
  p_asserv->RAZIntegral();
}

void BrSM::react(BrEmergencyBrakeEvent const &)
{

  p_ros->logPrint(WARN, "Received emergency brake signal, stopping");

  // send signal to rampSM //TODO make it the same signal
  EmergencyBrakeEvent emergencyBrakeEvent;
  currentTrajectory->rampSpeed.rampSM.send_event(emergencyBrakeEvent);

  transit<BR_EmergencyStop>();
}

void BrSM::react(BrUpdateEvent const &e)
{

  if (currentTrajectory == NULL)
  {
    p_ros->logPrint(FATAL, "Pointer to current trajectory is NULL in BR update function");
    return;
  }

  // if requested state not undef, means we have a pending transition
  if (requestedState != BR_UNDEF) {

    if (waitTimer.isExpired( millis() )) {

      requestedState = BR_UNDEF;

      switch (requestedState) {
        case BR_FORWARD:
          transit<Forward>();
        case BR_FINALROT:
          transit<FinalRot>();
        case BR_READY:
          transit<Ready>();
        default:
          p_ros->logPrint(ERROR, "Requested state not handled");
      }
    }
  }

  currentTrajectory->updateTrajectory(e.currentTime);
  currentGoalPos = currentTrajectory->getGoalPoint();

  // FORTEST caler le robot parfaitement sur la goalPos, si l'asserv est bypassed
  //  p_odos->setPosition(currentGoalPos);

  p_asserv->updateError(toAsservPointFrame(currentGoalPos));

  p_asserv->updateCommand_2(
      currentTrajectory->getTrajectoryAbsoluteSpeed());

  if (!currentTrajectory->isTrajectoryActive())
  {

    // Can mean that the trajectory is done if the asserv agrees
    if (p_asserv->isAtObjectivePoint(false) /* || true*/)
    { // TODO checkangle ?? //FORTEST

      // Serial.println("Send goal reached event");

      GoalReachedEvent e;
      e.goalType = currentOrder.goalType;
      send_event(e);
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

String BrSM::getCurrentStateStr()
{
  return BrStateStr[currentState];
}

Position2D BrSM::getCurrentGoalPos()
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
    // p_ros->logPrint(ERROR, "ERROR : Unhandled trajectory type");
    return 0.0;
  }
}

// Variable initializations
AxisStates BrSM::axisStates = {0};
OrderType BrSM::currentOrder = {0};
Position2D BrSM::currentGoalPos = Position2D(0.0, 0.0, 0.0);
Trajectory *BrSM::currentTrajectory = NULL;
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
