
#include "ros/ROS.hpp"
#include "logging.h"
#include "utils/clock.h"

#include "trajectories/Trajectory.hpp"
#include "state_machine/BrSMWrapper.hpp"
#include "state_machine/RampSM.hpp"
#include "state_machine/Callback.hpp"
#include "Asserv.hpp"

void ROS::loop()
{
  publishFullLogs();
  spin();
}

// callback to an order
void ROS::s_goToCb(const geometry_msgs::Quaternion *positionMsg)
{

  int goalType = (int)positionMsg->w;

  // Action depending on goal type
  switch (goalType)
  {

  case GoalType::FINAL:
  case GoalType::TRANS:
  case GoalType::ORIENT:
  case GoalType::REVERSE:

  case GoalType::RECAL_BACK:
  case GoalType::RECAL_FRONT:

  case GoalType::CONTROL:
  {

    Position2D<Millimeter> position(positionMsg->x, positionMsg->y, positionMsg->z);

    OrderEvent orderEvent;
    orderEvent.order = OrderType(convert(position), positionMsg->w);

    BrSM::dispatch(orderEvent);
    break;
  }

  case GoalType::RESET:
  {

    ResetPosEvent resetPosEvent;
    resetPosEvent.x = positionMsg->x;
    resetPosEvent.y = positionMsg->y;
    resetPosEvent.theta = positionMsg->z;

    BrSM::dispatch(resetPosEvent);
    break;
  }

  case GoalType::STOP:
  {

    BrEmergencyBrakeEvent brEmergencyBrakeEvent;

    BrSM::dispatch(brEmergencyBrakeEvent);
    break;
  }

  default:
    // order ignored
    break;
  }
}

void ROS::s_debug(const std_msgs::Int16 *debugMsg)
{
  int res = debugMsg->data;

  if (res == 0)
  {
    OrderEvent orderEvent;
    orderEvent.order = OrderType(2.0, 0, 0.0, GoalType::TRANS);

    BrSM::dispatch(orderEvent);
  }

  if (res == 1)
  {
    OrderEvent orderEvent;
    orderEvent.order = OrderType(0.0, 0, 0.0, GoalType::TRANS);

    BrSM::dispatch(orderEvent);
  }

  else if (res == 2)
  {
    GoalSpeedChangeEvent goalSpeedChangeEvent;
    goalSpeedChangeEvent.newSpeed = 0.5;

    RampSM::dispatch(goalSpeedChangeEvent);
  }

  else if (res == 3)
  {
    GoalSpeedChangeEvent goalSpeedChangeEvent;
    goalSpeedChangeEvent.newSpeed = 0.1;

    RampSM::dispatch(goalSpeedChangeEvent);
  }

  else if (res == 4)
  {
    EmergencyBrakeEvent emergencyBrakeEvent;

    RampSM::dispatch(emergencyBrakeEvent);
  }

  else if (res == 5)
  {
    EndRampEvent endRampEvent;

    RampSM::dispatch(endRampEvent);
  }
}

void ROS::s_changeGains(const std_msgs::Float32MultiArray *gains)
{
  const float *gains_ = multi_array_get_raw_data(gains);
  Asserv::instance().setGains(gains_[0], gains_[1], gains_[2]);
  //TODO RAZ de l'asserv ?
}

void ROS::s_setSpeed(const std_msgs::Int16 *speedMsg)
{
  if (!BrSM::currentTrajectory)
  {
    return;
  }

  float newSpeedFactor = (float)speedMsg->data / 100.0; // data is a percentage

  switch (BrSM::currentTrajectory->trajectoryType)
  {

  case TRAJ_UNDEF: //TODO handle differently ?
  case TRAJ_LINEAR:
    BrSM::currentTrajectory->setGoalSpeed(newSpeedFactor * MAX_LINEAR_GOAL_SPEED);
    break;

  case TRAJ_ROTATION:
    BrSM::currentTrajectory->setGoalSpeed(newSpeedFactor * MAX_ROTATION_GOAL_SPEED);
    break;

  default:
    log(ERROR, "Unhandled trajectory type in setSpeed callback");
    break;
  }
}

void ROS::sendDebug()
{

  Position2D<Meter> pos = BrSM::currentTrajectory->getGoalPoint();

  m_debugVar.x = pos.x;
  m_debugVar.y = pos.y;
  m_debugVar.z = pos.theta;
  m_debugVar.w = 0;

  m_debugPub.publish(m_debugVar);
}

// void ROS::s_changeGainsMotor(const std_msgs::Float32MultiArray& gainsM)
// {
//   if(gainsM.data[4]){
//     asservMoteurLeftTask->changeGains(gainsM.data[0], gainsM.data[1], gainsM.data[2], gainsM.data[3]);
//   }
//   else{
//     asservMoteurRightTask->changeGains(gainsM.data[0], gainsM.data[1], gainsM.data[2], gainsM.data[3]);
//   }
// }

void sendCallback(AsservCallback callback)
{
  ROS::instance().sendCallback(callback);
}

void ROS::sendCallback(AsservCallback callback)
{
  m_callbackHN.data = callback;
  m_pubHN.publish(m_callbackHN);
}

void ROS::s_idle(const std_msgs::Int16 *msg)
{

  if (msg->data == 1)
  {
    log(INFO, "Received get ready event");

    BrGetReadyEvent brGetReadyEvent;
    BrSM::dispatch(brGetReadyEvent);
  }

  else if (msg->data == 0)
  {
    log(INFO, "Received set to Idle event");

    BrSetToIdleEvent brSetToIdleEvent;
    BrSM::dispatch(brSetToIdleEvent);
  }
}

void ROS::sendCurrentPosition(Position2D<Millimeter> position)
{
  m_feedbackPosition.x = position.x;
  m_feedbackPosition.y = position.y;
  m_feedbackPosition.theta = position.theta;
  m_positionFeedback.publish(m_feedbackPosition);
}

void ROS::sendOdosCounts(int32_t left, int32_t right)
{
  int32_t ticks[2] = {left, right};
  multi_array_set_data(m_odosTicks, ticks, 2);
  m_odosTicksPub.publish(m_odosTicks);
}

void ROS::publishFullLogs()
{
  static uint32_t lastTime = 0.0;

  uint32_t time = millis();

  if (time - lastTime > LOG_PERIOD)
  {
    float *tab = Logger::getArrayOfValues();
    multi_array_set_data(m_logTotalArray, tab, Logger::NbOfFields);
    m_logTotale.publish(m_logTotalArray);
    lastTime = time;
  }
}

void log(const LogType type, string_t message)
{
  ROS::instance().logPrint(type, message);
}
