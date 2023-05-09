// #include "a_Led_RGB.h"
// #include "a_define.h"
// #include "e_odos_motor.h"
// #include "g_Asserv_Motors.h"
// #include "i_etat_asserv.h"
// #include "k_Asserv.h"
// #include "l_error_manager.h"
// #include "p_Odos.h"
// #include "u_ROS.h"
// #include "v_Logger.h"
// #include "z_Setup_Loop.h"
#include "ROS.hpp"

#include <Motors.hpp>

#include "main_loop.hpp"

#include "BrSM/BrSMWrapper.hpp"
#include "Asserv.hpp"

// init ROS object
ROS::ROS()
{
    m_nodeHandle.initNode();
    // m_nodeHandle.subscribe(m_subGainsP);
    m_nodeHandle.subscribe(m_subOrder);
    m_nodeHandle.subscribe(m_subDebug);
    // m_nodeHandle.subscribe(m_subGainsM);
    // m_nodeHandle.subscribe(m_subAcc);
    // m_nodeHandle.subscribe(m_subAcc2);
    // m_nodeHandle.subscribe(m_subSpeed);

    m_nodeHandle.advertise(m_positionFeedback);

    m_nodeHandle.advertise(m_odosTicksPub);
    // m_nodeHandle.advertise(m_okFeedback);
    // m_nodeHandle.advertise(m_logTotale);

    m_nodeHandle.getHardware()->setBaud(250000);
}




void ROS::loop()
{
//   publishFullLogs();
  m_nodeHandle.spinOnce(); // boucle ros
}


// callback to an order
void ROS::s_goToCb(const geometry_msgs::Quaternion& positionMsg)
{

  int goalType = (int) positionMsg.w;

  // Action depending on goal type
  switch (goalType) {

    case GoalType::FINAL:
    case GoalType::TRANS:
    case GoalType::ORIENT:

    case GoalType::RECAL_BACK:
    case GoalType::RECAL_FRONT:

    case GoalType::CONTROL:

      OrderEvent orderEvent;

      orderEvent.order.x = positionMsg.x;
      orderEvent.order.y = positionMsg.y;
      orderEvent.order.theta = positionMsg.z;
      orderEvent.order.goalType = positionMsg.w;

      p_sm->send_event(orderEvent);
      break;


    case GoalType::RESET:

      ResetPosEvent resetPosEvent;
      resetPosEvent.x = positionMsg.x;
      resetPosEvent.y = positionMsg.y;
      resetPosEvent.theta = positionMsg.z;

      p_sm->send_event(resetPosEvent);
      break;


    case GoalType::STOP:

      EmergencyBrakeEvent emergencyBrakeEvent;

      p_sm->send_event(emergencyBrakeEvent);
      break;


    default:
      // order ignored
      break;
  }

}


void ROS::logPrint(LogType logtype, String msg)
{

  Serial.println(msg);
  return; //NOTE used to debug with serial interface

  if (logtype == INFO) {m_nodeHandle.loginfo(msg.c_str());}
  else if (logtype == WARN) {m_nodeHandle.logwarn(msg.c_str());}
  else if (logtype == ERROR) {m_nodeHandle.logerror(msg.c_str());}
  else if (logtype == FATAL) {m_nodeHandle.logfatal(msg.c_str());}
  else if (logtype == DEBUG) {m_nodeHandle.logdebug(msg.c_str());}
  else {m_nodeHandle.logerror("Unknown log type");}
}


void ROS::s_debug(const std_msgs::Int16& debugMsg)
{
  int res = debugMsg.data;

  if (res == 1) {
    OrderEvent orderEvent;
    orderEvent.order = {.x = 500, .y = 0, .theta = 0.0, .goalType = GoalType::TRANS};

    p_sm->send_event(orderEvent);
  }
}

// void ROS::s_goToCb(const geometry_msgs::Quaternion& positionMsg)
// { // goToCallBack
//   if (int(positionMsg.w) >= MachineAEtatAsserv::UNVALID_GOALTYPE) // on ignore le message si l'id n'est pas valide
//     return;
//   machineAEtatAsservInstance->manageNewOrder(Position2D(positionMsg.x,positionMsg.y,positionMsg.z),(MachineAEtatAsserv::GoalType) int(positionMsg.w));
// }

void ROS::s_changeGains(const std_msgs::Float32MultiArray& gains)
{
  p_asserv->setGains(gains.data[0], gains.data[1], gains.data[2]);
  //TODO RAZ de l'asserv ?
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

// void ROS::s_setSpeed(const std_msgs::Float32MultiArray& speeds)
// {
//     machineAEtatAsservInstance->getRampePosition()->setSpeed(speeds.data[0],timeFloat());
//     machineAEtatAsservInstance->getRampeOrientation()->setSpeed(speeds.data[1],timeFloat());
// }


void ROS::sendCallback(CallbackHN callback)
{
  m_callbackHN.data = callback;
  m_pubHN.publish( &m_callbackHN );
}

// void ROS::sendOkTurn()
// {
//   m_callbackHN.data = 2;
//   m_pubHN.publish( &m_callbackHN );
// }

// void ROS::confirmMarcheArriere()
// {
//   m_feedbackOk.data = 3;
//   m_okFeedback.publish( &m_feedbackOk );
// }

// void ROS::confirmMarcheAvant()
// {
//   m_feedbackOk.data = 4;
//   m_okFeedback.publish( &m_feedbackOk );
// }

// void ROS::errorAsserv(String details)
// { // erreur dans l'asserv qui peut necessiter une tentative supplementaire
//   m_feedbackOk.data = 0;
//   m_okFeedback.publish( &m_feedbackOk );
//   m_nodeHandle.logerror(details.c_str());
// }

// void ROS::errorAsservNotSet(String details)
// { // erreur dans l'asserv bloquante (manque de gains, paramètres foirées ...)
//   m_feedbackOk.data = -1;
//   m_okFeedback.publish( &m_feedbackOk );
//   m_nodeHandle.logerror(details.c_str());
// }

void ROS::sendCurrentPosition(Position2D position)
{
    m_feedbackPosition.x = position.x;
    m_feedbackPosition.y = position.y;
    m_feedbackPosition.theta = position.theta;
    m_positionFeedback.publish( &m_feedbackPosition );
}


// void ROS::publishFullLogs()
// {
//   static float lastTime = 0.0;
//   unsigned long time = micros();
//   if (time - lastTime > 4e4) { // on envoie tous les 20 ms
//     float* tab = Logger::getArrayOfValues();
//     m_logTotalArray.data = tab;
//     m_logTotalArray.data_length = Logger::NbOfFields;
//     m_logTotale.publish(&m_logTotalArray);
//     lastTime = time;
//   }
// }
