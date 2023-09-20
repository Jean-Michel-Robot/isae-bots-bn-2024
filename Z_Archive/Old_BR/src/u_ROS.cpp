#include "a_Led_RGB.h"
#include "a_define.h"
#include "e_odos_motor.h"
#include "g_Asserv_Motors.h"
#include "i_etat_asserv.h"
#include "k_Asserv.h"
#include "l_error_manager.h"
#include "p_Odos.h"
#include "u_ROS.h"
#include "v_Logger.h"
#include "z_Setup_Loop.h"

#include "ros.h"
#include <std_msgs/Int16.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

ROSTask::ROSTask(TaskType type):Task(type)
{
  m_nodeHandle.initNode();
  m_nodeHandle.subscribe(m_subGainsP);
  m_nodeHandle.subscribe(m_subOrder);
  m_nodeHandle.subscribe(m_subGainsM);
  m_nodeHandle.subscribe(m_subAcc);
  m_nodeHandle.subscribe(m_subAcc2);
  m_nodeHandle.subscribe(m_subSpeed);

  m_nodeHandle.advertise(m_positionFeedback);
  m_nodeHandle.advertise(m_okFeedback);
  m_nodeHandle.advertise(m_logTotale);

  m_nodeHandle.getHardware()->setBaud(250000);
}

void ROSTask::_loop()
{
  publishFullLogs();
  m_nodeHandle.spinOnce(); // boucle ros
}

void ROSTask::s_goToCb(const geometry_msgs::Quaternion& positionMsg)
{ // goToCallBack
  if (int(positionMsg.w) >= MachineAEtatAsserv::UNVALID_GOALTYPE) // on ignore le message si l'id n'est pas valide
    return;
  machineAEtatAsservInstance->manageNewOrder(Position2D(positionMsg.x,positionMsg.y,positionMsg.z),(MachineAEtatAsserv::GoalType) int(positionMsg.w));
}

void ROSTask::s_changeGainsPosition(const std_msgs::Float32MultiArray& gains)
{
  asservPositionTask->changeGains(gains.data[0], gains.data[1], gains.data[2], gains.data[3], gains.data[4], gains.data[5]);
  asservPositionTask->asservRAZ();
}

void ROSTask::s_changeGainsMotor(const std_msgs::Float32MultiArray& gainsM)
{
  if(gainsM.data[4]){
    asservMoteurLeftTask->changeGains(gainsM.data[0], gainsM.data[1], gainsM.data[2], gainsM.data[3]);
  }
  else{
    asservMoteurRightTask->changeGains(gainsM.data[0], gainsM.data[1], gainsM.data[2], gainsM.data[3]);
  }
}

void ROSTask::s_setSpeed(const std_msgs::Float32MultiArray& speeds)
{
    machineAEtatAsservInstance->getRampePosition()->setSpeed(speeds.data[0],timeFloat());
    machineAEtatAsservInstance->getRampeOrientation()->setSpeed(speeds.data[1],timeFloat());
}

void ROSTask::s_changeAccDecRampe(const std_msgs::Float32MultiArray& gains)
{
    machineAEtatAsservInstance->getRampePosition()->setAccDecc(gains.data[0], gains.data[1], gains.data[2]);
    machineAEtatAsservInstance->getRampeOrientation()->setAccDecc(gains.data[3], gains.data[4], gains.data[4]);
}


void ROSTask::s_changeAccDecRampePrecise(const std_msgs::Float32MultiArray& gains)
{
    machineAEtatAsservInstance->getRampePosition()->setAccDecc(gains.data[0], gains.data[1], gains.data[2], gains.data[3], gains.data[4], gains.data[5]);
    machineAEtatAsservInstance->getRampeOrientation()->setAccDecc(gains.data[6], gains.data[7],gains.data[7]);
}

void ROSTask::sendOkPos()
{
  m_feedbackOk.data = 1;
  m_okFeedback.publish( &m_feedbackOk );
}

void ROSTask::sendOkTurn()
{
  m_feedbackOk.data = 2;
  m_okFeedback.publish( &m_feedbackOk );
}

void ROSTask::confirmMarcheArriere()
{
  m_feedbackOk.data = 3;
  m_okFeedback.publish( &m_feedbackOk );
}

void ROSTask::confirmMarcheAvant()
{
  m_feedbackOk.data = 4;
  m_okFeedback.publish( &m_feedbackOk );
}

void ROSTask::errorAsserv(String details)
{ // erreur dans l'asserv qui peut necessiter une tentative supplementaire
  m_feedbackOk.data = 0;
  m_okFeedback.publish( &m_feedbackOk );
  m_nodeHandle.logerror(details.c_str());
}

void ROSTask::errorAsservNotSet(String details)
{ // erreur dans l'asserv bloquante (manque de gains, paramètres foirées ...)
  m_feedbackOk.data = -1;
  m_okFeedback.publish( &m_feedbackOk );
  m_nodeHandle.logerror(details.c_str());
}

void ROSTask::sendCurrentPosition()
{
  m_feedbackPosition.x = odosPositionTask->getRobotPosition().x;
  m_feedbackPosition.y = odosPositionTask->getRobotPosition().y;
  m_feedbackPosition.theta = odosPositionTask->getRobotPosition().theta;
  m_positionFeedback.publish( &m_feedbackPosition );
}

void ROSTask::logPrint(String msg)
{
  m_nodeHandle.loginfo(msg.c_str());
}

void ROSTask::publishFullLogs()
{
  static float lastTime = 0.0;
  unsigned long time = micros();
  if (time - lastTime > 4e4) { // on envoie tous les 20 ms
    float* tab = Logger::getArrayOfValues();
    m_logTotalArray.data = tab;
    m_logTotalArray.data_length = Logger::NbOfFields;
    m_logTotale.publish(&m_logTotalArray);
    lastTime = time;
  }
}
