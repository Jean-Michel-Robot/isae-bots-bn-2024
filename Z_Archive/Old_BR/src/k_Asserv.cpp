/*
   centre du code d'asserv
   est appelé par la loop arduino, et gére l'execution de toutes les morceaux de l'asserv vus avant
   calcule l'erreur a utiliser par l'asserv en position, et calcule les consignes de vitesse a envoyer à l'asserv moteurs
*/
#include "a_geometric_tools.h"
#include "a_Led_RGB.h"
#include "a_define.h"
#include "g_Asserv_Motors.h"
#include "i_etat_asserv.h"
#include "k_Asserv.h"
#include "u_ROS.h"
#include "v_Logger.h"
#include "p_Odos.h"
#include "z_Setup_Loop.h"



AsservPositionTask::AsservPositionTask(TaskType type):Task(type)
{
}

void AsservPositionTask::_loop()
{
  unsigned long t_micro = micros();
  AsservObjectif objectif = machineAEtatAsservInstance->updateStateAndComputeAsservObjective();
  Logger::setFieldValue(objectif.erreurAvance, Logger::erreurDistAsserv);
  Logger::setFieldValue(radToDeg(objectif.erreurTourne), Logger::erreurAngleAsserv);
  switch (objectif.type)
  {
    case OBJECTIF_POSITION :
      {
        float avancer = 0.0;
        float tourner = 0.0;
        avancer = m_asservDist.computeOutput(objectif.erreurAvance, t_micro) + objectif.feedForwardAvancer;
        tourner = m_asservAngle.computeOutput(objectif.erreurTourne, t_micro) + objectif.feedForwardTourner;
        float goalSpeedL = (AsservMoteursTask::PRECOMMANDE_AVANCE * avancer - AsservMoteursTask::PRECOMMANDE_ROTATION * tourner);
        float goalSpeedR = AsservMoteursTask::PRECOMMANDE_AVANCE * avancer + AsservMoteursTask::PRECOMMANDE_ROTATION * tourner;
        asservMoteurLeftTask->setSpeedObjective(goalSpeedL);
        asservMoteurRightTask->setSpeedObjective(goalSpeedR);
      }
      break;
    case OBJECTIF_VITESSE :
      {
        float goalSpeedL = objectif.speedLeft;
        float goalSpeedR = objectif.speedRight;
        asservMoteurLeftTask->setSpeedObjective(goalSpeedL);
        asservMoteurRightTask->setSpeedObjective(goalSpeedR);
        m_asservAngle.RAZ(t_micro);
        m_asservDist.RAZ(t_micro);
      }
      break;
    case OBJECTIF_COMMANDE :
      {
        asservRAZ();
        asservMoteurLeftTask->byPass(objectif.commandeLeft);
        asservMoteurRightTask->byPass(objectif.commandeRight);
      }
      break;
  }
}

void AsservPositionTask::changeGains(float KP, float TI, float TD, float KPa, float TIa, float TDa)
{
  m_asservDist.setGains(KP, TI, TD);
  m_asservAngle.setGains(KPa, TIa, TDa);
}

void AsservPositionTask::asservRAZ()
{ // on vide les intégrales des 4 PID
  unsigned long time = micros();
  m_asservDist.RAZ(time);
  m_asservAngle.RAZ(time);
  asservMoteurLeftTask->RAZ();
  asservMoteurRightTask->RAZ();
}

void AsservPositionTask::setIStatusOnPID(bool isIDist, bool isIAngle)
{
  m_asservDist.setI(isIDist);
  m_asservAngle.setI(isIAngle);
}

bool AsservPositionTask::areGainsSet() const
{
  return m_asservDist.areGainsOk() && m_asservAngle.areGainsOk();
}

float AsservPositionTask::getKPAngleGain() const
{
  return m_asservAngle.getKP();
}
