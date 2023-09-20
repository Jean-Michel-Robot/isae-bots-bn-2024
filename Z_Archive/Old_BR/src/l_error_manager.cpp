//
#include "a_define.h"
#include "a_geometric_tools.h"
#include "e_odos_motor.h"
#include "i_etat_asserv.h"
#include "l_error_manager.h"
#include "p_Odos.h"
#include "u_ROS.h"
#include "src/FilterLowPass/FilterLowPass.h"
#include "z_Setup_Loop.h"

ErrorRaiserTask::ErrorRaiserTask(TaskType type):Task(type)
{

}

void ErrorRaiserTask::_loop()
{
#ifdef ASSERV_MOTEURS
  float deltaT = float(micros() - m_lastTime) * 1e-6;
  m_lastTime = micros();
  float omegaUnfiltered = (odosPositionTask->getRobotPosition().theta - m_lastPos.theta) / deltaT;
  float omega = m_filterOmega.computeOutput(omegaUnfiltered, micros());
  Position2D vitesseUnFiltered = (odosPositionTask->getRobotPosition() - m_lastPos)/deltaT;
  vitesseUnFiltered.changeReferentiel(odosPositionTask->getRobotPosition());
  float vitesse = m_filterVit.computeOutput(vitesseUnFiltered.norm(), micros()); // on projette le vecteur vitesse dans l'axe du robot
  m_speedLEstimatedWithPosition = vitesse - 135 * omega;
  m_speedREstimatedWithPosition = vitesse + 135 * omega;
  m_lastPos = odosPositionTask->getRobotPosition();
  m_speedLEstimatedWithMotors = m_filterLDelay.computeOutput(odosMoteursTask->getSpeedL(), micros());
  m_speedREstimatedWithMotors = m_filterRDelay.computeOutput(odosMoteursTask->getSpeedR(), micros());

  if (!machineAEtatAsservInstance->isWheelAllowedToDrift())
  {
    if (std::abs(m_speedLEstimatedWithMotors - m_speedLEstimatedWithPosition) > max(SEUIL_DERAPAGE, SEUIL_DERAPAGE_RAPPORT * std::abs(m_speedLEstimatedWithMotors)))
    {
      if (m_timerOutPatinage.startIfNotStartedAndTestExpiration(millis())) // onlance une erreur
        machineAEtatAsservInstance->error("derapage roue gauche");
    }
    else if (std::abs(m_speedREstimatedWithMotors - m_speedREstimatedWithPosition) > max(SEUIL_DERAPAGE, SEUIL_DERAPAGE_RAPPORT * std::abs(m_speedREstimatedWithMotors)))
    {
      if (m_timerOutPatinage.startIfNotStartedAndTestExpiration(millis())) // onlance une erreur
        machineAEtatAsservInstance->error("derapage roue droite");
    }
    else // aucun patinage détecté, tout va bien
      m_timerOutPatinage.reset();
  }
#endif
}
