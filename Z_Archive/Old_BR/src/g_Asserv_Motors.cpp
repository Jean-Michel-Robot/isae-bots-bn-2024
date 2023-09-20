/*
  gere l'asserv des moteurs, recoit la vitesse consigne et envoie les commandes aux moteurs
*/

#include "a_define.h"
#include "e_odos_motor.h"
#include "i_etat_asserv.h"
#include "g_Asserv_Motors.h"
#include "v_Logger.h"
#include "z_Setup_Loop.h"

AsservMoteursTask::AsservMoteursTask(TaskType type, GenericMotor::Side side, GenericMotor::DirectionMoteur direction):GenericAsservMoteurs(type, side, direction)
{
}

void AsservMoteursTask::_loop()
{
  if (m_isAsservMotorByPassed)
  { // en cas de controle par la manette pas d'asserv
    return;
  }
  if (!machineAEtatAsservInstance->isAsservWorking() && machineAEtatAsservInstance->getGoalOrder().goalType != MachineAEtatAsserv::CONTROL_SPEED)
  { // si on impose un stop les moteurs sont à 0
    m_motorItf->commandMotor(0);
    m_asservMotor.RAZ(micros());
    return;
  }
  static unsigned long lastTime = micros()-1;
  unsigned long times = micros();// on calcule la derivée de la commande sur chaque moteur
  static float oldGoalSpeed = m_goalSpeed;

  float derivCom = m_filterComD.computeOutput((m_goalSpeed - oldGoalSpeed) / (times - lastTime) * 1e6, times);
  oldGoalSpeed = m_goalSpeed ;
  lastTime = times;
  float motor = 0.0;
  if(m_motorItf->isLeft()){
    motor = m_asservMotor.computeOutputWithDerivateOfError(m_goalSpeed - odosMoteursTask->getSpeedL(), /*derivCom - odosMoteursTask->getAccelL()*/0.0, times); // on applique les PID
    motor = int(constrain(motor + m_goalSpeed * m_Kf_motor, -CytronMotorBR::MAX_COMMAND_MOTOR, CytronMotorBR::MAX_COMMAND_MOTOR));
  }
  else{
    motor = m_asservMotor.computeOutputWithDerivateOfError(m_goalSpeed - odosMoteursTask->getSpeedR(), /*derivCom - odosMoteursTask->getAccelR()*/0.0, times);
    motor = int(constrain(motor + m_goalSpeed * m_Kf_motor, -CytronMotorBR::MAX_COMMAND_MOTOR, CytronMotorBR::MAX_COMMAND_MOTOR)); // on prend l'opposé car le moteur est retourné
  }
  m_motorItf->commandMotor(motor);
}

void AsservMoteursTask::RAZ()
{ // vide les I et D
  m_asservMotor.RAZ(micros());
}

void AsservMoteursTask::changeGains(float Kf, float Kp, float Ti, float Td)
{ // change les 4 gains
  m_asservMotor.setGains(Kp, Ti, Td);
  m_Kf_motor = Kf;
}

