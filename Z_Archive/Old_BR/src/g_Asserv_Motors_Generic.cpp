/*
  gere l'asserv des moteurs, recoit la vitesse consigne et envoie les commandes aux moteurs
*/
#include "a_define.h"
#include "e_odos_motor.h"
#include "f_Motors.h"
#include "i_etat_asserv.h"
#include "g_Asserv_Motors_Generic.h"
#include "v_Logger.h"
#include "z_Setup_Loop.h"


#ifdef __SIMU__
#include "../Simulation/CytronSimulator.h"
#endif
GenericAsservMoteurs::GenericAsservMoteurs(TaskType type, GenericMotor::Side side, GenericMotor::DirectionMoteur direction):Task(type),
#ifdef __SIMU__
m_motorItf(new SimuMotor(side,direction))
#else
m_motorItf(new CytronMotorBR(side,direction))
#endif
{
  m_Kf_motor = m_motorItf->isLeft() ? KF_L : KF_R;
}

FakeAsservMoteursTask::FakeAsservMoteursTask(TaskType type, GenericMotor::Side side, GenericMotor::DirectionMoteur direction):GenericAsservMoteurs(type, side, direction)
{
}



void FakeAsservMoteursTask::_loop()
{
  // if (m_isAsservMotorByPassed)
  // { // en cas de controle par la manette pas d'asserv
  //   return;
  // }
  float motor = int(constrain(m_goalSpeed * m_Kf_motor, -CytronMotorBR::MAX_COMMAND_MOTOR, CytronMotorBR::MAX_COMMAND_MOTOR));
  m_motorItf->commandMotor(m_goalSpeed);
}

void FakeAsservMoteursTask::changeGains(float Kf, float , float , float )
{ // change le gain direct uniquement, les autres sont ignorés
  m_Kf_motor = Kf;
}

void GenericAsservMoteurs::byPass(int commandMotor)
{
  setSpeedObjective(0.0);
  m_isAsservMotorByPassed = true;
  m_motorItf->commandMotor(commandMotor);
}

void GenericAsservMoteurs::setSpeedObjective(float speedGoal)
{
  m_goalSpeed = speedGoal;
  m_isAsservMotorByPassed = false;
  if(m_motorItf->isLeft()){
    Logger::setFieldValue(m_goalSpeed, Logger::goalSpeedL);
  }
  else{
    Logger::setFieldValue(m_goalSpeed, Logger::goalSpeedR);
  }
}

float GenericAsservMoteurs::getAbsSpeedObjective() const
{
  return std::abs(m_goalSpeed);
}
