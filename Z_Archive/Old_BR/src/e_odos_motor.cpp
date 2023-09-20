/*
   Gere les odometres moteurs, permet de maintenir à jour les vitesses currentSpeedMotor

*/
#include "a_define.h"
#include "e_odos_motor.h"
#include "v_Logger.h"
#include "z_Setup_Loop.h"

OdosMoteursTask::OdosMoteursTask(TaskType type):Task(type)
{
#ifdef ASSERV_MOTEURS
  m_knobLeft.write(0);
  m_knobRight.write(0);
#endif
}

void OdosMoteursTask::_loop()
{
//#ifdef ASSERV_MOTEURS
  unsigned long time_micros = micros();

  int32_t odoLM =  m_knobLeft.read();
  int32_t odoRM = -m_knobRight.read();
  Logger::setFieldValue(odoLM, Logger::odoMotorL);
  Logger::setFieldValue(odoRM, Logger::odoMotorR);

  m_derivL.update(m_filterPreL.computeOutput(float(odoLM) / UNITS_ODOS_MOTOR, time_micros), time_micros);
  m_derivR.update(m_filterPreR.computeOutput(float(odoRM) / UNITS_ODOS_MOTOR, time_micros), time_micros);

  m_currentSpeedMotorL = m_derivL.getDerivate();
  m_currentSpeedMotorR = m_derivR.getDerivate();

  m_currentAccMotorL = m_filterPostL.computeOutput(m_derivL.getDoubleDerivate(), time_micros);
  m_currentAccMotorR = m_filterPostR.computeOutput(m_derivR.getDoubleDerivate(), time_micros);
//#endif
  Logger::setFieldValue(m_currentSpeedMotorL, Logger::currentSpeedMotorL);
  Logger::setFieldValue(m_currentSpeedMotorR, Logger::currentSpeedMotorR);

}

float OdosMoteursTask::getSpeedL() const
{
  return m_currentSpeedMotorL;
}

float OdosMoteursTask::getSpeedR() const
{
  return m_currentSpeedMotorR;
}

float OdosMoteursTask::getAccelL() const
{
  return m_currentAccMotorL;
}

float OdosMoteursTask::getAccelR() const
{
  return m_currentAccMotorR;
}
