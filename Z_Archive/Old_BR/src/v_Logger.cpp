#include "v_Logger.h"
#include "z_Setup_Loop.h"
#include "a_define.h"

float Logger::m_arrayOfValues[Logger::NbOfFields];

void Logger::setFieldValue(float value, Logger::LogField fieldName)
{
  if (fieldName < NbOfFields)
  {
    m_arrayOfValues[fieldName] = value;
  }
}

float* Logger::getArrayOfValues()
{
  setFieldValue(micros()*1e-6,currentTime);
  return m_arrayOfValues;
}
