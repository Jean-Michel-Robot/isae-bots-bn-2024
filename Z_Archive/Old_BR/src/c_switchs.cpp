/*
   gère les switchs qui valident les recalages

*/
#include "a_define.h"
#include "c_switchs.h"
#include "src/FilterLowPass/FilterLowPass.h"

SwitchFiltered::SwitchFiltered(int pinSwitch):SWITCH_PIN(pinSwitch)
{
    pinMode(SWITCH_PIN, INPUT_PULLUP);
}

bool SwitchFiltered::isSwitchPressed()const
{
    return m_filterSwitch.getOutput() > SEUIL_SWITCH;
}

void SwitchFiltered::loop()
{
    if (digitalRead(SWITCH_PIN) == LOW) // si l'etat est bas (relaché)
      m_filterSwitch.computeOutput(0.0, micros());
    else
      m_filterSwitch.computeOutput(1.0, micros());
}

SwitchesTask::SwitchesTask(TaskType type):Task(type)
{
    m_switches[INDEX_ARD] = new SwitchFiltered(PIN_RECAL_ARD);
    m_switches[INDEX_ARG] = new SwitchFiltered(PIN_RECAL_ARG);
    m_switches[INDEX_AVD] = new SwitchFiltered(PIN_RECAL_AVD);
    m_switches[INDEX_AVG] = new SwitchFiltered(PIN_RECAL_AVG);
}

void SwitchesTask::_loop()
{
  for(SwitchFiltered* sw : m_switches)
  {
      sw->loop();
  }  
}

bool SwitchesTask::isSwitchPressed(int index) const
{
    return m_switches[index]->isSwitchPressed();
}
bool SwitchesTask::isSwitchPressed(bool avant, bool gauche) const
{
  if (avant)
  {
    if (gauche)
    {
      return isSwitchPressed(INDEX_AVG);
    }
    else
      return isSwitchPressed(INDEX_AVD);
  }
  else
  {
    if (gauche)
    {
      return isSwitchPressed(INDEX_ARG);
    }
    else
      return isSwitchPressed(INDEX_ARD);
  }
}
