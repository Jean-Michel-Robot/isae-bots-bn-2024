/*
  Gère les switchs utilisés pendant les recalages
*/

#include <SwitchFiltered.hpp>

#include <defines.hpp>
#include <FilterLowPass.h>

SwitchFiltered::SwitchFiltered(int pinSwitch):SWITCH_PIN(pinSwitch)
{
    pinMode(SWITCH_PIN, INPUT_PULLUP);
}

bool SwitchFiltered::isSwitchPressed() const
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
