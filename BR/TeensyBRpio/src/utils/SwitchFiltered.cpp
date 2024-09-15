/*
  Gère les switchs utilisés pendant les recalages
*/

#include "utils/SwitchFiltered.hpp"

#include "defines.hpp"
#include "utils/FilterLowPass.h"

// TODO provide alternative version for simulation?
// What does this file do?
#ifdef ARDUINO
#include <Arduino.h>
#endif

SwitchFiltered::SwitchFiltered(int pinSwitch) : SWITCH_PIN(pinSwitch)
{
#ifdef ARDUINO
  pinMode(SWITCH_PIN, INPUT_PULLUP);
#endif
}

bool SwitchFiltered::isSwitchPressed() const
{
  return m_filterSwitch.getOutput() > SEUIL_SWITCH;
}

void SwitchFiltered::loop()
{
#ifdef ARDUINO
  if (digitalRead(SWITCH_PIN) == LOW) // si l'etat est bas (relaché)
    m_filterSwitch.computeOutput(0.0, micros());
  else
    m_filterSwitch.computeOutput(1.0, micros());
#endif
}
