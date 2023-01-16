/*
  Allows for the creation of elevator arm objects, which are part of the elevator
*/

#ifndef ELEVATORARM_H
#define ELEVATORARM_H

#include <Arduino.h>
#include "StepperKamasutra.h"
#include "SwitchFiltered.h"

class ElevatorArm
{
private:
  String m_id; // id of the arm

  int m_pumpPin;               // pin where the pump is connected
  SwitchFiltered m_switchBuoy; // switch to detect presence of a buoy

public:
  ElevatorArm(String id, int pumpPin,
              int switchPin, float switchTau, float switchThreshold); // constructor of ElevatorArm class

  bool isSwitchBuoyPressed(); // reads the filtered bumper state in order to tell wether or not a buoy is present

  void switchPumpOnOrOff(bool state); // switch the pump on or off : true -> on, false -> off

  void setup();

  void loop(); // updates the switch state
};

#endif