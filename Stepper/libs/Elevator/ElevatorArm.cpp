/*
  Allows for the creation of elevator arm objects, which are part of the elevator
*/

#include "ElevatorArm.h"

// constructor of ElevatorArm class
ElevatorArm::ElevatorArm(String id, int pumpPin,
                         int switchPin, float switchTau, float switchThreshold)
    : m_switchBuoy(switchPin, switchTau, switchThreshold)
{
    this->m_id = id;
    this->m_pumpPin = pumpPin;
}

// reads the filtered bumper state in order to tell wether or not a buoy is present
bool ElevatorArm::isSwitchBuoyPressed()
{
    return m_switchBuoy.isSwitchPressed();
}

// switch the pump on or off : true -> on, false -> off
void ElevatorArm::switchPumpOnOrOff(bool state)
{
    digitalWrite(m_pumpPin, state);
}

void ElevatorArm::setup()
{
    pinMode(m_pumpPin, OUTPUT);
    switchPumpOnOrOff(LOW);

    m_switchBuoy.setup();
}

// updates the switch state
void ElevatorArm::loop()
{
    m_switchBuoy.loop();
}