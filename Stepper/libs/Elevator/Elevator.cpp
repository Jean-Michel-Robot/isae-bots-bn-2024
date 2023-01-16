/*
  Allows for the creation of elevator objects, that is 4 elevator arms and a stepper
*/

#include "Elevator.h"

// constructor of Elevator class
Elevator::Elevator(String id, float switchTau, float switchThreshold,
                   String armIds[NB_ARMS], int pumpPins[NB_ARMS], int switchPins[NB_ARMS],
                   String stepperId, int stepPin, int dirPin, int maxSpeed, int acceleration,
                   int nbPos, int *positions)
    : m_stepper(stepperId, stepPin, dirPin, maxSpeed, acceleration,
                nbPos, positions)
{
    this->m_id = id;

    for (int i = 0; i < NB_ARMS; i++)
    {
        m_arms[i] = new ElevatorArm(armIds[i], pumpPins[i], switchPins[i],
                                switchTau, switchThreshold);
    }
}

// returns the state of the filtered switch of the given arm
bool Elevator::isSwitchBuoyPressed(int armId)
{
    return m_arms[armId]->isSwitchBuoyPressed();
}

// switch the pump of the given arm
void Elevator::switchPump(int armId, bool state)
{
    m_arms[armId]->switchPumpOnOrOff(state);
}

// sets the position of the elevator
void Elevator::setPosition(PositionId positionId)
{
    m_stepper.setTargetPosition(positionId);
}

// initializes the Elevator and all of its components
void Elevator::setup()
{
    for (int i = 0; i < NB_ARMS; i++)
    {
        m_arms[i]->setup();
    }

    m_stepper.setup();
}

void Elevator::loop()
{
    for (int i = 0; i < NB_ARMS; i++)
    {
        m_arms[i]->loop();
    }

    m_stepper.loop();
}