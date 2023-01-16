/*
  Allows for the creation of stepper objects with a given list of accessible positions, hence the joke about Kamasutra
*/

#include "StepperKamasutra.h"
#include <assert.h>

#define MOTOR_INTERFACE_TYPE 1 //arduino connected to a stepper driver with 2 driver pins

// constructor of StepperKamasutra class
StepperKamasutra::StepperKamasutra(String id, int stepPin, int dirPin, int maxSpeed, int acceleration, int nbPos, int *positions)
        : AccelStepper(MOTOR_INTERFACE_TYPE, stepPin, dirPin) {
    this->id = id;
    this->stepPin = stepPin;
    this->dirPin = dirPin;
    this->maxSpeed = maxSpeed;
    this->acceleration = acceleration;

    this->nbPos = nbPos;

    //copy of the positions array so any modification will not affect the object
    this->positions = (int *) malloc(nbPos * sizeof(int));
    assert(positions != NULL);
    memcpy(this->positions, positions, nbPos * sizeof(int));

}

// destructor of StepperKamasutra class
StepperKamasutra::~StepperKamasutra() {
    free(positions); //frees memory allocated for the positions array inside the object
}

// sets the target position (in steps) of the stepper
void StepperKamasutra::setTargetPositionSteps(int steps) {
    this->moveTo(steps);
}

// sets the target position of the stepper among the listed positions
void StepperKamasutra::setTargetPosition(int positionId) {
    if (positionId < nbPos) {
        setTargetPositionSteps(positions[positionId]);
        //Serial.println(positionId);
    }
}

void StepperKamasutra::setup() {
    this->setMaxSpeed(maxSpeed); // sets the maximum speed of the stepper
    this->setAcceleration(acceleration); // sets the acceleration/deceleration rate of the stepper
}

void StepperKamasutra::loop() {
    this->run(); //one step if needed
}
