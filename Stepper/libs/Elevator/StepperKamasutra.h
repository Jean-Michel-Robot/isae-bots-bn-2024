/*
  Allows for the creation of stepper objects with a given list of accessible positions, hence the joke about Kamasutra
*/

#ifndef STEPPERKAMASUTRA_H
#define STEPPERKAMASUTRA_H

#include <Arduino.h>
#include <AccelStepper.h>

class StepperKamasutra : private AccelStepper {
  private:
    String id; // id of the motor (not very useful in most cases)
    int stepPin; // step pin of the stepper
    int dirPin; // dir pin of the stepper
    int maxSpeed; // maximum speed of the stepper; in steps/s
    int acceleration; // acceleration/deceleration rate of the stepper, in steps/(s*s)

    int nbPos; // numbers of allowed positions
    int *positions; // list of allowed positions in degres

    void setTargetPositionSteps(int steps); // sets the target position (in steps) of the stepper

  public:
    StepperKamasutra(String id, int stepPin, int dirPin, int maxSpeed, int acceleration, int nbPos, int positions[]); // constructor of StepperKamasutra class

    ~StepperKamasutra(); //destructor of StepperKamasutra class

    void setTargetPosition(int positionId); // sets the target position of the stepper among the listed positions

    void setup(); // initializes the stepper

    void loop(); // updates the stepper behaviour, to be called as frequently as possible
};

#endif
