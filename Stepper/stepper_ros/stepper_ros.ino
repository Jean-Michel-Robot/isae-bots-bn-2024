#include <AccelStepper.h>

#define MOTOR_INTERFACE_TYPE 1 //arduino connected to a stepper driver with 2 driver pins
#define DIR_PIN 2
#define STEP_PIN 3

#define MAX_SPEED 800 //steps/s
#define ACCELERATION 800 //steps/s/s

#define LOW_STEPS 250 //steps
#define MID_STEPS 500 //steps
#define HIGH_STEPS 1000 //steps

enum Position {
  LOW_POS = 0,
  MID_POS = 1,
  HIGH_POS = 2
};

//New instance of AccelStepper class
AccelStepper stepper = AccelStepper(MOTOR_INTERFACE_TYPE, STEP_PIN, DIR_PIN);

//Converts a position to steps
int posToSteps(Position pos) {
  switch(pos) {
    case(LOW_POS):
      return LOW_STEPS;
      break;
    case(MID_POS):
      return MID_STEPS;
      break;
    case(HIGH_POS):
      return HIGH_STEPS;
      break;
    default:
      return stepper.currentPosition();
  }
}

void setTargetPos(Position pos) {
  stepper.moveTo(posToSteps(pos)); //sets the target position in steps
}

void setupStepper() {
  stepper.setMaxSpeed(MAX_SPEED); //sets maximum speed of the steper, in steps/s
  stepper.setAcceleration(ACCELERATION); //sets acceleration/deceleration rate, in steps/s/s
}

void loopStepper() {
  stepper.run(); //one step if needed
}
