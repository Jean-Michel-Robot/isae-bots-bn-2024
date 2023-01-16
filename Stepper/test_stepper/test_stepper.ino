#include <AccelStepper.h>

#define MOTOR_INTERFACE_TYPE 1 //arduino connected to a stepper driver with 2 driver pins
#define DIR_PIN 2
#define STEP_PIN 3

//New instance of AccelStepper class
AccelStepper stepper = AccelStepper(MOTOR_INTERFACE_TYPE, STEP_PIN, DIR_PIN);

void setup() {
  stepper.setMaxSpeed(800); //max speed in step/s
  stepper.setAcceleration(400);
  stepper.moveTo(5000);

}

void loop() {
  stepper.run(); //runs at set speed

}
