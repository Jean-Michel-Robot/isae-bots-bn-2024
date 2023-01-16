#include <AccelStepper.h>

#define MOTOR_INTERFACE_TYPE 1 //arduino connected to a stepper driver with 2 driver pins

#define DIR_PIN_F 2
#define STEP_PIN_F 3

#define DIR_PIN_B 4
#define STEP_PIN_B 5

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

enum Id {
  FRONT = 0,
  BACK = 1
};

class Stepper : public AccelStepper {
  private:
    Id id;

  public:
    Stepper(Id id, int dirPin, int stepPin) : AccelStepper(MOTOR_INTERFACE_TYPE, stepPin, dirPin) {
      this->id = id;
    }

    //returns the id of the stepper
    Id getId() {
      return this->id;
    }

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
          return this->currentPosition(); //if a wrong position has been passed as arguments, returns the current position
      }
    }

    void setTargetPos(Position pos) {
     this->moveTo(posToSteps(pos)); //sets the target position in steps
    }
    
    void setup() {
      this->setMaxSpeed(MAX_SPEED); //sets maximum speed of the steper, in steps/s
      this->setAcceleration(ACCELERATION); //sets acceleration/deceleration rate, in steps/s/s
    }
    
    void loop() {
      this->run(); //one step if needed
    }
};

Stepper steppers[2] = {Stepper(FRONT, DIR_PIN_F, STEP_PIN_F), Stepper(BACK, DIR_PIN_B, STEP_PIN_B)};

//sets up all the steppers
void setupSteppers() {
  steppers[0].setup();
  steppers[1].setup();
}

//loops all the steppers
void loopSteppers() {
  steppers[0].loop();
  steppers[1].loop();
}
