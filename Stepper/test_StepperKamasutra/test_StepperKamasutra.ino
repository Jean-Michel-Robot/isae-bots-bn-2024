#include "StepperKamasutra.h"

#define STEP_PIN 2
#define DIR_PIN 3
#define MAX_SPEED 800
#define ACCELERATION 400

String id = "STEPPER";
int nbPos = 3;
int positions[] = {1000, 1400, 3000};

StepperKamasutra stepper = StepperKamasutra(id, STEP_PIN, DIR_PIN, MAX_SPEED, ACCELERATION, nbPos, positions);

void setup() {
	Serial.begin(115200);
	stepper.setup();
}

void loop() {
	stepper.loop();
}