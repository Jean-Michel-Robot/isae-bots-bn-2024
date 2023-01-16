#include "ElevatorArm.h"

#define SWITCH_TAU 0.05
#define SWITCH_THRESHOLD 0.5

#define NB_ARMS 2

String armIds[NB_ARMS] = {"R", "L"};
int pumpPins[NB_ARMS] = {2, 4};
int switchPins[NB_ARMS] = {3, 5};

ElevatorArm armR = ElevatorArm(armIds[0], pumpPins[0], switchPins[0], SWITCH_TAU, SWITCH_THRESHOLD);
ElevatorArm armL = ElevatorArm(armIds[1], pumpPins[1], switchPins[1], SWITCH_TAU, SWITCH_THRESHOLD);

void setup()
{

    Serial.begin(9600);
    armR.setup();
    armL.setup();

    Serial.print("armR");
    Serial.print(",");
    Serial.println("armL");
}

void loop()
{
    static unsigned long last = 0;

    armR.loop();
    armL.loop();

    if (millis() - last > 100)
    {
        last = millis();

        armR.switchPumpOnOrOff(armR.isSwitchBuoyPressed());
        armL.switchPumpOnOrOff(armL.isSwitchBuoyPressed());
        
        Serial.print(armR.isSwitchBuoyPressed());
        Serial.print(",");
        Serial.println(armL.isSwitchBuoyPressed());

    }
}