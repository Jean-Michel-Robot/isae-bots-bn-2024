#include "Tentacle.h"

#define SWITCH_PIN 2
#define SWITCH_TAU 0.05
#define SWITCH_THRESHOLD 0.75

#define PUMP_PIN 3
#define PUMP_HALF_PERIOD 1000 // ms

#define SERVO_PIN 4

String id = "TAVD";

String servoId = "SAVD";
int servoNbPos = 3;
int servoPositions[] = {20, 60, 90};

Tentacle tentacle = Tentacle(id, PUMP_PIN, SWITCH_PIN, SWITCH_TAU, SWITCH_THRESHOLD,
    servoId, SERVO_PIN, servoNbPos, servoPositions); 

void setup() {
    Serial.begin(9600);
    tentacle.setup();
    Serial.println("read,filter,threshold,servoPositionId");
    delay(500);

}

unsigned long lastTimeMillis = 0.0;
unsigned long currentTimeMillis = 0.0;
int servoPositionId = 0;

void loop() {
    // switch test
    tentacle.loop();

    // pump test and servo test
    currentTimeMillis = millis();
    if (currentTimeMillis - lastTimeMillis > PUMP_HALF_PERIOD) {
        
        digitalWrite(PUMP_PIN, HIGH - digitalRead(PUMP_PIN)); // switch on and off
        
        tentacle.setPosition(servoPositionId);
        servoPositionId = (servoPositionId + 1) % servoNbPos;

        lastTimeMillis = currentTimeMillis;
    }

    Serial.print((digitalRead(SWITCH_PIN)));
    Serial.print(",");
    Serial.print(tentacle.getSwitchBuoyFilterOutput());
    Serial.print(",");
    Serial.print(tentacle.isSwitchBuoyPressed());
    Serial.print(",");
    Serial.println(servoPositionId);
}
