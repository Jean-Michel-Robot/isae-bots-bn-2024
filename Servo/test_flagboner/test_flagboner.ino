#include "flagboner.h"

#define LOWER_SWITCH_PIN 2
#define HIGHER_SWITCH_PIN 3
#define SWITCH_TAU 0.05	// Usefull ?
#define SWITCH_THRESHOLD 0.75	// Usefull ?

#define SERVO_PIN 3

#define SERVO_HIGH_POS 50
#define SERVO_LOW_POS 0

String servoId = "SAVD";	// ??
int servoNbPos = 2;
int servoPositions[] = {SERVO_LOW_POS, SERVO_HIGH_POS}; // What value 
		// should be used ??

Flagboner flagboner = Flagboner(SWITCH_PIN, SWITCH_TAU, SWITCH_THRESHOLD,
    servoId, SERVO_PIN, servoNbPos, servoPositions); 


/*
	ROS-related things
*/


void rise_flag() {
	digitalWrite(SERVO_PIN, HIGH); // switch on the servo     
    flagboner.setPosition(1);
    digitalWrite(SERVO_PIN, LOW);
}

void lower_flag() {
	digitalWrite(SERVO_PIN, HIGH); // switch on the servo     
    flagboner.setPosition(0);
    digitalWrite(SERVO_PIN, LOW);
}

void setup() {
    Serial.begin(9600);
    flagboner.setup();
    Serial.println("Setup flagboner");
    delay(500);
}

void loop() {

    if ( true ) {
        rise_flag();        
    }

    delay(2000);

    if ( true ) {
    	lower_flag();
    }

    delay(2000);

    if ( true || digitalRead(LOWER_SWITCH_PIN) == 1 
    		|| digitalRead(HIGHER_SWITCH_PIN) == 1 ) {	// Stop moving
    	digitalWrite(SERVO_PIN, LOW);
    }

    delay(2000);

    Serial.print((digitalRead(SWITCH_PIN)));
}
