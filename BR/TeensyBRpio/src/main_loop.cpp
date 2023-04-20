
#include "main_loop.hpp"

#include <Arduino.h>

#include <motors.hpp>


#include "ROS.hpp"
#include "LED.hpp"
#include "OdosPosition.hpp"
#include "LinearTrajectory.hpp"
#include <RotationTrajectory.hpp>
#include "Asserv.hpp"
#include "BrSM.hpp"


ROS* p_ros = NULL;
OdosPosition* p_odos = NULL;
BlinkLED* p_blink = NULL;
// LED* led_instance = NULL;

LinearTrajectory* p_linearTrajectory = NULL;
RotationTrajectory* p_rotationTrajectory = NULL;

Asserv* p_asserv = NULL;
BrSM* p_sm = NULL;

void setup() {

    Serial.begin(9600);

    delay(100);

    Serial.println("Setup");

    pinMode(LED_BUILTIN, OUTPUT);

    motors_init();

    // Instanciation des classes en dynamique
    p_ros = new ROS();
    p_odos = new OdosPosition();
    p_blink = new BlinkLED();

    // led_instance = new LED();

    p_linearTrajectory = new LinearTrajectory(DEFAULT_LINEAR_GOAL_SPEED, DEFAULT_LINEAR_ACCEL_PARAM);
    p_rotationTrajectory = new RotationTrajectory(DEFAULT_ROTATION_GOAL_SPEED, DEFAULT_ROTATION_ACCEL_PARAM);

    // p_linearTrajectory->setRobotPos(100, 100, 0);
    // p_linearTrajectory->setDest(0.0, 0.0);
    // p_linearTrajectory->beginTrajectory( micros() );

    p_asserv = new Asserv(1.0, 1.0, 1.0);

    p_sm = new BrSM();

    Serial.println("Entering loop");
}

bool led_on = false;
unsigned long timer_old = 0;
// Un tour -> 8192 ticks à vue d'oeil
// int32_t odoL;
// int32_t odoR;

uint32_t loop_timer = 0.0;

float current_speed;

void loop() {

    uint32_t t = micros();

    p_ros->loop();

    p_odos->loop();
    p_blink->loop();


    p_sm->update(t);


    // Periodic display for test
    if (millis() - loop_timer > 100) {
        Serial.println(current_speed);
        loop_timer = millis();
    }


    // Commands for debugging
    if (Serial.available()) {
        Serial.println("good");
        char c = Serial.read();

        // Run calibration sequence
        if (c == 't') {
            Serial.println("Bien joue");
        }
    }
}
