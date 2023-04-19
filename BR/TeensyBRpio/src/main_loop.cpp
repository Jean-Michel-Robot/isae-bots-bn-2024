
#include "main_loop.hpp"

#include <Arduino.h>

#include <motors.hpp>


#include "ROS.hpp"
#include "LED.hpp"
#include "OdosPosition.hpp"
#include "LinearTrajectory.hpp"
#include "Asserv.hpp"
#include "BrSMWrapper.hpp"


ROS* p_ros = NULL;
OdosPosition* p_odos = NULL;
BlinkLED* p_blink = NULL;
// LED* led_instance = NULL;

LinearTrajectory* p_linearTrajectory = NULL;
Asserv* p_asserv = NULL;
BrSMWrapper* p_sm = NULL;

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

    p_linearTrajectory = new LinearTrajectory(0.0, 0.0);

    p_linearTrajectory->setRobotPos(100, 100, 0);
    p_linearTrajectory->setDest(0.0, 0.0);
    p_linearTrajectory->beginTrajectory( micros() );

    p_asserv = new Asserv(1.0, 1.0, 1.0);

    p_sm = new BrSMWrapper();

    Serial.println("Entering loop");
}

bool led_on = false;
unsigned long timer_old = 0;
// Un tour -> 8192 ticks à vue d'oeil
// int32_t odoL;
// int32_t odoR;

uint32_t loop_timer = 0.0;

float current_speed;

Position2D trajPoint = 0;

void loop() {


    p_ros->loop();

    p_odos->loop();
    p_blink->loop();

    // uint32_t t = micros();


    // trajPoint = p_linearTrajectory->getPointAtTime(t);
    // Serial.println(trajPoint.toString());


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
