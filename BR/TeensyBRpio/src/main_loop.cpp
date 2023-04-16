
#include "main_loop.hpp"

#include <Arduino.h>

#include <motors.hpp>
// #include <QuadDecode.h>


#include "ROS.hpp"
#include "LED.hpp"
#include "OdosPosition.hpp"
#include "LinearTrajectory.hpp"


ROS* p_ros = NULL;
OdosPosition* p_odos = NULL;
BlinkLED* p_blink = NULL;
// LED* led_instance = NULL;

LinearTrajectory* p_linearTrajectory = NULL;

unsigned long begin = 0.0;

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

    p_linearTrajectory = new LinearTrajectory();
    p_linearTrajectory->setDest(0.0, 0.0, 0.0, 0.0);
    p_linearTrajectory->beginTrajectory( micros() );



    begin = millis();

    Serial.println("Entering loop");
}

bool led_on = false;
unsigned long timer_old = 0;
// Un tour -> 8192 ticks à vue d'oeil
// int32_t odoL;
// int32_t odoR;

Position2D trajPoint = 0;

void loop() {


    p_ros->loop();

    p_odos->loop();
    p_blink->loop();

    uint32_t t = micros();

    trajPoint = p_linearTrajectory->getPointAtTime(t);
    Serial.println(trajPoint.toString());

    // led_on = !led_on;
    // if (led_on) {
    //     digitalWrite(LED_BUILTIN, HIGH);
    //     // led_instance->color(255, 255, 255);
    //     }
    // else {
    //     digitalWrite(LED_BUILTIN, LOW);
    //     // led_instance->color(0, 0, 0);
    //     }


    // while (millis() - begin < 100) {}  // période de 10 Hz
    // begin = millis();

}
