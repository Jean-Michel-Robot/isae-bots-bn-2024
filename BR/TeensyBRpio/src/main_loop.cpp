
#include "main_loop.hpp"

#include <Arduino.h>

#include <motors.hpp>
// #include <QuadDecode.h>


#include "ROS.hpp"
#include "LED.hpp"
#include "OdosPosition.hpp"


ROS* p_ros = NULL;
OdosPosition* p_odos = NULL;
// LED* led_instance = NULL;

unsigned long begin = 0.0;

void setup() {

    pinMode(LED_BUILTIN, OUTPUT);

    motors_init();

    // Instanciation des classes
    p_ros = new ROS();
    p_odos = new OdosPosition();

    // led_instance = new LED();


    Serial.begin(9600);


    begin = millis();
}


bool led_on = false;
unsigned long timer_old = 0;
// Un tour -> 8192 ticks à vue d'oeil
// int32_t odoL;
// int32_t odoR;

void loop() {


    p_ros->loop();

    p_odos->loop();



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
