#include "ROS.h"

#include "setup_loop.h"


ROS* ros_instance = NULL;
// LED* led_instance = NULL;

unsigned long begin = 0.0;

void setup() {

    pinMode(LED_BUILTIN, OUTPUT);

    ros_instance = new ROS();

    // led_instance = new LED();

    begin = millis();
}


bool led_on = false;

void loop() {


    ros_instance->loop();


    led_on = !led_on;
    if (led_on) {
        // digitalWrite(LED_BUILTIN, HIGH);
        // led_instance->color(255, 255, 255);
        }
    else {
        // digitalWrite(LED_BUILTIN, LOW);
        // led_instance->color(0, 0, 0);
        }

    while (millis() - begin < 100) {}
    begin = millis();

}
