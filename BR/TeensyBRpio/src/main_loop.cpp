
#include "main_loop.hpp"


ROS* ros_instance = NULL;
OdosPosition* p_odos = NULL;
// LED* led_instance = NULL;

unsigned long begin = 0.0;

void setup() {

    pinMode(LED_BUILTIN, OUTPUT);

    motors_init();

    ros_instance = new ROS();

    // led_instance = new LED();

    p_odos = new OdosPosition(ros_instance);

    Serial.begin(9600);


    begin = millis();
}


bool led_on = false;
unsigned long timer_old = 0;
// Un tour -> 8192 ticks à vue d'oeil
// int32_t odoL;
// int32_t odoR;

void loop() {


    ros_instance->loop();

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
