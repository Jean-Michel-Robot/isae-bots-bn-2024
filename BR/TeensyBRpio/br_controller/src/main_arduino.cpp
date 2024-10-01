#if defined(ARDUINO) && !defined(_SIMULATION)

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include "specializations/manager.hpp"
#include "specializations/ros.hpp"
#include <Led.hpp>

std::optional<BlinkLED> blinkingLed = {};
std::optional<ros_t> rosInstance = {};

void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    blinkingLed.emplace();

    rosInstance.emplace();
    rosInstance->attachManager(std::make_shared<manager_t>());
}

void loop() {
    rosInstance->loop();
    blinkingLed->loop();
}

void log(LogSeverity severity, string_t message) {
    if (rosInstance) {
        rosInstance->sendLog(severity, message);
    }
}

#endif