#include "pumps.h"
#include <Arduino.h>

void Pump::setup() {
    this->switch_pump(0);
}

void Pump::loop() {

}

Pump::Pump(int pin_command_arg) {
    pinMode(pin_command_arg, OUTPUT);
    this->pin_command = pin_command_arg;
}

void Pump::switch_pump(int future_state) {
    this->state_of_pump = future_state;
    digitalWrite(this->pin_command, future_state);
}