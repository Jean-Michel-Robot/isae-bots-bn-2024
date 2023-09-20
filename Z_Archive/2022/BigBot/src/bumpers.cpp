#include "bumpers.h"
#include <Arduino.h>

void Bumper::setup() {
    
}

void Bumper::update_state() {
    this->state = digitalRead(this->pin_read);
}

Bumper::Bumper(int pin_read_arg) {
    pinMode(pin_read_arg, INPUT);
    this->pin_read = pin_read_arg;
}