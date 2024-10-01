#ifdef ARDUINO

#include "Led.hpp"

#include <Arduino.h>

#define BLINK_INTERVAL 100 // ms
#define LED_PIN 13

BlinkLED::BlinkLED() {
    pinMode(LED_BUILTIN, OUTPUT);
    m_state = 1;
    m_timer = 0.0;
}

void BlinkLED::loop() {
    if (millis() - m_timer > BLINK_INTERVAL) {
        m_state = 1 - m_state;
        digitalWrite(LED_PIN, m_state);
        m_timer = millis();
    }
}

#endif