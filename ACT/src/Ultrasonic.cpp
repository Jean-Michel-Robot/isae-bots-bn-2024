
#include "Ultrasonic.hpp"
#include "a_define.hpp"
#include <Arduino.h>

Ultrasonic::Ultrasonic(int trig_pin,int echo_pin){
    m_trig_pin = trig_pin;
    m_echo_pin = echo_pin;
    m_state = UltrasonicState::IDLE;
}

float Ultrasonic::getLastMeasure(){
    return m_last_measure;
}

void Ultrasonic::setup(){
    pinMode(m_trig_pin, OUTPUT);
    pinMode(m_echo_pin, INPUT);
}

void Ultrasonic::loop(){

    if(m_state == UltrasonicState::IDLE){
        if(millis() - m_start_measure_timer_ms > ULTRASONIC_MEASURE_INTERVAL){
            digitalWrite(m_trig_pin, HIGH);
            m_timer_us = micros();
            m_state = UltrasonicState::SENDING_SIG;
        }
    }

    else if(m_state == UltrasonicState::SENDING_SIG){
        if(micros() - m_timer_us > ULTRASONIC_TRIG_INTERVAL){
            digitalWrite(m_trig_pin, HIGH);
            m_timer_us = micros();
            m_state = UltrasonicState::WAITING_SIG;
        }
    }

    else if(m_state == UltrasonicState::WAITING_SIG){
        if(digitalRead(m_echo_pin) == HIGH){
            m_last_measure = (micros() - m_timer_us) / ULTRASONIC_TIME_TO_DIST;
            m_state = UltrasonicState::IDLE;
        }
    }
}

