
#include "Ultrasonic.hpp"
#include "a_define.hpp"
#include <Arduino.h>

Ultrasonic::Ultrasonic(int trig_pin,int echo_pin){
    m_trig_pin = trig_pin;
    m_echo_pin = echo_pin;
    m_state = UltrasonicState::IDLE;

    m_timer_us = micros();
    m_last_measure_timer_ms = millis();

}

float Ultrasonic::getLastMeasure(){
    return m_last_measure;
}

void Ultrasonic::setup(){
    pinMode(m_trig_pin, OUTPUT);
    pinMode(m_echo_pin, INPUT);
    digitalWrite(m_trig_pin, LOW);
}

void Ultrasonic::loop(){

    if(m_state == UltrasonicState::IDLE){
        if(millis() - m_last_measure_timer_ms > ULTRASONIC_MEASURE_INTERVAL){
            digitalWrite(m_trig_pin, LOW);
            delayMicroseconds(2);   //short delays (fine)
            digitalWrite(m_trig_pin, HIGH);
            delayMicroseconds(10);
            digitalWrite(m_trig_pin, LOW);
            m_state = UltrasonicState::SENDING_SIG;
        }
    }

    else if(m_state == UltrasonicState::SENDING_SIG){
        if(digitalRead(m_echo_pin) == HIGH){
            m_timer_us = micros();
            m_state = UltrasonicState::WAITING_SIG;
        }
        else if(micros() - m_timer_us > 20000){
            m_state = UltrasonicState::IDLE;
            m_last_measure = 401;
        }
    }

    else if(m_state == UltrasonicState::WAITING_SIG && micros()){
        if(digitalRead(m_echo_pin) == LOW){
            m_last_measure = (float)(micros() - m_timer_us) * ULTRASONIC_SOUND_SPEED / 2;
            m_state = UltrasonicState::IDLE;
            m_last_measure_timer_ms = millis();
        }
        else if(micros() - m_timer_us > 20000){
            m_state = UltrasonicState::IDLE;
            m_last_measure = 401;
        }
    }
}

UltrasonicROS::UltrasonicROS(Ultrasonic* p_left_ultrasonic, Ultrasonic* p_right_ultrasonic, ros::NodeHandle* p_nh) :
    m_pub("ultrasonicDistances",&m_distance_msg){
    
    m_p_left_ultrasonic = p_left_ultrasonic;
    m_p_right_ultrasonic = p_right_ultrasonic;

    m_p_nh = p_nh;

    m_timer_pub = millis();
}

void UltrasonicROS::setup(){  

    m_p_left_ultrasonic->setup();
    m_p_right_ultrasonic->setup();

    m_p_nh->advertise(m_pub);

    m_p_nh->loginfo("[ULTRASONIC] Setup");
}

void UltrasonicROS::loop(){

    m_p_left_ultrasonic->loop();
    m_p_right_ultrasonic->loop();

    if(millis() - m_timer_pub > ULTRASONIC_MEASURE_INTERVAL){
        m_distance_msg.x = (float)m_p_left_ultrasonic->getLastMeasure();
        m_distance_msg.y = (float)m_p_right_ultrasonic->getLastMeasure();
        m_pub.publish(&m_distance_msg);
        m_timer_pub = millis();
    }
}


