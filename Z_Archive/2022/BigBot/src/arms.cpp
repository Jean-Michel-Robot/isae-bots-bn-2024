#include "arms.h"
#include <Arduino.h>


void Arm::setup(int arm_angle_up, int arm_angle_down, int arm_angle_down_gallery) {

    this->arm_angle_up = arm_angle_up;
    this->arm_angle_down = arm_angle_down;
    this->arm_angle_down_gallery = arm_angle_down_gallery;

    this->m_p_servo->write(arm_angle_up);

    current_angle = arm_angle_up;

    m_p_bumper->setup();
    m_p_pump->setup();
}


Arm::Arm(int pin, Servo* m_p_servo_arg, Bumper* m_p_bumper_arg, Pump* m_p_pump1_arg) {

    this->m_p_servo = m_p_servo_arg;
    this->m_p_servo->attach(pin);

    this->m_p_bumper = m_p_bumper_arg;
    this->m_p_pump = m_p_pump1_arg;
}

void Arm::move_down_take() {
    this->movedown_take = 1;
    this->triggered_take = 1;
}

void Arm::move_down_rack() {
    this->movedown_rack = 1;
    this->triggered_rack = 1;
}

void Arm::move_down_gallery() {
    this->movedown_gallery = 1;
    this->triggered_gallery = 1;
}

void Arm::move_down_camp() {
    this->movedown_camp = 1;
    this->triggered_camp = 1;
}