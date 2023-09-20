#include "isb.h"
#include <Arduino.h>

// ADVERTISE

std_msgs::Int16 Isb::msg_tirette;
ros::Publisher Isb::m_publisher_tirette("start", &msg_tirette);

void Isb::setup() {
    tirette->setup();
    tirette->update_state();

    this->m_p_ros_pointer_tirette->advertise(m_publisher_tirette);
}

Isb::Isb(ros::NodeHandle* m_p_ros_pointer_tirette_arg, int arg_pin_tirette, Bumper* tirette_arg) {
    
    this->tirette = tirette_arg;
    pinMode(arg_pin_tirette, INPUT);
    this->pin_tirette = arg_pin_tirette;
    this->m_p_ros_pointer_tirette = m_p_ros_pointer_tirette_arg;
}

void Isb::update_state_tirette() {
    this->tirette->update_state();
}