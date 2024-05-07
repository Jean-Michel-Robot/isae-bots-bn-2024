
#include "Doors.hpp"


Doors::Doors() :
    m_left_servo(),
    m_right_servo(){

        m_left_positions[0] = DOORS_LEFT_CLOSED_POS;
        m_left_positions[1] = DOORS_LEFT_OPEN_POS;
        m_left_positions[2] = DOORS_LEFT_CLOSED_POS;
        
        m_right_positions[0] = DOORS_RIGHT_CLOSED_POS;
        m_right_positions[1] = DOORS_RIGHT_CLOSED_POS;
        m_right_positions[2] = DOORS_RIGHT_OPEN_POS;
}

void Doors::setState(DoorsState state){
    m_state = state;
    m_left_servo.write(m_left_positions[state]);
    m_right_servo.write(m_right_positions[state]);

}

void Doors::setup(){

    m_left_servo.attach(DOORS_LEFT_PIN);   
    m_right_servo.attach(DOORS_RIGHT_PIN);

    m_state = DoorsState::CLOSED;
    this->setState(m_state);

}

void Doors::loop(){
    ;
}

Doors* DoorsROS::m_p_doors = NULL;
ros::NodeHandle* DoorsROS::m_p_nh = NULL;
long DoorsROS::m_callback_time_left = 0;
int DoorsROS::m_callback_value_left = 0;
long DoorsROS::m_callback_time_right = 0;
int DoorsROS::m_callback_value_right = 0;

DoorsROS::DoorsROS(Doors* p_doors, ros::NodeHandle* p_nh) :
    m_subLeft("/act/order/left_arm", subCallbackLeft),
    m_subRight("/act/order/right_arm", subCallbackRight),
    m_pubLeft("/act/callback/left_arm", &m_msg_left),
    m_pubRight("/act/callback/right_arm", &m_msg_right){

    m_p_doors = p_doors;
    m_p_nh = p_nh;

    m_callback_value_left = -1;
    m_callback_value_right = -1;

}

void DoorsROS::subCallbackLeft(const std_msgs::Int16& stateVal){

    m_p_nh->loginfo("[ARMS] Order Left");

    if (stateVal.data == 0) m_p_doors->setState(DoorsState::CLOSED);
    else if(stateVal.data == 1)  m_p_doors->setState(DoorsState::LEFT_OPEN);
    m_callback_value_left = stateVal.data;
    m_callback_time_left = millis();
}

void DoorsROS::subCallbackRight(const std_msgs::Int16& stateVal){

    m_p_nh->loginfo("[ARMS] Order Right");

    if (stateVal.data == 0) m_p_doors->setState(DoorsState::CLOSED);
    else if(stateVal.data == 1)  m_p_doors->setState(DoorsState::RIGHT_OPEN);
    m_callback_value_right = stateVal.data;
    m_callback_time_right = millis();

}

void DoorsROS::setup(){
    m_p_doors->setup();
    m_p_nh->subscribe(m_subLeft);
    m_p_nh->subscribe(m_subRight);

    m_p_nh->advertise(m_pubLeft);
    m_p_nh->advertise(m_pubRight);

    m_p_nh->loginfo("[ARMS] Setup");
}

void DoorsROS::loop(){
    m_p_doors->loop();

    if(m_callback_value_left != -1 && millis() - m_callback_time_left > CALLBACK_INTERVAL){
        m_msg_left.data = m_callback_value_left;
        m_pubLeft.publish(&m_msg_left);
        m_callback_value_left = -1;
    }

    if(m_callback_value_right != -1 && millis() - m_callback_time_right > CALLBACK_INTERVAL){
        m_msg_right.data = m_callback_value_right;
        m_pubRight.publish(&m_msg_right);
        m_callback_value_right = -1;
    }
}

