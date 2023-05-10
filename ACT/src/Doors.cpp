
#include "Doors.hpp"


Doors::Doors() :
    m_left_servo(),
    m_right_servo(){

        m_left_positions[0] = DOORS_LEFT_CLOSED_POS;
        m_left_positions[1] = DOORS_LEFT_OPEN_POS;
        
        m_right_positions[0] = DOORS_RIGHT_CLOSED_POS;
        m_right_positions[1] = DOORS_RIGHT_OPEN_POS;
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

DoorsROS::DoorsROS(Doors* p_doors, ros::NodeHandle* p_nh) :
    m_sub("/strat/doors", subCallback){

    m_p_doors = p_doors;
    m_p_nh = p_nh;

}

void DoorsROS::subCallback(const std_msgs::Int16& stateVal){

    m_p_nh->loginfo("[DOORS] Order");

    if (stateVal.data == 0) m_p_doors->setState(DoorsState::CLOSED);
    else               m_p_doors->setState(DoorsState::OPEN);
}

void DoorsROS::setup(){
    m_p_doors->setup();
    m_p_nh->subscribe(m_sub);

    m_p_nh->loginfo("[DOORS] Setup");
}

void DoorsROS::loop(){
    m_p_doors->loop();
}

