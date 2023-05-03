
#include "Doors.hpp"


Doors::Doors() :
    m_left_servo(),
    m_right_servo(){

        m_left_servo.attach(DOORS_LEFT_PIN);   
        m_right_servo.attach(DOORS_RIGHT_PIN);

        m_left_positions[0] = DOORS_LEFT_CLOSED_POS;
        m_left_positions[1] = DOORS_LEFT_OPEN_POS;
        
        m_right_positions[0] = DOORS_RIGHT_CLOSED_POS;
        m_right_positions[1] = DOORS_RIGHT_OPEN_POS;

        m_state = DoorsState::OPEN;
        this->setState(m_state);
}

void Doors::setState(DoorsState state){
    m_state = state;
    m_left_servo.write(m_left_positions[state]);
    m_right_servo.write(m_right_positions[state]);
}

DoorsROS::DoorsROS(Doors* p_doors, ros::NodeHandle* p_nh) :
    m_sub("/strat/doors", subCallback){

    m_p_doors = p_doors;
    m_p_nh = p_nh;

}

void DoorsROS::subCallback(const std_msgs::Int16& stateVal){
    if (stateVal.data == 0) m_p_doors->setState(DoorsState::CLOSED);
    else               m_p_doors->setState(DoorsState::OPEN);
}

