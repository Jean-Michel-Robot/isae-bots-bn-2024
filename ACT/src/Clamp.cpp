
#include "Clamp.hpp"


Clamp::Clamp() :
    m_servo(){

        m_positions[0] = CLAMP_CLOSED_POS;
        m_positions[1] = CLAMP_OPEN_POS;
        
}

void Clamp::setState(ClampState state){
    m_state = state;
    m_servo.write(m_positions[(int)state]);
}

void Clamp::setup(){

    m_servo.attach(CLAMP_PIN);   

    m_state = ClampState::OPEN;
    this->setState(m_state);
}

void Clamp::loop(){
    ;
}

Clamp* ClampROS::m_p_clamp = NULL;
ros::NodeHandle* ClampROS::m_p_nh = NULL;

ClampROS::ClampROS(Clamp* p_clamp, ros::NodeHandle* p_nh) :
    m_sub("/strat/clamp", subCallback){

    m_p_clamp = p_clamp;
    m_p_nh = p_nh;

}

void ClampROS::subCallback(const std_msgs::Int16& stateVal){
    m_p_nh->loginfo("[CLAMP] Order");
    if (stateVal.data == 0) m_p_clamp->setState(ClampState::CLOSED);
    else               m_p_clamp->setState(ClampState::OPEN);
}

void ClampROS::setup(){
    m_p_clamp->setup();
    m_p_nh->subscribe(m_sub);

    m_p_nh->loginfo("[CLAMP] Setup");
}

void ClampROS::loop(){
    m_p_clamp->loop();
}

