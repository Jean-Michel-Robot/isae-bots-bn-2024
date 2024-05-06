
#include "ClampHoriz.hpp"


ClampHoriz::ClampHoriz() :
    m_servo(){

        m_positions[0] = CLAMPHORIZ_CLOSED_POS;
        m_positions[1] = CLAMPHORIZ_OPEN_POS;
        
}

void ClampHoriz::setState(ClampHorizState state){
    m_state = state;
    m_servo.write(m_positions[(int)state]);
}

void ClampHoriz::setup(){

    m_servo.attach(CLAMPHORIZ_PIN);   

    m_state = ClampHorizState::OPEN;
    this->setState(m_state);
}

void ClampHoriz::loop(){
    ;
}

ClampHoriz* ClampHorizROS::m_p_clamphoriz = NULL;
ros::NodeHandle* ClampHorizROS::m_p_nh = NULL;

ClampHorizROS::ClampHorizROS(ClampHoriz* p_clamphoriz, ros::NodeHandle* p_nh) :
    m_sub("/act/order/doors", subCallback){

    m_p_clamphoriz = p_clamphoriz;
    m_p_nh = p_nh;

}

void ClampHorizROS::subCallback(const std_msgs::Int16& stateVal){
    m_p_nh->loginfo("[CLAMPHORIZ] Order");
    if (stateVal.data == 2) m_p_clamphoriz->setState(ClampHorizState::CLOSED);
    else               m_p_clamphoriz->setState(ClampHorizState::OPEN);
}

void ClampHorizROS::setup(){
    m_p_clamphoriz->setup();
    m_p_nh->subscribe(m_sub);

    m_p_nh->loginfo("[CLAMPHORIZ] Setup");
}

void ClampHorizROS::loop(){
    m_p_clamphoriz->loop();
}

