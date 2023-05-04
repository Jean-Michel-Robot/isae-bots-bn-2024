
#include "Cherry.hpp"


Cherry::Cherry() :
    m_servo(){

        m_positions[0] = CHERRY_UP_POS;
        m_positions[1] = CHERRY_DOWN_POS;
        
}

void Cherry::setState(CherryState state){
    m_state = state;
    m_servo.write(m_positions[(int)state]);
}

void Cherry::setup(){

    m_state = CherryState::DOWN;
    this->setState(m_state);

    m_servo.attach(CHERRY_PIN);   

}

void Cherry::loop(){
    ;
}

Cherry* CherryROS::m_p_cherry = NULL;
ros::NodeHandle* CherryROS::m_p_nh = NULL;

CherryROS::CherryROS(Cherry* p_cherry, ros::NodeHandle* p_nh) :
    m_sub("/strat/cherries", subCallback){

    m_p_cherry = p_cherry;
    m_p_nh = p_nh;

}

void CherryROS::subCallback(const std_msgs::Int16& stateVal){
    if (stateVal.data == 0) m_p_cherry->setState(CherryState::UP);
    else               m_p_cherry->setState(CherryState::DOWN);
}

void CherryROS::setup(){
    m_p_cherry->setup();
    m_p_nh->subscribe(m_sub);
}

void CherryROS::loop(){
    m_p_cherry->loop();
}

