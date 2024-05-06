
#include "ClampElevator.hpp"


ClampElevator::ClampElevator() :
    m_servo(){

        m_positions[0] = CLAMPELEVATOR_UP_POS;
        m_positions[1] = CLAMPELEVATOR_DOWN_POS;
        
}

void ClampElevator::setState(ClampElevatorState state){
    m_state = state;
    m_servo.write(m_positions[(int)state]);
}

void ClampElevator::setup(){

    m_servo.attach(CLAMPELEVATOR_PIN);   

    m_state = ClampElevatorState::DOWN;
    this->setState(m_state);


}

void ClampElevator::loop(){
    ;
}

ClampElevator* ClampElevatorROS::m_p_clampelevator = NULL;
ros::NodeHandle* ClampElevatorROS::m_p_nh = NULL;

ClampElevatorROS::ClampElevatorROS(ClampElevator* p_clampelevator, ros::NodeHandle* p_nh) :
    m_sub("/act/order/elevator", subCallback){

    m_p_clampelevator = p_clampelevator;
    m_p_nh = p_nh;

}

void ClampElevatorROS::subCallback(const std_msgs::Int16& stateVal){
    m_p_nh->loginfo("[CLAMPELEVATOR] Order");
    if (stateVal.data == 1) m_p_clampelevator->setState(ClampElevatorState::UP);
    else               m_p_clampelevator->setState(ClampElevatorState::DOWN);
}

void ClampElevatorROS::setup(){
    m_p_clampelevator->setup();
    m_p_nh->subscribe(m_sub);

    m_p_nh->loginfo("[CLAMPELEVATOR] Setup");
}

void ClampElevatorROS::loop(){
    m_p_clampelevator->loop();
}

