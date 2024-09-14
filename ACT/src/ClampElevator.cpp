
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
long ClampElevatorROS::m_callback_time = 0;
int ClampElevatorROS::m_callback_value = 0;

ClampElevatorROS::ClampElevatorROS(ClampElevator* p_clampelevator, ros::NodeHandle* p_nh) :
    m_sub("/act/order/elevator", subCallback),
    m_pub("/act/callback/elevator", &m_msg){

    m_p_clampelevator = p_clampelevator;
    m_p_nh = p_nh;

    m_callback_value = -1;
}

void ClampElevatorROS::subCallback(const std_msgs::Int16& stateVal){
    m_p_nh->loginfo("[CLAMPELEVATOR] Order");
    if (stateVal.data == 1) m_p_clampelevator->setState(ClampElevatorState::UP);
    else               m_p_clampelevator->setState(ClampElevatorState::DOWN);
    m_callback_value = stateVal.data;
    m_callback_time = millis();
}

void ClampElevatorROS::setup(){
    m_p_clampelevator->setup();
    m_p_nh->subscribe(m_sub);

    m_p_nh->advertise(m_pub);
    
    m_p_nh->loginfo("[CLAMPELEVATOR] Setup");
}

void ClampElevatorROS::loop(){
    m_p_clampelevator->loop();

    if(m_callback_value != -1 && millis() - m_callback_time > CALLBACK_INTERVAL){
        m_msg.data = m_callback_value;
        m_pub.publish(&m_msg);
        m_callback_value = -1;
    }
}

