
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
long ClampROS::m_callback_time = 0;
int ClampROS::m_callback_value = 0;

ClampROS::ClampROS(Clamp* p_clamp, ros::NodeHandle* p_nh) :
    m_sub("/act/order/clamp", subCallback),
    m_pub("/act/callback/clamp", &m_msg){

    m_p_clamp = p_clamp;
    m_p_nh = p_nh;
    m_callback_value = -1;

}

void ClampROS::subCallback(const std_msgs::Int16& stateVal){
    m_p_nh->loginfo("[CLAMP] Order");
    if (stateVal.data == 1) m_p_clamp->setState(ClampState::CLOSED);
    else               m_p_clamp->setState(ClampState::OPEN);
    m_callback_value = stateVal.data;
    m_callback_time = millis();
}

void ClampROS::setup(){
    m_p_clamp->setup();
    m_p_nh->subscribe(m_sub);

    m_p_nh->advertise(m_pub);

    m_p_nh->loginfo("[CLAMP] Setup");
}

void ClampROS::loop(){
    m_p_clamp->loop();

    if(m_callback_value != -1 && millis() - m_callback_time > CALLBACK_INTERVAL){
        m_msg.data = m_callback_value;
        m_pub.publish(&m_msg);
        m_callback_value = -1;
    }
}

