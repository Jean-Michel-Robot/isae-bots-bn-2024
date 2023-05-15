
#include "a_define.hpp"
#include "Deguisement.hpp"

ros::NodeHandle* DeguisementROS::m_p_nh = NULL;

DeguisementROS::DeguisementROS(ros::NodeHandle* p_nh) :
    m_sub("/strat/deguisement", DeguisementROS::subCallback){
        
        m_p_nh = p_nh;
}

void DeguisementROS::setup(){

    m_p_nh->subscribe(m_sub);
    pinMode(DEGUISEMENT_NEOPX_PIN, OUTPUT);
}

void DeguisementROS::loop(){
    ;
}

void DeguisementROS::subCallback(const std_msgs::Int16& stateVal){
    m_p_nh->loginfo("[DEGUISEMENT] Order");
    if(stateVal.data == 0)  digitalWrite(DEGUISEMENT_NEOPX_NUMBER, LOW);
    else                    digitalWrite(DEGUISEMENT_NEOPX_NUMBER, HIGH);
}