/**
 * @file FlipperROS.h
 * @brief Flipper class + ROS communication
 */

#include "FlipperROS.h"

Flipper* FlipperROS::m_p_flipper = NULL;
bool FlipperROS::m_isFlipping = false;
unsigned long FlipperROS::m_timeout = 0;
LeftRight FlipperROS::m_currentSide = LEFT;


FlipperROS::FlipperROS(Flipper* p_flipper, ros::NodeHandle* p_nh) :                                         
                       m_pubFlipperStatus("arm_feedback", &m_flipperStatusMsg),
                       m_subFlipperCall("arm_request", launchFlippingCb),
                       m_subFlipperUpdateAngles("arm_update",updateAnglesCb){
    m_p_nh = p_nh;
    m_p_flipper = p_flipper;
}

void FlipperROS::setup(){
  
  m_p_flipper->setup();

  m_p_nh->subscribe(m_subFlipperCall);
  m_p_nh->advertise(m_pubFlipperStatus);
  m_p_nh->subscribe(m_subFlipperUpdateAngles);
  m_isFlipping = 0;
  m_currentSide = LEFT;
}

void FlipperROS::loop(){

    m_p_flipper->loop();

    if(m_isFlipping && millis() > m_timeout){
        m_p_flipper->setPosition(m_currentSide, FLIPPER_RETRACTED);
        m_isFlipping = 0;    
        m_flipperStatusMsg.data = 0;
        m_pubFlipperStatus.publish(&m_flipperStatusMsg);
    }
}

void FlipperROS::launchFlippingCb(const std_msgs::Int16 &rosLaunchMsg){
    if(!m_isFlipping){
        if(rosLaunchMsg.data == 1)  m_currentSide = LEFT;
        else                        m_currentSide = RIGHT;
        m_p_flipper->setPosition(m_currentSide, FLIPPER_DEPLOYED);
        m_isFlipping = 1;
        m_timeout = millis() + FLIPPER_TIMEOUT;
    }
}

void FlipperROS::updateAnglesCb(const std_msgs::Int16MultiArray &arrayMsg){
  m_p_flipper->m_leftServo.updatePositionAngle(FLIPPER_RETRACTED, arrayMsg.data[0]);
  m_p_flipper->m_leftServo.updatePositionAngle(FLIPPER_DEPLOYED, arrayMsg.data[1]);
  m_p_flipper->m_RightServo.updatePositionAngle(FLIPPER_RETRACTED, arrayMsg.data[2]);
  m_p_flipper->m_RightServo.updatePositionAngle(FLIPPER_DEPLOYED, arrayMsg.data[3]);
}
