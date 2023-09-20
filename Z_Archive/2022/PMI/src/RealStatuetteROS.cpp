/**
 * @file RealStatuetteROS.cpp
 * @author Supearo Robotics Club
 * @brief Classs for grabbing and dropping the real statuette
 * @date 2022-04-02
 */

#include "RealStatuetteROS.h"

RealStatuette* RealStatuetteROS::m_p_realStatuette = NULL;
bool RealStatuetteROS::m_isDropping = false;
bool RealStatuetteROS::m_isGrabbing = false;
unsigned long RealStatuetteROS::m_nextTimer = 0;


RealStatuette::RealStatuette(const int leftPin, int leftOpenedPosition, int leftClosedPosition,
                             const int rightPin, int rightOpenedPosition, int rightClosedPosition){
    m_leftServo.attach(leftPin);
    m_rightServo.attach(rightPin);

    m_leftClosedPosition = leftClosedPosition;
    m_leftOpenedPosition = leftOpenedPosition;
    m_rightClosedPosition = rightClosedPosition;
    m_rightOpenedPosition = rightOpenedPosition;
}

void RealStatuette::setup(){
    this->changeState(m_state);
}

void RealStatuette::loop(){
    ;
}

void RealStatuette::changeState(RealStatuetteState state){
    m_state = state;
    if(state == REAL_STATUETTE_CLOSED){
        m_leftServo.write(m_leftClosedPosition);
        m_rightServo.write(m_rightClosedPosition);
    }
    else if (state == REAL_STATUETTE_OPENED){
        m_leftServo.write(m_leftOpenedPosition);
        m_rightServo.write(m_rightOpenedPosition);
    }
}

RealStatuetteState RealStatuette::getState(){
    return m_state;
}

RealStatuetteROS::RealStatuetteROS(ros::NodeHandle* p_nh, RealStatuette* p_realStatuette) :
                         m_subDropRealStatuette("drop_statue_request", dropRealStatuetteCb),
                         m_pubDropRealStatuette("drop_statue_feedback", &m_dropRealStatuetteMsg),
                         m_subGrabRealStatuette("grab_statue_request",grabRealStatuetteCb),
                         m_pubGrabRealStatuette("grab_statue_feedback",&m_grabRealStatuetteMsg){
    
    m_p_nh=p_nh;
    m_p_realStatuette = p_realStatuette;

}

void RealStatuetteROS::setup(){

    m_p_realStatuette->setup();
    m_p_nh->subscribe(m_subDropRealStatuette);
    m_p_nh->advertise(m_pubDropRealStatuette);
    m_p_nh->subscribe(m_subGrabRealStatuette);
    m_p_nh->advertise(m_pubGrabRealStatuette);

}

void RealStatuetteROS::loop(){

    if(m_isDropping && millis() > m_nextTimer){
        m_dropRealStatuetteMsg.data = 1;
        m_pubDropRealStatuette.publish(&m_dropRealStatuetteMsg);
        m_isDropping = false;
    }

    else if(m_isGrabbing && millis() > m_nextTimer){
        m_grabRealStatuetteMsg.data = 1;
        m_pubGrabRealStatuette.publish(&m_grabRealStatuetteMsg);
        m_isGrabbing = false;
    }
}

void RealStatuetteROS::dropRealStatuetteCb(const std_msgs::Int16 &rosLaunch){
    m_p_realStatuette->changeState(REAL_STATUETTE_OPENED);
    m_isDropping = true;
    m_nextTimer = millis() + REAL_STAT_OPENED_TIMER_MS;
}

void RealStatuetteROS::grabRealStatuetteCb(const std_msgs::Int16 &rosLaunch){
    m_p_realStatuette->changeState(REAL_STATUETTE_CLOSED);
    m_isGrabbing = true;
    m_nextTimer = millis() + REAL_STAT_CLOSED_TIMER_MS;
}