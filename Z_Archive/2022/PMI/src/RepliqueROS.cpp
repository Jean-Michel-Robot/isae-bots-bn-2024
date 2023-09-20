/**
 * @file RepliqueROS.cpp
 * @author Supearo Robotics Club
 * @brief Classes for delivering the replique
 * @date 2022-03-05
 */

#include "RepliqueROS.h"

bool RepliqueROS::m_isDropping = false;
Replique* RepliqueROS::m_p_replique = NULL;
unsigned long RepliqueROS::m_nextTimer = 0;
int RepliqueROS::m_iterationNumber = 0;

Replique::Replique(const int pusherPin, int pusherPulledPosition, int pusherPushedPosition,
                   const int blockerPin, int blockerUnlockedPosition, int blockerLockedPosition){

    m_blockerServo.attach(blockerPin);
    m_pusherServo.attach(pusherPin);

    m_blockerUnlockedPosition = blockerUnlockedPosition;
    m_blockerLockedPosition = blockerLockedPosition;
    m_pusherPulledPosition = pusherPulledPosition;
    m_pusherPushedPosition = pusherPushedPosition;
}

void Replique::setup(){
    this->changeState(REPLIQUE_BLOCKED, 0);
}

void Replique::loop(){
    ;
}
void Replique::changeState(RepliqueState state){
    this->changeState(state, -1);
}

void Replique::changeState(RepliqueState state, int nbIter){
    m_state = state;
    if(state == REPLIQUE_BLOCKED){
        m_pusherServo.write(m_pusherPulledPosition);
        m_blockerServo.write(m_blockerLockedPosition);
    }
    else if(state == REPLIQUE_UNLOCKED){
        m_blockerServo.write(m_blockerUnlockedPosition);
    }
    else if(state == REPLIQUE_DELIVERING){
        if(nbIter > 0){
            m_pusherServo.write((m_pusherPulledPosition*(REPLIQUE_DELIVER_NUMBER_ITERATIONS - nbIter) + m_pusherPushedPosition*nbIter) / REPLIQUE_DELIVER_NUMBER_ITERATIONS);
        }
    }
    else if(state == REPLIQUE_DELIVERED){
        m_pusherServo.write(m_pusherPushedPosition);
        m_blockerServo.write(m_blockerUnlockedPosition);
    }
}

RepliqueState Replique::getState(){
    return m_state;
}


RepliqueROS::RepliqueROS(ros::NodeHandle* p_nh, Replique* p_replique) :
                         m_subDropReplique("drop_replic_request", dropRepliqueCb),
                         m_pubDropReplique("drop_replic_feedback", &m_dropRepliqueMsg){
    
    m_p_nh=p_nh;
    m_p_replique = p_replique;

}

void RepliqueROS::setup(){

    m_p_replique->setup();
    m_p_nh->subscribe(m_subDropReplique);
    m_p_nh->advertise(m_pubDropReplique);

}

void RepliqueROS::loop(){

    if(m_isDropping && millis() > m_nextTimer){
        
        if(m_p_replique->getState() == REPLIQUE_UNLOCKED){
            m_p_replique->changeState(REPLIQUE_DELIVERING);
            m_nextTimer = millis() + REPLIQUE_UNLOCK_TIMER_MS;
            m_iterationNumber = 1;
        }

        else if(m_p_replique->getState() == REPLIQUE_DELIVERING){
            if(m_iterationNumber < REPLIQUE_DELIVER_NUMBER_ITERATIONS){
                m_p_replique->changeState(REPLIQUE_DELIVERING, m_iterationNumber);
                m_nextTimer = millis() + REPLIQUE_DELIVER_DIVISION_MS;
                m_iterationNumber++;
            }
            else{
                m_p_replique->changeState(REPLIQUE_DELIVERED);
                m_nextTimer = millis() + REPLIQUE_DELIVER_FINAL_TIMER_MS;
            }
        }

        else if(m_p_replique-> getState() == REPLIQUE_DELIVERED){

            m_p_replique->changeState(REPLIQUE_BLOCKED);
            
            m_dropRepliqueMsg.data = 1;
            m_pubDropReplique.publish(&m_dropRepliqueMsg);

            m_isDropping = false;

        }
    }
}

void RepliqueROS::dropRepliqueCb(const std_msgs::Int16 &rosLaunch){
    m_p_replique->changeState(REPLIQUE_UNLOCKED);
    m_isDropping = true;
    m_nextTimer = millis() + REPLIQUE_UNLOCK_TIMER_MS;
}

