
#include "asserv.hpp"

#include "ROS.hpp"
#include "OdosPosition.hpp"


#include "main_loop.hpp"
#include <cmath>



asservPID::asservPID(float k1, float k2, float k3) {
    m_k1 = k1;
    m_k2 = k2;
    m_k3 = k3;

    m_p_trajectory = NULL;
}

void asservPID::updateError() {
    //CHECK IF REF CHANGE IS OK
    Position2D botPosition = p_odos->getRobotPosition();
    m_errorPos = m_p_trajectory->getPointAtTime(micros()) - p_odos->getRobotPosition();
    m_errorPos.changeReferentiel(botPosition);
}



void asservPID::updateCommand() {

    // m_target = m_p_trajectory->getVelAndTheta(micros());  //TODO

    float vd = m_target[0];
    float omega_d = m_target[1];

    this->updateError();

    /* En utilisant la formule qu'on sait pas d'où elle sort*/
    if(cos(m_errorPos.theta) == 0){
        // Protection div par 0 (ça peut servir)
        m_botSpeed[0] = 0;
        m_botSpeed[1] = 0; 
    }
    else {   
        m_botSpeed[0] = (vd - m_k1*abs(vd)*(m_errorPos.x + m_errorPos.y*tan(m_errorPos.theta)))/cos(m_errorPos.theta);
        m_botSpeed[1] = omega_d - (m_k2*vd*m_errorPos.y  + m_k3*abs(vd)*tan(m_errorPos.theta))*pow(cos(m_errorPos.theta),2);
    }

    m_leftWheelSpeed = m_botSpeed[0] + m_botSpeed[1]*WHEEL_DISTANCE/2; 
    m_rightWheelSpeed = m_botSpeed[0] - m_botSpeed[1]*WHEEL_DISTANCE/2;        
}

void asservPID::loop() {
    this->updateCommand();
}

