
#include "asserv.hpp"

#include "ROS.hpp"
#include "OdosPosition.hpp"

#include <LinearTrajectory.hpp>
#include "main_loop.hpp"

#include <cmath>




asservPID::asservPID(float k1, float k2, float k3) {

    m_k1 = k1;
    m_k2 = k2;
    m_k3 = k3;

    setErrorPositionThreshold(OBJECTIVE_THRESHOLD_X, OBJECTIVE_THRESHOLD_Y, OBJECTIVE_THRESHOLD_THETA);

    m_state = ACTIVE;
}

void asservPID::updateError() {

    Position2D currentBotPosition = p_odos->getRobotPosition();
    Position2D errorPosTableFrame = p_linearTrajectory->getTrajectoryPoint() - p_odos->getRobotPosition();
    float angle = currentBotPosition.theta;

    //TOTEST CHECK IF REF CHANGE IS OK (avec alpha et beta aussi)
    m_errorPos.x = cos(angle)*errorPosTableFrame.x + sin(angle)*errorPosTableFrame.y;
    m_errorPos.y = -sin(angle)*errorPosTableFrame.x + cos(angle)*errorPosTableFrame.y;
    m_errorPos.theta = errorPosTableFrame.theta;
}



void asservPID::updateCommand() {

    // update trajectory
    p_linearTrajectory->updateTrajectory( micros() );

    if(m_state == ACTIVE) {
        
        // get trajectory speed (linear and angular)
        m_target[0] = p_linearTrajectory->getTrajectoryLinearSpeed();
        m_target[1] = p_linearTrajectory->getTrajectoryAngularSpeed();

        float vd = m_target[0];
        float omega_d = m_target[1];

        // update error using trajectory
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
}

void asservPID::setErrorPositionThreshold(float x, float y, float theta){
    if (x>0)
        m_errorPosThreshold.x = x;
    
    if (y>0)
        m_errorPosThreshold.y = y;

    if (theta>0)
        m_errorPosThreshold.theta = theta;
}


bool asservPID::isAtObjectivePoint(bool checkAngle){
    
    this->updateError();

    if (abs(m_errorPos.x) > m_errorPosThreshold.x)
        return false;
    else if (abs(m_errorPos.y) > m_errorPosThreshold.y)
        return false;
    else if (checkAngle && (abs(fmod(m_errorPos.theta, TWO_PI)) > m_errorPosThreshold.theta))
        return false;
    else
        return true;
}

void asservPID::loop() {
    // this->updateCommand();

    /*
    Dansl'ordre :

    setRobotPos(x0, y0, theta0);
    setDest( <vars> );

    beginTrajectory(t0);

    updateTrajectory(t);

    getTrajectoryPoint() -> (x, y, theta)
    getTrajectoryLinearSpeed() -> V
    getTrajectoryAngularSpeed() -> omega
    */
}

