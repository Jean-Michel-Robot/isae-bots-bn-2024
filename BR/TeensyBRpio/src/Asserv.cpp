
#include "Asserv.hpp"

#include "OdosPosition.hpp"
#include <Motors.hpp>

#include "ROS.hpp"
#include "main_loop.hpp"

#include <cmath>




Asserv::Asserv(float k1, float k2, float k3) {
    m_k1 = k1;
    m_k2 = k2;
    m_k3 = k3;

    m_errorPosThreshold.x = DEFAULT_OBJECTIVE_THRESHOLD_X;
    m_errorPosThreshold.y = DEFAULT_OBJECTIVE_THRESHOLD_Y;
    m_errorPosThreshold.theta = DEFAULT_OBJECTIVE_THRESHOLD_THETA;

    m_state = ACTIVE; //TODO idle by default ?
}


void Asserv::setGains(float k1, float k2, float k3) {
    m_k1 = k1;
    m_k2 = k2;
    m_k3 = k3;
}

void Asserv::updateError(Position2D trajectoryPointPos) {

    Position2D currentBotPosition = p_odos->getRobotPosition();
    Position2D errorPosTableFrame = trajectoryPointPos - p_odos->getRobotPosition();
    float angle = currentBotPosition.theta;

    //TOTEST CHECK IF REF CHANGE IS OK (avec alpha et beta aussi)
    m_errorPos.x = cos(angle)*errorPosTableFrame.x + sin(angle)*errorPosTableFrame.y;
    m_errorPos.y = -sin(angle)*errorPosTableFrame.x + cos(angle)*errorPosTableFrame.y;
    m_errorPos.theta = errorPosTableFrame.theta;
}


/*
This function takes an input command and sends a command to the motors
- If the asserv state is ACTIVE the command is transformed using the asserv formula
- If the asserv state is BYPASSED the command is send directly to the motors
  (the asserv can also be bypassed if the bypassAsserv argument is set to true)
- If the asserv state is IDLE the command is not used and a command of zero is sent
*/
void Asserv::updateCommand(float vd, float omega_d, bool bypassAsserv=false) {

    // update trajectory
    // p_linearTrajectory->updateTrajectory( micros() );

    m_state = BYPASSED; //FORTEST


    if(m_state == ACTIVE) {
        
        // get trajectory speed (linear and angular)
        // m_target[0] = p_linearTrajectory->getTrajectoryLinearSpeed();
        // m_target[1] = p_linearTrajectory->getTrajectoryAngularSpeed();

        // float vd = m_target[0];
        // float omega_d = m_target[1];

        // // update error using trajectory
        // this->updateError();

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

    else if (m_state == BYPASSED || bypassAsserv) {
        
        // asserv is bypassed and we directly feed the command
        m_leftWheelSpeed = vd + omega_d*WHEEL_DISTANCE/2;
        m_rightWheelSpeed = vd - omega_d*WHEEL_DISTANCE/2;
    }

    else { // asserv is idle

        //TODO make the motors stay still
        m_leftWheelSpeed = 0.0;
        m_rightWheelSpeed = 0.0;
    }

    // send the commands to the motors
    sendMotorCommand(BR_LEFT, m_leftWheelSpeed);
    sendMotorCommand(BR_RIGHT, m_rightWheelSpeed);
}

void Asserv::setErrorPositionThreshold(float x, float y, float theta){
    if (x>0)
        m_errorPosThreshold.x = x;
    
    if (y>0)
        m_errorPosThreshold.y = y;

    if (theta>0)
        m_errorPosThreshold.theta = theta;
}


bool Asserv::isAtObjectivePoint(bool checkAngle){
    
    // this->updateError();

    if (abs(m_errorPos.x) > m_errorPosThreshold.x)
        return false;
    else if (abs(m_errorPos.y) > m_errorPosThreshold.y)
        return false;
    else if (checkAngle && (abs(fmod(m_errorPos.theta, TWO_PI)) > m_errorPosThreshold.theta))
        return false;
    else
        return true;
}

void Asserv::loop() {
    // this->updateCommand();

    /*
    Dans l'ordre :

    setRobotPos(x0, y0, theta0);
    setDest( <vars> );

    beginTrajectory(t0);

    updateTrajectory(t);

    getTrajectoryPoint() -> (x, y, theta)
    getTrajectoryLinearSpeed() -> V
    getTrajectoryAngularSpeed() -> omega
    */
}

