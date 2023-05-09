
#include "Asserv.hpp"

#include "OdosPosition.hpp"
#include <Motors.hpp>

#include "ROS.hpp"
#include "main_loop.hpp"

#include <Arduino.h>
#include <cmath>




Asserv::Asserv(float Kp, float Ti, float Td) {
    m_Kp = Kp;
    m_Ti = Ti;
    m_Td = Td;

    m_errorPosThreshold.x = DEFAULT_OBJECTIVE_THRESHOLD_X;
    m_errorPosThreshold.y = DEFAULT_OBJECTIVE_THRESHOLD_Y;
    m_errorPosThreshold.theta = DEFAULT_OBJECTIVE_THRESHOLD_THETA;

    m_state = ACTIVE; //TODO idle by default ?

    currentRobotPos = Position2D(0.0, 0.0, 0.0);
    cmd_v = 0.0; cmd_omega = 0.0;

}


void Asserv::setGains(float Kp, float Ti, float Td) {
    m_Kp = Kp;
    m_Ti = Ti;
    m_Td = Td;
}

void Asserv::updateError(Position2D trajectoryPointPos) {

    //TODO maybe put this elsewhere, should be done at every loop
    currentRobotPos = p_odos->getRobotPosition();

    // Position2D currentBotPosition = p_odos->getRobotPosition();

    // calcul de l'erreur : on passe la pos du robot au point (alpha, beta)
    // puis on soustrait à la position du point donné par la trajectoire

    //TOTEST CHECK IF REF CHANGE IS OK (avec alpha et beta aussi)
    // m_errorPos.x = cos(angle)*errorPosTableFrame.x + sin(angle)*errorPosTableFrame.y;
    // m_errorPos.y = -sin(angle)*errorPosTableFrame.x + cos(angle)*errorPosTableFrame.y;
    // m_errorPos.theta = errorPosTableFrame.theta;
    error[0] = (double) trajectoryPointPos.x - (currentRobotPos.x + ASSERV_ALPHA*cos(currentRobotPos.theta));
    error[1] = (double) trajectoryPointPos.y - (currentRobotPos.y + ASSERV_ALPHA*sin(currentRobotPos.theta));
    //TODO assuming ASSERV_BETA is 0 here, could generalize
    //TODO utiliser le fait que l'erreur est un double
}


/*
This function takes an input command and sends a command to the motors
- If the asserv state is ACTIVE the command is transformed using the asserv formula
- If the asserv state is BYPASSED the command is send directly to the motors
  (the asserv can also be bypassed if the bypassAsserv argument is set to true)
- If the asserv state is IDLE the command is not used and a command of zero is sent
*/
void Asserv::updateCommand(float vd, float omega_d, bool bypassAsserv) {

    // update trajectory
    // p_linearTrajectory->updateTrajectory( micros() );

    m_state = BYPASSED; //FORTEST


    if(m_state == ACTIVE) {

        // /* En utilisant la formule qu'on sait pas d'où elle sort*/
        // if(cos(m_errorPos.theta) == 0){
        //     // Protection div par 0 (ça peut servir)
        //     p_ros->logPrint(ERROR, "Division by 0 in asserv");
        //     m_botSpeed[0] = 0;
        //     m_botSpeed[1] = 0;
        // }
        // else {   
        //     m_botSpeed[0] = (vd - m_k1*abs(vd)*(m_errorPos.x + m_errorPos.y*tan(m_errorPos.theta)))/cos(m_errorPos.theta);
        //     m_botSpeed[1] = omega_d - (m_k2*vd*m_errorPos.y  + m_k3*abs(vd)*tan(m_errorPos.theta))*pow(cos(m_errorPos.theta),2);
        // }

        m_leftWheelSpeed = m_botSpeed[0] + m_botSpeed[1]*WHEEL_DISTANCE/2; 
        m_rightWheelSpeed = m_botSpeed[0] - m_botSpeed[1]*WHEEL_DISTANCE/2;


    }

    else if (m_state == BYPASSED || bypassAsserv == ASSERV_BYPASSED) {

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

void Asserv::setErrorPositionThreshold(float x, float y, float theta) {
    if (x > 0)
        m_errorPosThreshold.x = x;
    
    if (y > 0)
        m_errorPosThreshold.y = y;

    if (theta > 0)
        m_errorPosThreshold.theta = theta;
}


bool Asserv::isAtObjectivePoint(bool checkAngle){
    
    // this->updateError();

    if (abs(error[0]) > m_errorPosThreshold.x)
        return false;
    else if (abs(error[1]) > m_errorPosThreshold.y)
        return false;
    // else if (checkAngle && (abs(fmod(m_errorPos.theta, TWO_PI)) > m_errorPosThreshold.theta))
    //     return false;
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


    currentRobotPos = p_odos->getRobotPosition(); // avant le reste

    //il faut que la trajectoire sopit update


    computeOutput( micros() , NULL);
    calculateSpeeds();

    // send commands
    /*

    update error

    ??
    en fct de si l'asserv est bypassed ou pas
    */
}


void Asserv::updateCommand_2(float* ppoint_d, bool bypassAsserv) {


    if(m_state == ACTIVE) {

        // il faut que la trajectoire et l'erreur soient update avant

        computeOutput( micros(), ppoint_d);
        calculateSpeeds();


        // conversion en vitesse des roues
        m_leftWheelSpeed = cmd_v + cmd_omega*WHEEL_DISTANCE/2;
        m_rightWheelSpeed = cmd_v - cmd_omega*WHEEL_DISTANCE/2;

    }

    // else if (m_state == BYPASSED || bypassAsserv == ASSERV_BYPASSED) {

    //     // asserv is bypassed and we directly feed the command
    //     m_leftWheelSpeed = vd + omega_d*WHEEL_DISTANCE/2;
    //     m_rightWheelSpeed = vd - omega_d*WHEEL_DISTANCE/2;
    // }

    else { // asserv is idle

        //TODO make the motors stay still
        m_leftWheelSpeed = 0.0;
        m_rightWheelSpeed = 0.0;
    }

    // send the commands to the motors
    sendMotorCommand(BR_LEFT, m_leftWheelSpeed);
    sendMotorCommand(BR_RIGHT, m_rightWheelSpeed);
}


void Asserv::computeOutput(unsigned long t_micro, float* ppoint_d) {

    if (m_lastTimeOfCalcul > t_micro)
        // TODO error case
        return;
        // return constrain(m_Kp * error + m_cmdDerivee + m_sumIntegral, -m_outputMax, m_outputMax);
        // on renvoie la valeur identique a la derniere iteration

    double deltaT = (double)(t_micro - m_lastTimeOfCalcul) *1e-6;
    m_lastTimeOfCalcul = t_micro;


    for (int k=0; k<2; k++) {  // boucle sur les deux composantes du vecteur cmd_coordspoint

        if (m_Ti != 0.0 && m_enableI) {

            m_sumIntegral[k] += m_Kp / m_Ti * deltaT * error[k];
            m_sumIntegral[k] = constrain(m_sumIntegral[k], -m_satuIntegrale, m_satuIntegrale);

        } else {
            m_sumIntegral[k] = 0.0;
        }
        if (m_Kp != 0.0 && m_Td != 0.0)
            m_cmdDerivee[k] = constrain(m_Kp * m_Td * (error[k] - m_lastMesuredError[k] + (m_cmdDerivee[k] / (m_N * m_Kp))) / (deltaT + m_Td / m_N) , -m_outputMax, m_outputMax);
        else{
            m_cmdDerivee[k] = 0.0;
        }
        float commande = ppoint_d[k] + m_Kp * error[k] + float (m_sumIntegral[k]) + m_cmdDerivee[k];
        m_lastMesuredError[k] = error[k];

        cmd_coordspoint[k] = constrain(commande, -m_outputMax, m_outputMax);

    }
}


void Asserv::calculateSpeeds() {
    float theta = currentRobotPos.theta;

    cmd_v =((ASSERV_ALPHA*cos(theta) - ASSERV_BETA*sin(theta))*cmd_coordspoint[0]
         + (ASSERV_ALPHA*sin(theta) + ASSERV_BETA*cos(theta))*cmd_coordspoint[1])
           /ASSERV_ALPHA;

    cmd_omega = (-sin(theta)*cmd_coordspoint[0] + cos(theta)*cmd_coordspoint[1])/ASSERV_ALPHA;
}