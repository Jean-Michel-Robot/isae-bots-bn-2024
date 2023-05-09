#ifndef ASSERV_H
#define ASSERV_H

#include <Position2D.h>

#include "defines.hpp"


enum AsservState{
    IDLE,
    ACTIVE,
    BYPASSED
};

class Asserv
{

public:

    float m_Kp, m_Ti, m_Td;

    float m_target[2];  // vd, omegad

    // Position2D m_errorPos;  // ex, ey, etheta
    double m_errorPos_x;
    double m_errorPos_y;


    Position2D m_errorPosThreshold; // error threshold x,y,tetha (m, m & rad)
    float m_botSpeed[2];  // v, omega of the center of the bot (m/s and rad/s)

    float m_leftWheelSpeed;  // left speed wheel (m/s)
    float m_rightWheelSpeed; // right speed wheel (m/s)

    AsservState m_state;

    Asserv(float Kp, float Ti, float Td);

    void setGains(float Kp, float Ti, float Td);

    void setGains(float k1, float k2, float k3);

    // update l'erreur en recevant la position du point objectif actuel
    void updateError(Position2D trajectoryPointPos);

    // update la commande en recevant la consigne en vitesse (linéaire et angulaire)
    void updateCommand(float vd, float omega_d, bool bypassAsserv=0);

    void updateCommand_2(float* ppoint_d, bool bypassAsserv=0);

    void computeOutput(unsigned long t_micro, float* ppoint_d);

    /* Set error position threshold
    x : m, y : m, tetha : rad
    Provide negative value not to update the corresponding direction
    */
    void setErrorPositionThreshold(float x, float y, float theta);

    /*
    */
    bool isAtObjectivePoint(bool checkAngle);

    void loop();

    void calculateSpeeds();



private:


    Position2D currentRobotPos;

    float cmd_v, cmd_omega;

    double error[2] = {0.0};
    float cmd_coordspoint[2] = {0.0};  // xpoint, ypoint dans le repère table
    float xpointd, ypointd;  // vitesse du point objectif dans le repère table

    double m_sumIntegral[2] = {0.0};
    double m_lastMesuredError[2] = {0.0};
    unsigned long m_lastTimeOfCalcul = 0;
    float m_outputMax = 0.0;
    float m_satuIntegrale = 0.0;
    float m_cmdDerivee[2] = {0.0};

    bool m_enableI = true;
    float m_N = 5.0;
    float m_Rsb[2][2];  // matrice de passage du repère monde vers le repère robot

};

#endif  // ASSERV_H
