#ifndef ASSERV_H
#define ASSERV_H

#include <Position2D.h>
#include <LinearTrajectory.hpp>

#include "defines.hpp"


class asservPID
{

public:

    float m_k1, m_k2, m_k3;

    float m_KP, m_KI, m_KD;

    float m_target[2];  // vd, omegad

    Position2D m_errorPos;  // ex, ey, etheta
    float m_botSpeed[2];  // v, omega of the center of the bot

    float m_leftWheelSpeed;
    float m_rightWheelSpeed;

    LinearTrajectory* m_p_trajectory; // DEFINED AS POINTER ?????

    asservPID(float k1, float k2, float k3);

    void updateError();
    void updateCommand();
    void loop();


private:

    float m_Rsb[2][2];  // matrice de passage du repère monde vers le repère 

    // Position2D robotPos;  // x, y, theta

};

#endif  // ASSERV_H
