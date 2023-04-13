#ifndef ASSERV_H
#define ASSERV_H

#include <Position2D.h>
#include <LinearTrajectory.hpp>


class asservPID
{

public:

    float m_k1, m_k2, m_k3;

    float m_KP, m_KI, m_KD;

    float m_target[2];  // vd, omegad

    Position2D m_errorPos;  // ex, ey, etheta
    float m_cmdV[2];  // v, omega

    LinearTrajectory m_trajectory;

    void updateError();
    void updateCommand();


private:

    float m_Rsb[2][2];  // matrice de passage du repère monde vers le repère 

    // Position2D robotPos;  // x, y, theta

};

#endif  // ASSERV_H
