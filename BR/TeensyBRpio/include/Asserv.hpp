#ifndef ASSERV_H
#define ASSERV_H

#include <Position2D.h>



#include "defines.hpp"


enum AsservState{
    IDLE,
    ACTIVE
};

class Asserv
{

public:

    float m_k1, m_k2, m_k3;

    float m_KP, m_KI, m_KD;

    float m_target[2];  // vd, omegad

    Position2D m_errorPos;  // ex, ey, etheta
    float m_botSpeed[2];  // v, omega of the center of the bot (m/s and rad/s)

    float m_leftWheelSpeed;  // left speed wheel (m/s)
    float m_rightWheelSpeed; // right speed wheel (m/s)

    AsservState m_state;

    Asserv(float k1, float k2, float k3);

    void updateError();
    void updateCommand();
    void loop();


private:

    float m_Rsb[2][2];  // matrice de passage du repère monde vers le repère robot

};

#endif  // ASSERV_H
