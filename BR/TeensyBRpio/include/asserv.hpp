#ifndef ASSERV_H
#define ASSERV_H

#include <Position2D.h>



#include "defines.hpp"


enum asservState{
    IDLE,
    ACTIVE
};

class asservPID
{

public:

    float m_k1, m_k2, m_k3;

    float m_KP, m_KI, m_KD;

    float m_target[2];  // vd, omegad

    Position2D m_errorPos;  // ex, ey, etheta
    Position2D m_errorPosThreshold; // error threshold x,y,tetha (m, m & rad)
    float m_botSpeed[2];  // v, omega of the center of the bot (m/s and rad/s)

    float m_leftWheelSpeed;  // left speed wheel (m/s)
    float m_rightWheelSpeed; // right speed wheel (m/s)

    asservState m_state;

    asservPID(float k1, float k2, float k3);

    void updateError();
    void updateCommand();

    /* Set error position threshold
    x : m, y : m, tetha : rad
    Provide negative value not to update the corresponding direction
    */
    void setErrorPositionThreshold(float x, float y, float theta);

    /*
    */
    bool isAtObjectivePoint(bool checkAngle);

    void loop();



private:

    float m_Rsb[2][2];  // matrice de passage du repère monde vers le repère robot

};

#endif  // ASSERV_H
