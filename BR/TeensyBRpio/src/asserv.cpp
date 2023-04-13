
#include "asserv.hpp"

// #include "ROS.hpp"
// #include "OdosPosition.hpp"

#include <ROS.hpp>
#include "OdosPosition.hpp"


#include "main_loop.hpp"
#include <math.h>

// void asservPID::updatePosition() {

//     robotPos = p_odos->getRobotPosition();

//     p_ros->logPrint(LogType::INFO, "Robot position updated");
// }

void asservPID::updateError()
{
    m_errorPos = m_trajectory.getPointAtTime(micros()) - p_odos->getRobotPosition();
}



void asservPID::updateCommand() {

    m_target = m_trajectory.getSpeed();

    float vd = m_target[0];
    float wd = m_target[1];

    this->updateError();

    if(cos(m_errorPos.tetha) == 0){
        m_cmdV = 100;
    }
    else {   
        m_cmdV[0] = (vd - m_k1*abs(vd)(m_errorPos.x + m_errorPos.y*tan(m_errorPos.tetha)))/cos(m_errorPos.thetha);
    }
     
}

