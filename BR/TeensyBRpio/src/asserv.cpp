
#include "asserv.hpp"

// #include "ROS.hpp"
// #include "OdosPosition.hpp"

#include <ROS.hpp>
#include "OdosPosition.hpp"


#include "main_loop.hpp"


void asservPID::updatePosition() {

    robotPos = p_odos->getRobotPosition();

    p_ros->logPrint(LogType::INFO, "Robot position updated");
}

void asservPID::updateError(float errorPos[3])
{
    for (int k = 0; k < 3; k++) {
        this->errorPos[k] = errorPos[k];
    }
}



void asservPID::updateCommand() {

    // cmdV[0] = vd + KP*errorPos[0]
     
}

