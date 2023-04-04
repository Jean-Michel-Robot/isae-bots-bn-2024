
#include "asserv.hpp"


void asservPID::updateError(float errorPos[3])
{
    for (int k = 0; k < 3; k++) {
        this->errorPos[k] = errorPos[k];
    }
}


void asservPID::updateCommand() {

    // cmdV[0] = vd + KP*errorPos[0]
     
}