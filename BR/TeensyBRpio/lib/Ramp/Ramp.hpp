#ifndef __H_RAMP
#define __H_RAMP

#include <Arduino.h>

#include "RampSM.hpp"

// #include "a_define.h"
// #include <Position2D.h>


class Ramp
{
public :
    Ramp(uint32_t t_0, float accelParam);
    // TODO destructeur


    float getOutputSpeed(uint32_t t_0);

private :

    float accelParam;

    RampSM rampSM;

};

#endif