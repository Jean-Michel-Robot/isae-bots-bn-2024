
#include "Ramp.hpp"



Ramp::Ramp(uint32_t t_0, float accelParam) {
    this->accelParam = accelParam;

    this->rampSM = RampSM();  // stack memory (instanciation without new keyword)

    // init accel param using set function
    this->rampSM.setAccelParam(accelParam);

}


float Ramp::getOutputSpeed(uint32_t t_0) {

    //TODO comment utiliser t_0 ?
    
    return rampSM.getCurrentSpeed();
}