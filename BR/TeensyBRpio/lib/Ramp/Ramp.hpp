#ifndef __H_RAMP
#define __H_RAMP

#include <Arduino.h>

#include "RampSM.hpp"
#include "Events.hpp"

// #include "a_define.h"
// #include <Position2D.h>

/*
Une rampe est utilisée sur n'importe quelle trajectoire pour générer une 
vitesse de commande (linéaire ou angulaire) avec des rampes
Il faut lui fournir l'event de fin de rampe pour savoir quand amorcer la
décélération de fin (condition sur la distance/distance angulaire restante à parcourir)
car on ne calcule pas tout le profil de vitesse comme avant
*/
class Ramp
{
public :
    Ramp(float accelParam);
    // TODO destructeur

    void beginRamp(uint32_t t0, float goalSpeed);

    float updateRamp(uint32_t t);

    void endRamp();

    void changeGoalSpeed(float goalSpeed);
    void emergencyBrake();

private :

    float accelParam;

    RampSM rampSM;

    GoalSpeedChangeEvent goalSpeedChangeEvent;
    EndRampEvent endRampEvent;
    UpdateEvent updateEvent;
    EmergencyBrakeEvent emergencyBrakeEvent;

};

#endif