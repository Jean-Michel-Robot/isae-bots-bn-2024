#ifndef __H_RAMP
#define __H_RAMP

#include "state_machine/Events.hpp"

#define ACCEL_BRAKE 2.0  // m/s^-2
#define RAMP_EPSILON 0.001  // epsilon for values to be considered the same

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
    Ramp();
    // TODO destructeur

    void beginRamp(uint32_t t0, float goalSpeed, float accelParam);

    float updateRamp(uint32_t t);

    void endRamp();
    bool isRampIdle();
    void setToIdle();

    void changeGoalSpeed(float goalSpeed);
    void emergencyBrake();

private :

    BeginRampEvent beginRampEvent;
    GoalSpeedChangeEvent goalSpeedChangeEvent;
    EndRampEvent endRampEvent;
    UpdateEvent updateEvent;  //TODO rename in updateRampEvent
    EmergencyBrakeEvent emergencyBrakeEvent;

};

#endif