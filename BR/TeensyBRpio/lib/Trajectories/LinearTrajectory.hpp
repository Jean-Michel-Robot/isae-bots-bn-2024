#ifndef __H_LINEAR_TRAJECTORY
#define __H_LINEAR_TRAJECTORY

// #include <Arduino.h>
// #include "a_define.h"
#include <Position2D.h>


class LinearTrajectory
{
public :
    LinearTrajectory(uint32_t t_0, float x0, float y0, float xdest, float ydest);  // quatre coords définissant le segment + l'origine de temps
    // TODO destructeur

    Position2D getPointAtTime(uint32_t current_time);

// protected :


private:

    uint32_t t_0;

    uint32_t t_pred;

    // pour une traj générale on ne connait pas la distance totale

    // t_next = t_pred + 

    //NOTE avec la distance totale de la ligne droite

    // s = 0 a l'init, il va de 0 à 1

    // s = d/Dtotale
    // s = s_prev + V*(t - t_pred)/Dtotale
    //NOTE : pour l'instant on peut mettre un vilain échelon de consigne et voir si l'asserv peut le gérer

    float u, v;  // coefs directeurs de la droite
    // x = x0 + u(t - t0)
    // y = y0 + v(t - t0)
    // theta = theta0 (constant)

    float x0, y0, theta0;  // point initial de la droite

};

#endif
