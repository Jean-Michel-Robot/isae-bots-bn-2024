#ifndef __H_LINEAR_TRAJECTORY
#define __H_LINEAR_TRAJECTORY

#include <Arduino.h>
// #include "a_define.h"
#include <Position2D.h>
#include <Ramp.hpp>


class LinearTrajectory
{
public :
    LinearTrajectory();  // quatre coords définissant le segment + l'origine de temps
    // TODO destructeur

    void setDest(float x0, float y0, float xdest, float ydest);

    void beginTrajectory(uint32_t t_0);

    Position2D getPointAtTime(uint32_t current_time);
    float* getVelAndTheta(uint32_t current_time);


    // protected :


private:

    float velTheta[2];

    Ramp* rampSpeed;

    float x0, y0, xdest, ydest;
    float theta0;
    uint32_t t_0;
    float V;

    float Dtotale;
    float s;

    float d_current;
    float vd;

    
    /*
    s est déduit de t et de V(t)

    x, y, theta sont déduit de s


    Avec une vitesse de consigne variable Vc(t) on ne peut pas connaître x et y en tout point dès le début
    Et le but c'est de ne pas recalculer la trajectoire mais de la suivre en temps réel
    => on calcule les variables pas à pas

    v = sqrt( x'(t)^2 + y'(t)^2 )
    w = theta'(t)

    On part de t et de Vc(t)
    On a aussi t0 et les params de la courbe

    Au prochain appel, avec t et tpred :
    on calcule dt = t - tpred

    Dans SLOPE on update V(t) linéairement :
    deltat = Vc / accelParam
    V(t) = Vc * (t - t0)/deltat
    => V(t) = Vprev + accelParam * (t - t0) en ASC
    => V(t) = Vc - accelParam * (t - tdesc) en DESC
    => V(t) = Vc en CST

    On calcule la distance à parcourir d(t) = V(t) * dt (V(t) est donnée par une rampe de valeur max Vc donnée par le HN)

    On l'ajoute à la distance parcourue :
    d_parc = d_parc + d(t)

    Déclenchement de EndRamp lorsque :
    d_selon_courbe( (x,y), (xdest, ydest)) < d_decel soit 0.5*V(t)^2/accelParam
    -> mieux de prendre la distance par rapport à la dest qu'au début, moins d'erreurs
    Si cette condition est vérifiée dès le début alors on calcule la vitesse max
    à atteindre à savoir sqrt(Dtotale * accelParam)

    Si on a la longueur totale de la courbe (oui en ligne droite)
    Dtotale = sqrt((x0 - xdest)*(x0 - xdest) + (y0 - ydest)*(y0 - ydest))
    theta0 = atan2(ydest - y0, xdest - x0)

    On calcule le s à ajouter :
    ds(t) = d(t)/Dtotale

    On peut alors calculer le nouveau s :
    s(t) = sprev + ds(t)
    OU
    s(t) = (d_parc + d(t)) / Dtotale

    Si s devient supérieur à 1 on le ramène à 1 et on repasse en Idle
    Il faudrait un moyen de l'amener à 1 s'il a tendance à s'arrêter avant

    Et les nouvelles coords :
    x(s) = x0 + s*(xdest - x0)
    y(s) = y0 + s*(ydest - y0)
    theta(s) = theta0 (cst)

    La nouvelle vitesse est Vc(t)
    La vitesse angulaire est ici de 0



    ----
    Plus simple avec la droite

    d(t) peut être utilisée directement sur la droite :
    x(t) = xprev + d(t)*cos(theta(t))
    y(t) = yprev(theta(t))
    theta(t) = theta0

    Et pas besoin de variable s ici

    */


};

#endif
