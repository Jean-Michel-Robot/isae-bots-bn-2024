/*
   
*/

#include <LinearTrajectory.hpp>

LinearTrajectory::LinearTrajectory()
{

    goalSpeed = 1.0;  // m/s

    d_current = 0.0;

    float accelParam = 0.2;

    rampSpeed = Ramp(accelParam);  //TODO : static object or change it to dynamic ?
                                   // Or LinearTrajectory is dynamic but this is static ?
}


void LinearTrajectory::setDest(float x0, float y0, float xdest, float ydest) {
    this->x0 = x0;
    this->y0 = 0;
    this->xdest = xdest;
    this->ydest = ydest;

    Dtotale = sqrt((x0 - xdest) * (x0 - xdest) + (y0 - ydest) * (y0 - ydest));
    theta0 = atan2(ydest - y0, xdest - x0);  // returned
 
}


void LinearTrajectory::beginTrajectory(uint32_t t0) {
    this->t0 = t0;

    rampSpeed.beginRamp(t0, goalSpeed);

}



// Retourne (xd(t), yd(t), thetad(t)) dans une Position2D
Position2D LinearTrajectory::getPointAtTime(uint32_t current_time)  //TODO general class for this 
{

    // On récupère V(t) de la rampe
    float current_speed = rampSpeed.updateRamp(current_time);

    d_current = goalSpeed * (current_time - t0)*0.000001;
    d_parc = d_parc + d_current;

    s = d_parc / Dtotale;

    // Test s > 1
    if (s >= 1) {
        s = 1;
        rampeTerminee; // event de fin de trajectoire
    }

    // Injection de s dans les équations paramétriques
    float x = x0 + s*(xdest - x0);
    float y = y0 + s*(ydest - y0);
    float theta = theta0;

    Position2D pos = Position2D(x, y, theta);

    return pos;
}

float LinearTrajectory::getVelAndTheta(uint32_t current_time) {
    d_current = goalSpeed*(current_time - t0);

    float current_speed = rampSpeed.updateRamp(current_time);

    // float res[2] = {vd, theta0};
    return current_speed;
}