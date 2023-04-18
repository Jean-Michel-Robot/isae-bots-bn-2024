/*
   
*/

#include <LinearTrajectory.hpp>

#include <Arduino.h>


LinearTrajectory::LinearTrajectory()
{

    // goalSpeed = 1.0;  // m/s  //TODO à set par le HN
    // accelParam = 1.0;  // m/s^2  //TODO à set par le HN
    // currentSpeed = 0.0;

    // d_current = 0.0;

    float accelParam = 0.2;

}


void LinearTrajectory::setDest(float x0, float y0, float xdest, float ydest) {
    this->x0 = x0;
    this->y0 = 0;
    this->xdest = xdest;
    this->ydest = ydest;

    Dtotale = sqrt((x0 - xdest) * (x0 - xdest) + (y0 - ydest) * (y0 - ydest));
    theta0 = atan2(ydest - y0, xdest - x0);  // returned
 
}



// Update un array de 5 floats [x, y, theta, V, omega] à un temps t
void LinearTrajectory::calculatePointAtTime(uint32_t current_time, float *q)
{
    // Calcul de dt
    float dt = current_time - this->current_time;

    if (dt < 0) {
        Serial.println("ERROR : dt négatif");
        return;
    }

    this->current_time = current_time;

    // On récupère et on stocke V(t) de la rampe
    currentSpeed = rampSpeed.updateRamp(current_time);

    // Calcul de la distance parcourue en dt
    d_current = currentSpeed * dt*0.000001;

    // Ajout à la distance totale parcourue
    d_parc = d_parc + d_current;

    // Calcul de s comme la fraction de distance parcourue sur distance totale
    s = d_parc / Dtotale;


    // Test s >= 1 (rampe terminee)
    if (s >= 1) {
        s = 1;

        //TODO rampe terminée, à envoyer à la SM
    }

    // Test phase finale de rampe (pour une droite)
    if ( detectEndRamp() ) {
        rampSpeed.endRamp(); // request phase finale de rampe
    }

    // Injection de s dans les équations paramétriques
    Position2D pos = calculateTrajCoords(s);

    q[0] = pos.x;
    q[1] = pos.y;
    q[2] = pos.theta;
    q[3] = currentSpeed;
    q[4] = 0.0;
}



float LinearTrajectory::getVelAndTheta(uint32_t current_time) {
    d_current = goalSpeed*(current_time - t0);

    float current_speed = rampSpeed.updateRamp(current_time);

    // float res[2] = {vd, theta0};
    return current_speed;
}

void LinearTrajectory::setGoalSpeed(float goalSpeed) {
    this->goalSpeed = goalSpeed;
    rampSpeed.changeGoalSpeed(goalSpeed);
}