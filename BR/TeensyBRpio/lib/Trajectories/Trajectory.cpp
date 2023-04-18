/*
   
*/

#include <Trajectory.hpp>

Trajectory::Trajectory()
{
    x = 0.0; y = 0.0; theta = 0.0; V = 0.0; omega = 0.0;
    x0 = 0.0; y0 = 0.0; theta0 = 0.0;

    goalSpeed = 1.0;  // m/s  //TODO à set par le HN
    accelParam = 1.0;  // m/s^2  //TODO à set par le HN
    currentSpeed = 0.0;

    d_current = 0.0;
    Dtotale = 0.0;

    float accelParam = 0.2;

    rampSpeed = Ramp(accelParam);  //TODO : static object or change it to dynamic ?
                                   // Or LinearTrajectory is dynamic but this is static ?
}

Trajectory::~Trajectory() {
}

bool Trajectory::detectEndRamp() {
    return false;
}

void Trajectory::setRobotPos(float x0, float y0, float theta0) {
    this->x0 = x0; this->y0 = y0; this->theta0 = theta0;
}

void Trajectory::beginTrajectory(uint32_t t0) {
    this->t0 = t0;

    x = x0; y = y0; theta = theta0;

    current_time = t0;
    d_parc = 0.0;
    currentSpeed = 0.0;  // une trajectoire commence toujours à vitesse nulle

    rampSpeed.beginRamp(t0, goalSpeed);
}



// // Update un array de 5 floats [x, y, theta, V, omega] à un temps t
// void Trajectory::calculatePointAtTime(uint32_t current_time, float *q)
// {
//     // Calcul de dt
//     float dt = current_time - this->current_time;

//     if (dt < 0) {
//         Serial.println("ERROR : dt négatif");
//         return;
//     }

//     this->current_time = current_time;

//     // On récupère et on stocke V(t) de la rampe
//     currentSpeed = rampSpeed.updateRamp(current_time);

//     // Calcul de la distance parcourue en dt
//     d_current = currentSpeed * dt*0.000001;

//     // Ajout à la distance totale parcourue
//     d_parc = d_parc + d_current;

//     // Calcul de s comme la fraction de distance parcourue sur distance totale
//     s = d_parc / Dtotale;


//     // Test s >= 1 (rampe terminee)
//     if (s >= 1) {
//         s = 1;

//         //TODO rampe terminée, à envoyer à la SM
//     }

//     // Test phase finale de rampe (pour une droite)
//     if ( detectEndRamp() ) {
//         rampSpeed.endRamp(); // request phase finale de rampe
//     }

//     // Injection de s dans les équations paramétriques
//     Position2D pos = calculateTrajCoords(s);

//     q[0] = pos.x;
//     q[1] = pos.y;
//     q[2] = pos.theta;
//     q[3] = currentSpeed;
//     q[4] = 0.0;
// }


// // Update un array de 5 floats [x, y, theta, V, omega] à un temps t
void Trajectory::updateTrajectory(uint32_t current_time)
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

    // Application des équations paramétriques avec s
    // et attribution des vitesses (linéaire et angulaire)
    updateTrajectoryState();

}

void Trajectory::setGoalSpeed(float goalSpeed) {
    this->goalSpeed = goalSpeed;
    rampSpeed.changeGoalSpeed(goalSpeed);
}


Position2D Trajectory::getTrajectoryPoint() {
    return Position2D(x, y, theta);
}

float Trajectory::getTrajectoryLinearSpeed() {
    return V;
}

float Trajectory::getTrajectoryAngularSpeed() {
    return omega;
}