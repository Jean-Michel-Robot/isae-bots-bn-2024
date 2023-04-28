/*
   
*/

#include "Trajectories/Trajectory.hpp"

#include <defines.hpp>

#include "ROS.hpp"
#include "main_loop.hpp"

Trajectory::Trajectory()
{
    x = 0.0; y = 0.0; theta = 0.0; V = 0.0; omega = 0.0;
    x0 = 0.0; y0 = 0.0; theta0 = 0.0;

    currentSpeed = 0.0;
    d_current = 0.0;
    Dtotale = 0.0;

    trajectoryType = TrajectoryType::TRAJ_UNDEF;

    rampSpeed = Ramp(accelParam);  //TODO : static object or change it to dynamic ?
                                   // Or LinearTrajectory is dynamic but this is static ?
}

Trajectory::~Trajectory() {
}

bool Trajectory::detectEndRamp() {
    return false;
}

// The trajectory is active when the rampSpeed is not Idle
bool Trajectory::isTrajectoryActive() {
    return !rampSpeed.isRampIdle();
}

void Trajectory::setRobotPos(Position2D pos) {
    this->x0 = pos.x;
    this->y0 = pos.y;
    this->theta0 = pos.theta;
}

void Trajectory::beginTrajectory(uint32_t t0) {
    this->t0 = t0;

    x = x0; y = y0; theta = theta0;

    current_time = t0;
    d_parc = 0.0;
    currentSpeed = 0.0;  // une trajectoire commence toujours à vitesse nulle

    rampSpeed.beginRamp(t0, goalSpeed);
}


// Update les variables de la trajectoire [x, y, theta, V, omega] à un temps t
void Trajectory::updateTrajectory(uint32_t new_time)
{
    // Calcul de dt
    // uint32_t previous_time = this->current_time;
    uint32_t dt = new_time - current_time;

    if (dt < 0) {
        //Serial.println("ERROR : dt négatif");
        return;
    }

    current_time = new_time;

    // On récupère et on stocke V(t) de la rampe
    currentSpeed = rampSpeed.updateRamp(current_time);

    // Ajout de la distance parcourue en dt à la distance totale parcourue
    d_parc = d_parc + currentSpeed * dt*0.000001;

    // Calcul de s comme la fraction de distance parcourue sur distance totale
    s = d_parc / Dtotale;


    // Test s >= 1 (rampe terminee)
    if (s >= 1) {
        s = 1;
        //Serial.println("s >= 1");
        /*
        Sécurité : on met la commande de vitesse à 0
        (si jamais on a pas décéléré à temps on essaie quand même
        de s'arrêter, même si l'asserv risque d'en prendre un coup)
        */
        currentSpeed = 0;
        V = 0.0;
        omega = 0.0;


        /*
        En fait la terminaison de rampe se déclenche quand la vitesse est 
        redevenue nulle, ce qui peut arriver de deux manières :
        - depuis rampSM, on a descendu la vitesse en-dessous de 0
        - depuis Trajectory, on a atteint s = 1 donc on est au bout de la 
        trajectoire, on doit alors mettre la vitesse à 0

        Ici on implémente la mise en Idle de rampSM si s a dépassé 1
        */
        rampSpeed.setToIdle(); //TOTEST
    }

    // Test phase finale de rampe
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

    // transform coordinates to the asserv tracking point frame
    float x_p = x + ASSERV_ALPHA*cos(theta);
    float y_p = y + ASSERV_BETA*sin(theta);
    
    return Position2D(x_p, y_p, theta);
}

float Trajectory::getTrajectoryLinearSpeed() {
    return V;
}

float Trajectory::getTrajectoryAngularSpeed() {
    return omega;
}


// default def
void Trajectory::updateTrajectoryState() {
    //Serial.println("ERROR : default implementation of updateTrajectoryState");
}

void Trajectory::setDest(OrderType order) {
    //Serial.println("ERROR : default implementation of setDest");
}