/*
   
*/

#include "Trajectories/Trajectory.hpp"

#include <defines.hpp>
#include <GeometricTools.hpp>

#include "ROS.hpp"
#include "main_loop.hpp"

Trajectory::Trajectory()
{
    x = 0.0; y = 0.0; theta = 0.0; V = 0.0; omega = 0.0;
    x0 = 0.0; y0 = 0.0; theta0 = 0.0;

    currentSpeed = 0.0;
    Dtotale = 0.0;

    trajectoryType = TrajectoryType::TRAJ_UNDEF;

    rampSpeed = Ramp(accelParam);  //TODO : static object or change it to dynamic ?
                                   // Or LinearTrajectory is dynamic but this is static ?
}

Trajectory::~Trajectory() {
}


// The trajectory is active when the rampSpeed is not Idle
bool Trajectory::isTrajectoryActive() {
    return !rampSpeed.isRampIdle();
}

// We make sure the trajectory goal point starts exactly on the robot pos
// so that the error at the start is 0
void Trajectory::setRobotPos(Position2D pos) {
    this->x0 = pos.x;
    this->y0 = pos.y;
    this->theta0 = pos.theta;
}

void Trajectory::beginTrajectory(uint32_t t0) {
    this->t0 = t0;

    x = x0; y = y0; theta = theta0;

    current_time = t0;
    currentSpeed = 0.0;  // une trajectoire commence toujours à vitesse nulle
    s = 0;  // on commence au début de la trajectoire

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

    if (Dtotale == 0.0) { //TODO make it so that never happens (no update when Dtotale is small)
        // p_ros->logPrint(ERROR, "Dtotale is zero in updateTrajectory");
        return;
    }

    // TOTEST Update de s par l'ajout d'une fraction de la distance totale parcourue    
    s = s + (currentSpeed * dt*0.000001) / Dtotale;

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
    // et attribution des vitesses (relatives et absolues)
    // Cette fonction dépend du type de trajectoire
    updateTrajectoryState();
}


bool Trajectory::detectEndRamp() {

    // On utilise la fraction de Dtotale donnée par s pour savoir quand s'arrêter
    return ( Dtotale * (1 - s) < 0.5*currentSpeed*currentSpeed/accelParam );
}



void Trajectory::setGoalSpeed(float goalSpeed) {
    this->goalSpeed = goalSpeed;
    rampSpeed.changeGoalSpeed(goalSpeed);
}


Position2D Trajectory::getGoalPoint() {

    // transform coordinates to the asserv tracking point frame
    return Position2D(x, y, theta);
}

Position2D Trajectory::getGoalOffsetPoint() {

    // transform coordinates to the asserv tracking point frame
    return toAsservPointFrame(Position2D(x, y, theta));
}



float Trajectory::getTrajectoryLinearSpeed() {
    return V;
}

float Trajectory::getTrajectoryAngularSpeed() {
    return omega;
}

// returns an array of floats [xpoint, ypoint]
float* Trajectory::getTrajectoryAbsoluteSpeed() {
    return ppoint_d;
}


// default def
void Trajectory::updateTrajectoryState() {
    p_ros->logPrint(ERROR, "Shouldn't use default implementation of updateTrajectoryState");
}

void Trajectory::setDest(Position2D orderInfo) {
    p_ros->logPrint(ERROR, "Shouldn't use default implementation of setDest");
}