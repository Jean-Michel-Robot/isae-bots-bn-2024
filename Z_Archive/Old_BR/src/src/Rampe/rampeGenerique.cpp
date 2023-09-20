#include "rampeGenerique.h"
#ifndef ARDUINO
        using std::string;
        using std::to_string;
#endif

RampeGenerique::RampeGenerique(float acceleration, float decelerationSpeedShift, float decelerationArret,float goalSpeed)
{				//constructeur
    setAccDecc(acceleration,decelerationSpeedShift,decelerationArret);
    m_goalSpeed = goalSpeed;
}
RampeGenerique::RampeGenerique(float accelerationHard, float deccelerationSpeedShiftHard, float deccelerationArretHard,float accelerationSmooth, float deccelerationSpeedShiftSmooth, float deccelerationArretSmooth,float goalSpeed)
{				//constructeur
    setAccDecc(accelerationHard, deccelerationSpeedShiftHard,deccelerationArretHard,accelerationSmooth, deccelerationSpeedShiftSmooth,deccelerationArretSmooth);
    m_goalSpeed = goalSpeed;
}

void RampeGenerique::setAccDecc(float accelerationHard, float deccelerationSpeedShiftHard, float deccelerationArretHard,float accelerationSmooth, float deccelerationSpeedShiftSmooth, float deccelerationArretSmooth)
{
    if (accelerationHard <= 0. || deccelerationArretHard <= 0.0 || accelerationSmooth <= 0. || deccelerationArretSmooth <= 0.0)
            return;
    if (deccelerationSpeedShiftHard < deccelerationArretHard)
            deccelerationSpeedShiftHard = deccelerationArretHard;
    if (deccelerationSpeedShiftSmooth < deccelerationArretSmooth)
            deccelerationSpeedShiftSmooth = deccelerationArretSmooth;
    m_accelerationHard = accelerationHard;
    m_deccelerationSpeedShiftHard = deccelerationSpeedShiftHard;
    m_deccelerationStopHard = deccelerationArretHard;
    m_accelerationSmooth = accelerationSmooth;
    m_deccelerationSpeedShiftSmooth = deccelerationSpeedShiftSmooth;
    m_deccelerationStopSmooth = deccelerationArretSmooth;
}

void RampeGenerique::setAccDecc(float acceleration, float decelerationSpeedShift, float decelerationArret)
{
    setAccDecc(acceleration,decelerationSpeedShift,decelerationArret,acceleration/2,decelerationSpeedShift/2,decelerationArret/2);
}

void RampeGenerique::setSpeed(float speed, float time){				// on defini la vitesse max a obtenir, on peut la modifier a la volée
        if (speed <= 0.0 || speed == m_goalSpeed)
                return;
        if (m_etat != ETAT_RUNNING ) {	// si une rampe n'est pas en cours (non demarrée)
                m_posLinStart = 0.0;
                m_goalSpeed = speed;
        } else {		// on recalcule les parametres avec deux decelerations
            // on va calculer la vitesse et la position actuelle, et refaire une rampe avec une vitesse de départ non nulle
            m_goalSpeed = speed;
            float currentSpeed = calcSpeed(time);
            float startPos = _calcLinPos(time);
            if (m_posLinFin > startPos && currentSpeed != 0.0) // si on est pas arrivé, on réadapte la rampe
            {
                m_posLinStart = startPos;
                _calcTimesDist(m_posLinFin - m_posLinStart, time, currentSpeed, m_endSpeed);
            }
        }
}

void RampeGenerique::_calcTimesDist(float distLeft, float currentTime,float currentSpeed, float endSpeed)
{				// cette fonction calcule les temps et distances des changements de phase
        if (m_etat != ETAT_WAITING_START && m_etat != ETAT_RUNNING) {	// on verifie qu'on est bien sensé recalculer les temps
                return;
        }
        //la fonction est appelee soit au demarrage de la rampe, soit a un changement de vitesse
        m_startTime = currentTime;
        m_startSpeed = currentSpeed;
        m_endSpeed = endSpeed;
        if (currentSpeed < m_goalSpeed) {	// si on doit effectuer une phase d'acceleration (!= du cas 2)
                m_timeFinAccelerationInitiale = currentTime + (m_goalSpeed - currentSpeed) / m_acceleration;
                m_distFinAccelerationInitiale = (sq(m_goalSpeed) - sq(currentSpeed)) / (2 * m_acceleration);
                m_distDebutFreinage = 	distLeft - ((sq(m_goalSpeed) - sq(endSpeed)) / (2 * m_deccelerationStop));
                m_timeDebutFreinage = m_timeFinAccelerationInitiale + 1.0 / m_goalSpeed * (m_distDebutFreinage - m_distFinAccelerationInitiale);

                if (m_timeDebutFreinage < m_timeFinAccelerationInitiale) {	// si on a pas le temps d'etre etabli (cas 3)
                        m_vitesseEtablie = sqrt((distLeft + sq(endSpeed) / (2*m_deccelerationStop) + sq(currentSpeed) / (2*m_acceleration)) / (1/(2 * m_acceleration) + 1/(2 * m_deccelerationStop)));
                        m_timeDebutFreinage = currentTime + (m_vitesseEtablie - currentSpeed) / m_acceleration;
                        m_timeFinAccelerationInitiale = m_timeDebutFreinage;
                        m_timeEnd = (m_vitesseEtablie - endSpeed)/m_deccelerationStop + m_timeDebutFreinage;
                        m_distDebutFreinage = (sq(m_vitesseEtablie) - sq(currentSpeed)) / (2 * m_acceleration);
                        m_distFinAccelerationInitiale = m_distDebutFreinage;
                        m_cas = CAS_GOAL_SPEED_NON_ATTEINT;
                } else {
                        m_vitesseEtablie = m_goalSpeed;
                        m_timeEnd = (m_vitesseEtablie - endSpeed)/m_deccelerationStop + m_timeDebutFreinage;
                        m_cas = CAS_COMPLET;
                }
        }else{		// cas 2, deux decelerations successives
                m_vitesseEtablie = m_goalSpeed;
                m_timeFinAccelerationInitiale = currentTime + (currentSpeed - m_goalSpeed) / m_deccelerationSpeedShift; // on considère la 1e deceleration comme l'acceleration initiale
                m_distFinAccelerationInitiale = (sq(currentSpeed) - sq(m_goalSpeed)) / (2 * m_deccelerationSpeedShift);
                m_distDebutFreinage = distLeft - (sq(m_goalSpeed) - sq(endSpeed)) / (2 * m_deccelerationStop);
                m_timeDebutFreinage = m_timeFinAccelerationInitiale + 1 / m_goalSpeed * (m_distDebutFreinage - m_distFinAccelerationInitiale);
                m_timeEnd = (m_goalSpeed - endSpeed) / m_deccelerationStop + m_timeDebutFreinage;
                m_cas = CAS_DOUBLE_DECELERATION;
        }
}

float RampeGenerique::calcSpeed(float time) const{				// calcul de la vitesse actuelle (commun angle/distance)
        if (m_etat != ETAT_RUNNING)	// si la rampe n'est pas demarrée
                return 0.0;
        if (time >= m_timeEnd)
                return m_endSpeed;
        if (time < m_startTime)
                return m_startSpeed;
        if (time < m_timeFinAccelerationInitiale && m_cas != CAS_DOUBLE_DECELERATION) {// phase acceleration initiale
                return m_startSpeed + m_acceleration * (time - m_startTime);
        }
        if (m_cas == CAS_DOUBLE_DECELERATION && time < m_timeFinAccelerationInitiale) {// phase de la 1ere deceleration (après changementde vitesse objectif à la volée)
                return m_startSpeed - m_deccelerationSpeedShift * (time - m_startTime);
        }
        if (time >= m_timeFinAccelerationInitiale && time < m_timeDebutFreinage) { // phase a vitesse constante
                return m_vitesseEtablie;
        }
        if (time >= m_timeDebutFreinage) {// phase deceleration
                return m_vitesseEtablie - m_deccelerationStop * (time - m_timeDebutFreinage);
        }
        return 0.0;
}

float RampeGenerique::_calcLinPos(float time) const
{				// calcul de la position projetée
        if (m_etat != ETAT_RUNNING)	// si la rampe n'est pas demarrée
                return 0.0;
        if(m_posLinFin-m_posLinStart <= 0.0) // trajet de duree nulle
            return m_posLinFin;
        if (time < m_startTime)
                return m_posLinStart;
        if (time >= m_timeEnd)
                return m_posLinFin + m_endSpeed * (time - m_timeEnd);
        if (time < m_timeFinAccelerationInitiale && m_cas != CAS_DOUBLE_DECELERATION) { // phase acceleration initiale
                return m_posLinStart + m_startSpeed * (time - m_startTime) +
                    m_acceleration * sq(time - m_startTime) * 0.5;
        }
        if (m_cas == CAS_DOUBLE_DECELERATION && time < m_timeFinAccelerationInitiale) { // phase de la 1ere deceleration (après changementde vitesse objectif à la volée)
                return m_posLinStart + m_startSpeed * (time - m_startTime) -
                    m_deccelerationSpeedShift * sq(time - m_startTime) * 0.5;
        }
        if (time >= m_timeFinAccelerationInitiale && time < m_timeDebutFreinage) { // phase a vitesse constante
                return m_posLinStart + m_distFinAccelerationInitiale + m_vitesseEtablie * (time - m_timeFinAccelerationInitiale);
        }
        if (time >= m_timeDebutFreinage) { // phase deceleration
                return m_posLinStart + m_distDebutFreinage + m_vitesseEtablie * (time - m_timeDebutFreinage) -
                    m_deccelerationStop * sq(time - m_timeDebutFreinage) * 0.5;
        }
        return 0.0;
}

void RampeGenerique::_setLinParcours(float posLinStart, float posLinEnd,DynamicType dynamicType,float startSpeed,float endSpeed)
{    
    switch(dynamicType)// on set les accelerations
    {
    case FAST:
        m_acceleration = m_accelerationHard;
        m_deccelerationStop = m_deccelerationStopHard;
        m_deccelerationSpeedShift = m_deccelerationSpeedShiftHard;
        break;
    case SMOOTH:
        m_acceleration = m_accelerationSmooth;
        m_deccelerationStop = m_deccelerationStopSmooth;
        m_deccelerationSpeedShift = m_deccelerationSpeedShiftSmooth;
        break;
    case BRAKE :
        m_acceleration = m_accelerationHard;
        m_deccelerationStop = m_deccelerationBrake;
        m_deccelerationSpeedShift = m_deccelerationBrake;
        break;
    }
    m_posLinStart = posLinStart;
    m_posLinFin = posLinEnd;
    m_startSpeed = startSpeed;
    m_endSpeed = endSpeed;
    m_etat = ETAT_WAITING_START;
}

void RampeGenerique::startParcours(float time){				// on lance la rampe
        if (m_etat != ETAT_WAITING_START)
                return;
    _calcTimesDist(m_posLinFin, time, m_startSpeed, m_endSpeed);	// on calcule les temps et distances de transition
    m_etat = ETAT_RUNNING;
}

bool RampeGenerique::isFinished(float time)const{				// teste si la rampe est en cours ou pas
        if (m_etat != ETAT_RUNNING)
                return true;
        return (time > m_timeEnd);
}

bool RampeGenerique::isWorking(float time) const{				// retourne false si il y a une erreur interne a la rampe
        if (m_etat != ETAT_RUNNING)	// si la rampe n'est pas initialisee
                return false;
    if (time < m_startTime - 10.0)	// si retour dans le temps (overflow de micros())
                return false;
        if (!s_isValid(m_distFinAccelerationInitiale) || !s_isValid(m_distDebutFreinage) || !s_isValid(m_distEnd)
            || !s_isValid(m_startTime) || !s_isValid(m_timeFinAccelerationInitiale)
            || !s_isValid(m_timeDebutFreinage) || !s_isValid(m_timeEnd)
            || !s_isValid(m_startSpeed) || !s_isValid(m_vitesseEtablie))
                return false;

        return true;
}

bool RampeGenerique::s_isValid(float val)
{
        return (!std::isnan(val) && !std::isinf(val));
}

#ifdef ARDUINO
String RampeGenerique::getStringError(float time) const
{
        if (isWorking(time)) {
                return "no error";
        }
        if (m_etat != ETAT_RUNNING)	// si la rampe n'est pas initialisee
                return "erreur etat non demarre" + String(m_etat);
    if (time < m_startTime - 10.0)	// si retour dans le temps (overflow)
                return "retour dans le temps, depart dans le futur";
    if (!s_isValid(m_distFinAccelerationInitiale))
                return "m_distFinAcceleration non valide " + String(m_distFinAccelerationInitiale);
    if (!s_isValid(m_distDebutFreinage))
                return "m_distDebutFreinage non valide" + String(m_distDebutFreinage);
    if (!s_isValid(m_distEnd))
                return "m_distEnd non valide" + String(m_distEnd);
    if (!s_isValid(m_startTime))
                return "start_stime non valide" + String(m_startTime);
    if (!s_isValid(m_timeFinAccelerationInitiale))
                return "m_timeFinAcceleration non valide" + String(m_timeFinAccelerationInitiale);
    if (!s_isValid(m_timeDebutFreinage))
                return "m_timeDebutFreinage non valide" + String(m_timeDebutFreinage);
    if (!s_isValid(m_timeEnd))
                return "m_timeEnd non valide" + String(m_timeEnd);
    if (!s_isValid(m_startSpeed))
                return "start_speed non valide" + String(m_startSpeed);
    if (!s_isValid(m_vitesseEtablie))
                return "m_vitesseEtablie non valide" + String(m_vitesseEtablie);
        return "unkown";
}
#else
string RampeGenerique::getStringError(float time) const
{
    if (isWorking(time))
        return string("no error");
    if (m_etat != ETAT_RUNNING)	// si la rampe n'est pas initialisee
        return string("erreur etat non demarre") + to_string(m_etat);
    if (time < m_startTime -10.0)	// si retour dans le temps (overflow de micros())
        return string("retour dans le temps, depart dans le futur, temps actuel" + std::to_string(time) + "temps de depart " + std::to_string(m_startTime));
    if (!s_isValid(m_distFinAccelerationInitiale))
        return string("m_distFinAcceleration non valide ") + to_string(m_distFinAccelerationInitiale);
    if (!s_isValid(m_distDebutFreinage))
        return string("m_distDebutFreinage non valide") + to_string(m_distDebutFreinage);
    if (!s_isValid(m_distEnd))
        return string("m_distEnd non valide") + to_string(m_distEnd);
    if (!s_isValid(m_startTime))
        return string("start_stime non valide") + to_string(m_startTime);
    if (!s_isValid(m_timeFinAccelerationInitiale))
        return string("m_timeFinAcceleration non valide") + to_string(m_timeFinAccelerationInitiale);
    if (!s_isValid(m_timeDebutFreinage))
        return string("m_timeDebutFreinage non valide") + to_string(m_timeDebutFreinage);
    if (!s_isValid(m_timeEnd))
        return string("m_timeEnd non valide") + to_string(m_timeEnd);
    if (!s_isValid(m_startSpeed))
        return string("start_speed non valide") + to_string(m_startSpeed);
    if (!s_isValid(m_vitesseEtablie))
        return string("m_vitesseEtablie non valide") + to_string(m_vitesseEtablie);
    return string("unknown");
}
#endif
float RampeGenerique::calcDistChgmtVitesse(float speedStart, float speedEnd) const
{
    return s_calcDistChgmtVitesse(speedStart,speedEnd,m_acceleration,m_deccelerationSpeedShift);
}

float RampeGenerique::s_calcDistChgmtVitesse(float speedStart, float speedEnd,float acceleration,float decceleration)
{				// calcule la distance necessaire pour effectuer la variation de vitesse donnee
        if (speedStart < speedEnd)
        {	// si on doit effectuer une phase d'acceleration (!= du cas 2)
                return (sq(speedEnd) - sq(speedStart)) / (2 * acceleration);
        } else
        {			// cas 2, une seule deceleration
                return (sq(speedStart) - sq(speedEnd)) / (2 * decceleration);
        }
}
