//machine a etat gerant le statut de l'asserv
//inspiré des rapports 2004 robot


#include "a_Led_RGB.h"
#include "c_switchs.h"
#include "e_odos_motor.h"
#include "g_Asserv_Motors.h"
#include "i_etat_asserv.h"
#include "v_Logger.h"
#include "p_Odos.h"
#include "z_Setup_Loop.h"

constexpr AsservPositionTask::AsservObjectif MachineAEtatAsserv::OBJECTIFSTOP;

MachineAEtatAsserv::MachineAEtatAsserv()
{
    m_rampePosition.setDeccBrake(BRAKE_DECCELERATION);
}

void MachineAEtatAsserv::_transitionEtat(EtatAsserv next_etat)
{ // recoit l'etat vers lequel on va transitionner
  // pendant la transition on prépare les rampes, on ajuste le mode d'asserv et on met à jour la LED
  Timer::resetAllTimers();
  switch (next_etat)
  {
  case ETAT_ORIENTATION_INITIALE :
  case ETAT_ORIENTATION_FINALE :
  {
      float thetaObjective = 0.0,thetaStart = 0.0;
      if(m_goalOrder.goalType == TRANS && next_etat == ETAT_ORIENTATION_FINALE) // pas d'orientation a faire
      {
          m_goalOrder.goalPos.theta = _getThetaObjectiveInDistRampeAfterTransition(); // on set l'objectif comme l'objectif d'asserv actuel pour garder la consigne continue
          _transitionEtat(ETAT_ARRIVE);
          break;
      }
      if(next_etat == ETAT_ORIENTATION_INITIALE)
      {
          m_currentStateAsserv = ETAT_ORIENTATION_INITIALE;
          m_startPosBeforeRotation = m_lastGoalOrder.goalPos; // on stocke la position qu'on va essayer de conserver pendant la rotation
          thetaStart = m_lastGoalOrder.goalPos.theta;
          thetaObjective = _isCurrentOrderAskRecalage() ? m_goalOrder.goalPos.theta : _getAngleVersObjectifIncludingMarcheArriere(); // objectif a la fin de la rampe qui dépend du type de mission
      }
      else
      {
          m_currentStateAsserv = ETAT_ORIENTATION_FINALE; // on s'oriente
          thetaStart = _isOrderNeedInitialAndFinalRotation()? _getThetaObjectiveInDistRampeAfterTransition() : m_lastGoalOrder.goalPos.theta;
          thetaObjective = m_goalOrder.goalPos.theta;
      }
      asservPositionTask->setIStatusOnPID(true, true);
      m_rampeOrientation.setParcoursByShortestPath(thetaStart, thetaObjective,!_isOrderAskSmoothness());
      m_rampeOrientation.startParcours(timeFloat()); // on demarre la rampe
      break;
  }
  case ETAT_AVANCE_RAMPE :
      asservPositionTask->setIStatusOnPID(true, true);
      m_startPosAfterRotation = odosPositionTask->getRobotPosition();
      if (_isCurrentOrderAskRecalage() )//si on est en recalage il n'y a pas de rampe de distance a lancer
          _transitionEtat(ETAT_RECALAGE);
      else
      {

          m_rampePosition.setParcours(m_startPosBeforeRotation, m_goalOrder.goalPos,!_isOrderAskSmoothness());
          m_currentStateAsserv = ETAT_AVANCE_RAMPE;
          m_rampePosition.startParcours(timeFloat()); // on lance la rampe
      }
      break;

    case ETAT_ARRIVE:
      asservPositionTask->setIStatusOnPID(false, false);
      m_currentStateAsserv = ETAT_ARRIVE;
      break;

    case ETAT_RECALAGE :
      m_timerOutRecalageAvance.start(millis());
      m_timerRecalageAsservOn.start(millis());
      m_currentStateAsserv = ETAT_RECALAGE;
      m_rampeRecal.startParcours(timeFloat());; //on doit avancer en recalage
      m_startPosRecal = odosPositionTask->getRobotPosition();
      break;

    case ETAT_RECALAGE_TERMINE:
      m_currentStateAsserv = ETAT_RECALAGE_TERMINE;
      if (m_goalOrder.isRecalApplied)       // on applique le recalage si necessaire
          //if(_testSwitchCoherentWithRecalage()) // test des switchs au recalage a retester, désactivé pour l'instant
              odosPositionTask->setPositionAvecRecalage();
      m_timerOutRecalageStopWheel.start(millis());
      break;

  case ETAT_BLOQUE :
      m_currentStateAsserv = ETAT_BLOQUE;
      break;

  case ETAT_STOPPING :
      m_timerOutFreinage.start(millis());
      m_rampePosition.triggerBrake(timeFloat());
      m_currentStateAsserv = ETAT_STOPPING;
      break;

  case ETAT_STOP:
      m_currentStateAsserv = ETAT_STOP;
      break;

  case ETAT_ERREUR:
      m_currentStateAsserv = ETAT_ERREUR;
      break;
  default:
      break;
  }
  _updateLed(m_currentStateAsserv);
  Logger::setFieldValue(int(m_currentStateAsserv), Logger::currentStateIndex);
}

void MachineAEtatAsserv::departRobot(OrderCaracteristics order)
{
    m_goalOrder = order;
    // on démarre le robot en mode initial
    if (m_goalOrder.goalType == ORIENT)//  on passe direct à la dernière phase
        _transitionEtat(ETAT_ORIENTATION_FINALE);
    else
        _transitionEtat(ETAT_ORIENTATION_INITIALE);
}

void MachineAEtatAsserv::progressiveStop(OrderCaracteristics order)
{
    m_goalOrder.goalType = order.goalType;
    if(m_currentStateAsserv == ETAT_AVANCE_RAMPE) // si on est en train de rouler on déclenche un freinage
        _transitionEtat(ETAT_STOPPING);
    else // sinon on bloque les roues
        _transitionEtat(ETAT_STOP);
    // note : envoyer deux ETAT_STOP de suite va bloquer directement les roues
}

bool MachineAEtatAsserv::_calcTransitionEtat()
{ // retourne true si un etat a changé
    EtatAsserv lastEtatAsserv = m_currentStateAsserv;
    if(_manageTimeout()) // si il y a un timeout, il y a donc eu une transition
        return true;
    switch (m_currentStateAsserv)
    {
    case ETAT_STOP :
    case ETAT_ERREUR:
    case ETAT_BLOQUE:
    case ETAT_ARRIVE:
        break;//etats terminaux
    case ETAT_ORIENTATION_FINALE :  // etat orientation finale
    case ETAT_ORIENTATION_INITIALE : // on cherche à s'orienter vers l'objectif
        if (m_rampeOrientation.isFinished(timeFloat()))
        {
            if(m_currentStateAsserv == ETAT_ORIENTATION_INITIALE)
            {
                if (std::abs( modulo_pipi(m_rampeOrientation.getObjective() - odosPositionTask->getRobotPosition().theta)) < SEUIL_ORIENT )
                { // si l'erreur angulaire est assez faible
                    if (m_timerBufferTimeOrientation.startIfNotStartedAndTestExpiration(millis()))
                    { // on bascule sur la rampe d'avancement
                        _transitionEtat(ETAT_AVANCE_RAMPE);
                        rosTask->sendOkTurn();
                    }
                }
                else// on reset le timer pour le buffer de temps
                    m_timerBufferTimeOrientation.reset();
            }
            else  // orientation finale
                if (std::abs(_getPointProjectedInRobotReferential(m_goalOrder.goalPos).x) < SEUIL_DIST_ARRIVE &&
                    std::abs(modulo_pipi(m_goalOrder.goalPos.theta - odosPositionTask->getRobotPosition().theta)) < SEUIL_ANGLE_ARRIVE )
                { // on a fini
                    _transitionEtat(ETAT_ARRIVE);
                    rosTask->sendOkTurn();
                }
        }
        break;
    case ETAT_STOPPING :
    case ETAT_AVANCE_RAMPE : // on est en train d'avancer
        if (m_rampePosition.isFinished(timeFloat()))
        {
            if(m_currentStateAsserv == ETAT_STOPPING)
            {
                _transitionEtat(ETAT_STOP);
                break;
            }
            if (std::abs(_getPointProjectedInRobotReferential(m_goalOrder.goalPos).x) < SEUIL_DIST_FRONTAL && std::abs(_getPointProjectedInRobotReferential(m_goalOrder.goalPos).y) < SEUIL_DIST_LATERAL)
            { // si on est suffisament proche et qu'on a finit la rampe on va s'orienter
                if (m_timerBufferTimeAvance.startIfNotStartedAndTestExpiration(millis()))
                {
                    _transitionEtat(ETAT_ORIENTATION_FINALE);
                    rosTask->sendOkPos();
                }
            }
            else// la rampe est finie ,on utilise le timemin
                m_timerBufferTimeAvance.reset();
        }
        break;
    case ETAT_RECALAGE : // on regarde si le robot est au contact avec le mur
        if (Position2D::s_dist(m_startPosRecal, odosPositionTask->getRobotPosition()) > 3 && odosPositionTask->isRobotBlocked(SEUIL_BLOCAGE_MUR * (m_goalOrder.isRecalApplied ? 1.0 : 1.0)))
        { // on attend que le robot se soit déplacé d'au moins 3 mm, et qu'il est bloqué sur un mur
            if (m_timerRecalage.startIfNotStartedAndTestExpiration(millis()))
            { // on reste au contact pendant un minimum de temps
                _transitionEtat(ETAT_RECALAGE_TERMINE);
            }
        }
        else
            m_timerRecalage.reset();
        break;
    case ETAT_RECALAGE_TERMINE :
        if (
        #ifdef ASSERV_MOTEURS
                true
        #else
                false
        #endif
              || std::abs(odosMoteursTask->getSpeedL()) + std::abs(odosMoteursTask->getSpeedR()) < 1
                )
        {
            rosTask->sendOkPos();
            _transitionEtat(ETAT_BLOQUE);
        }
        break;
    default:
        break;
    }
    _testRampeStatus();
    return m_currentStateAsserv != lastEtatAsserv;
}

bool MachineAEtatAsserv::_manageTimeout()
{ // return true si un timeout s'est déclenché
    switch (m_currentStateAsserv)
    {
    case ETAT_STOP :
    case ETAT_ERREUR:
    case ETAT_BLOQUE:
    case ETAT_ARRIVE:
        break; // pas de timeout a gérer
    case ETAT_ORIENTATION_INITIALE :
    case ETAT_ORIENTATION_FINALE :
        if (m_rampeOrientation.isFinished(timeFloat()) && m_timerTimeOutOrientation.startIfNotStartedAndTestExpiration(millis()))
        {    // onlance une erreur
              error(m_currentStateAsserv == ETAT_ORIENTATION_INITIALE ?"timeout orientation initiale" : "timeout orientation finale");
              return true;
        }
        break;
    case ETAT_AVANCE_RAMPE :
        if(m_rampePosition.isFinished(timeFloat()) && m_timerTimeOutAvance.startIfNotStartedAndTestExpiration(millis()))
        {
          error("timeout deplacement lineaire");
          return true;
        }
        break;
    case ETAT_STOPPING :
        if (m_timerOutFreinage.isExpired(millis()))
        { // on lance une erreur
          _transitionEtat(ETAT_STOP);
          return true;
        }
        break;
    case ETAT_RECALAGE :
        if (m_timerOutRecalageAvance.isExpired(millis()))
        { // timeout recalage
          error("timeout deplacement recalage");
          return true;
        }
        break;
    case ETAT_RECALAGE_TERMINE:
        if (m_timerOutRecalageStopWheel.isExpired(millis()))
        { // timeout recalage
          error("timeout deplacement recalage");
          return true;
        }
        break;    
    }
    return false;
}

bool MachineAEtatAsserv::_testObjectivePointTooFar(AsservPositionTask::AsservObjectif objectiveToReturn)
{ // return true si une erreur est levee
    switch (m_currentStateAsserv)
    {
    case ETAT_STOP : //etats terminaux
    case ETAT_ERREUR:
    case ETAT_BLOQUE:
    case ETAT_RECALAGE_TERMINE: // le timeout suffit
        break; // pas d'erreur à gérer
    case ETAT_ORIENTATION_INITIALE :
    case ETAT_ORIENTATION_FINALE :
        if (std::abs(objectiveToReturn.erreurTourne) > PI / 3  || std::abs(objectiveToReturn.erreurAvance) > 150)
        { // si l'erreur angulaire est trop importante
          // ou si l'erreur en distance au point est trop importante (15 cm)
          if (m_timerTimeOutPositionErrorTooGreat.startIfNotStartedAndTestExpiration(millis()))
            error("erreur derive position theorique en rotation");
        }
        else
          m_timerTimeOutPositionErrorTooGreat.reset();
        break;
    case ETAT_AVANCE_RAMPE :
    case ETAT_STOPPING :
        if (std::abs(objectiveToReturn.erreurTourne) > PI / 4  || std::abs(objectiveToReturn.erreurAvance) > 300)
        { // si l'erreur angulaire est trop importante
          // ou si l'erreur en distance au point est trop importante (30 cm)
          if (m_timerTimeOutPositionErrorTooGreat.startIfNotStartedAndTestExpiration(millis()))
            error("erreur derive position theorique en avance");
        }
        else
          m_timerTimeOutPositionErrorTooGreat.reset();
    break;
    case ETAT_ARRIVE :
        if (Position2D::s_dist(odosPositionTask->getRobotPosition(), m_goalOrder.goalPos) > 100) // si l'erreur au point est trop importante. Le robot est quasi statique donc pas de temporisation
            error("erreur derive position finale");
        if ( std::abs(objectiveToReturn.erreurTourne) > PI / 8)
            error("erreur derive angle final");
        if( max(asservMoteurLeftTask->getAbsSpeedObjective(), asservMoteurRightTask->getAbsSpeedObjective())> 200 )
        {
            if (m_timerOutSpeedTooHighWhileArrived.startIfNotStartedAndTestExpiration(millis()))
              error("erreur position finale : vitesse de moteur trop elevee");
        }
        else
              m_timerOutSpeedTooHighWhileArrived.reset();
    break;
    case ETAT_RECALAGE :
        if (Position2D::s_dist(odosPositionTask->getRobotPosition(), m_goalOrder.goalPos) > SEUIL_ERREUR_DIST_RECAL) 
          error("erreur derive position en recalage");        
        if (std::abs(modulo_pipi(m_goalOrder.goalPos.theta - odosPositionTask->getRobotPosition().theta)) > (m_timerRecalageAsservOn.isExpired(millis())? SEUIL_ERREUR_ANGLE_RECAL: PI/8)) // si le robot est tourné de plus de 10 deg avec l'asserv en angle off ou de PI/8 avec l'asserv on
          error("erreur derive orientation en recalage ");
    break;
    }
    return m_currentStateAsserv == ETAT_ERREUR;
}

bool MachineAEtatAsserv::_testRampeStatus()
{
    if(m_currentStateAsserv == ETAT_AVANCE_RAMPE)
    {
      if (!m_rampePosition.isWorking(timeFloat()))
      { // si il y a une erreur interne (overflow, retour dans le temps...)
        error("erreur interne rampe distance " + m_rampePosition.getStringError(timeFloat())); // on passe a l'etat ERREUR
        return true;
      }
    }
    else if (m_currentStateAsserv == ETAT_ORIENTATION_INITIALE || m_currentStateAsserv == ETAT_ORIENTATION_FINALE)
    {
      if (!m_rampeOrientation.isWorking(timeFloat()))
      { // si il y a une erreur interne (overflow, retour dans le temps...)
        error("erreur interne rampe Angle" + m_rampeOrientation.getStringError(timeFloat())); // on passe a l'etat ERREUR
        return true;
      }
    }
    return false;
}

bool MachineAEtatAsserv::_testSwitchCoherentWithRecalage()
{
    if(m_goalOrder.goalType == RECALAGE_AVANT)
    {
        if(!switchesTask->isSwitchPressed(SwitchesTask::INDEX_AVD) ||!switchesTask->isSwitchPressed(SwitchesTask::INDEX_AVG))
        {
            error(String("robot bloque mais capteur avant ") + String(!switchesTask->isSwitchPressed(SwitchesTask::INDEX_AVG) ?String("gauche"):String("droite")) + String(" non enfonce"));
            return false;
        }
    }
    else
    {
        if(!switchesTask->isSwitchPressed(SwitchesTask::INDEX_ARD) ||!switchesTask->isSwitchPressed(SwitchesTask::INDEX_ARG))
        {
            error(String("robot bloque mais capteur arriere ") + String(!switchesTask->isSwitchPressed(SwitchesTask::INDEX_ARG) ?String("gauche"):String("droite")) + String(" non enfonce"));
            return false;
        }
    }
    return true;
}

Position2D MachineAEtatAsserv::_getPointProjectedInRobotReferential(Position2D const& pos) const
{
    Position2D positionInRobotReferential = pos;
    positionInRobotReferential.changeReferentiel(odosPositionTask->getRobotPosition());
    return positionInRobotReferential;
}

float MachineAEtatAsserv::_getAngleVersObjectifIncludingMarcheArriere() const
{
    if (Position2D::s_isStrictEgalityXY(m_goalOrder.goalPos,odosPositionTask->getRobotPosition()))
        return odosPositionTask->getRobotPosition().theta; // si le robot est pile sur l'objectif, la direction n'est pas définie. on prend donc l'angle actuel du robot
    else
    {
        float dirToFinalGoalPosition = Position2D::s_angleBetweenTwoPoints(odosPositionTask->getRobotPosition(),m_goalOrder.goalPos);
        if(_isCurrentOrderAskRampeDistInReverse())
            dirToFinalGoalPosition = modulo_pipi(dirToFinalGoalPosition + PI);
        return dirToFinalGoalPosition;
    }
}

float MachineAEtatAsserv::_getErreurAngleVersObjectifIncludingMarcheArriere() const
{
    if (_isCurrentOrderAskRecalage())
      return modulo_pipi(m_goalOrder.goalPos.theta - odosPositionTask->getRobotPosition().theta);
    else
      return modulo_pipi(_getAngleVersObjectifIncludingMarcheArriere() - odosPositionTask->getRobotPosition().theta);
}

float MachineAEtatAsserv::_getThetaObjectiveInDistRampeAfterTransition() const
{
    if (Position2D::s_isStrictEgalityXY(m_goalOrder.goalPos,odosPositionTask->getRobotPosition()))
      return odosPositionTask->getRobotPosition().theta; //le robot est pile sur la position objectif donc la projection n'a pas de sens, on indique le robot bien oriente
    float dirToFinalGoalPosition = Position2D::s_angleBetweenTwoPoints(odosPositionTask->getRobotPosition(),m_goalOrder.goalPos);
    float erreurLateraleRampe = sin( m_rampePosition.getThetaParcours() - dirToFinalGoalPosition) * Position2D::s_dist(odosPositionTask->getRobotPosition(), m_rampePosition.getObjective()); // erreur laterale % la rampe
    float thetaInfluenceOfLateralError = - constrain(erreurLateraleRampe * COEF_ERREUR_LATERALE, -0.2, 0.2);
    return modulo_pipi(m_rampePosition.getThetaParcours() + thetaInfluenceOfLateralError + (_isCurrentOrderAskRampeDistInReverse()? PI : 0.0)); // on bloque l'ecart induit par l'erreur laterale a 10°
}

AsservPositionTask::AsservObjectif MachineAEtatAsserv::updateStateAndComputeAsservObjective()
{
  AsservPositionTask::AsservObjectif objectiveToReturn;
  if (m_goalOrder.goalType == CONTROL)
  { // contrôle par la wiimote
    objectiveToReturn.type = AsservPositionTask::OBJECTIF_COMMANDE;
    objectiveToReturn.commandeLeft = int(m_goalOrder.goalPos.y);
    objectiveToReturn.commandeRight = int(m_goalOrder.goalPos.x);
    return objectiveToReturn;
  }
  if (m_goalOrder.goalType == CONTROL_SPEED)
  { // contrôle direct assrv moteur
    objectiveToReturn.type = AsservPositionTask::OBJECTIF_VITESSE;
    objectiveToReturn.speedLeft = int(m_goalOrder.goalPos.y);
    objectiveToReturn.speedRight = int(m_goalOrder.goalPos.x);
    return objectiveToReturn;
  }
  if (m_currentStateAsserv == ETAT_STOP || m_currentStateAsserv == ETAT_ERREUR)
  { // arrêt des moteurs, pas d'asserv en cours
    return OBJECTIFSTOP;
  }
  if (!asservPositionTask->areGainsSet())// on vérifie si une erreur s'est produite
  { // si les gains n'ont pas été reçus ou les accelerations/vitesse non recues / erronées
    rosTask->errorAsservNotSet("erreur gains non recus");
    error("erreur gains non recus"); // on passe a l'etat ERREUR
    return OBJECTIFSTOP;
  }

  // on calcule dans quel mode d'asserv on doit passer si nécéssaire
  _calcTransitionEtat();

  switch (m_currentStateAsserv)
  { //selon le mode actuel
    case ETAT_STOP :
      return OBJECTIFSTOP;
    case ETAT_ERREUR:
      return OBJECTIFSTOP;
    case ETAT_BLOQUE :
      // objectiveToReturn.type = AsservPositionTask::OBJECTIF_VITESSE;
      // objectiveToReturn.speedLeft = 0.0;
      // objectiveToReturn.speedRight = 0.0;
      objectiveToReturn.type = AsservPositionTask::OBJECTIF_COMMANDE;
      objectiveToReturn.commandeLeft = 0;
      objectiveToReturn.commandeRight = 0;
      break;
    case ETAT_ORIENTATION_FINALE:
    case ETAT_ORIENTATION_INITIALE :
    {
      Position2D posObjectiveForAvance = (m_currentStateAsserv == ETAT_ORIENTATION_INITIALE ? m_startPosBeforeRotation : m_goalOrder.goalPos);
      objectiveToReturn.type = AsservPositionTask::OBJECTIF_POSITION;
      objectiveToReturn.erreurTourne = modulo_pipi(m_rampeOrientation.calcOrientation(timeFloat()) - odosPositionTask->getRobotPosition().theta);
      objectiveToReturn.erreurAvance = _getPointProjectedInRobotReferential(posObjectiveForAvance).x;// erreur projetée par rapport au point initial
      Logger::setFieldValue(radToDeg(modulo_pipi(m_rampeOrientation.calcOrientation(timeFloat()))), Logger::rampeAngleTheta);
      if (m_rampeOrientation.getTravelLength() > 0.01)
        objectiveToReturn.feedForwardTourner += m_rampeOrientation.calcSpeed(timeFloat()) * (m_rampeOrientation.isRotationTrigo() ? 1 : -1); // on utilise la precommande et on signe selon le sens de rotation
      Logger::setFieldValue(radToDeg(m_rampeOrientation.calcSpeed(timeFloat())), Logger::rampeAngleSpeed);
    }
      break;
    case ETAT_STOPPING:
    case ETAT_AVANCE_RAMPE : // si avance en mode rampe
      {
        objectiveToReturn.type = AsservPositionTask::OBJECTIF_POSITION;
        Position2D posRampe = m_rampePosition.calcPos(timeFloat());
        Logger::setFieldValue(posRampe.x, Logger::rampePosX);
        Logger::setFieldValue(posRampe.y, Logger::rampePosY);
        if ( Position2D::s_isStrictEgalityXY(odosPositionTask->getRobotPosition(),posRampe))
        { // cas parfait, le robot est exactement sur le point de la rampe (situation possible à l'initialisation du robot)
          objectiveToReturn.erreurAvance = 0.;
          objectiveToReturn.erreurTourne = _getErreurAngleVersObjectifIncludingMarcheArriere();
        }
        else
        {
          float distPointDepart = Position2D::s_dist(odosPositionTask->getRobotPosition().x, m_startPosAfterRotation);
          objectiveToReturn.erreurAvance = _getPointProjectedInRobotReferential(posRampe).x;
          float thetaObjectiveAfterTransition = _getThetaObjectiveInDistRampeAfterTransition();
          if (distPointDepart > DIST_TRANSITION_ANGLE)
          {
            objectiveToReturn.erreurTourne = modulo_pi2pi2(thetaObjectiveAfterTransition - odosPositionTask->getRobotPosition().theta); // on prends entre - PI/2, PI/2
          }
          else
          {
            float thetaObjInTransition = m_rampeOrientation.getObjective() + modulo_pipi(thetaObjectiveAfterTransition - m_rampeOrientation.getObjective()) *  distPointDepart / DIST_TRANSITION_ANGLE;
            objectiveToReturn.erreurTourne = modulo_pi2pi2(thetaObjInTransition - odosPositionTask->getRobotPosition().theta); // on fait une transition entre le cap de fin de rampe de rotation et le cap précedemment choisi
          }
        }
        objectiveToReturn.feedForwardAvancer += m_rampePosition.calcSpeed(timeFloat()) * (_isCurrentOrderAskRampeDistInReverse() ? -1 : 1);
        Logger::setFieldValue(m_rampePosition.calcSpeed(timeFloat()), Logger::rampeDistSpeed);
      }
      break;
    case ETAT_ARRIVE :
      objectiveToReturn.type = AsservPositionTask::OBJECTIF_POSITION;
      objectiveToReturn.erreurTourne = modulo_pipi(m_goalOrder.goalPos.theta - odosPositionTask->getRobotPosition().theta); // on se fixe sur l'angle final
      objectiveToReturn.erreurAvance = _getPointProjectedInRobotReferential(m_goalOrder.goalPos).x;
      break;

    case ETAT_RECALAGE : // on fait avancer en ligne droite le robot
      {
        float avancer = 0.0;
        objectiveToReturn.type = AsservPositionTask::OBJECTIF_VITESSE;
        avancer = (m_goalOrder.goalType == RECALAGE_AVANT ? 1 : -1) * m_rampeRecal.calcSpeed(timeFloat());
        // pendant les x s du debut on active l'asserv en angle
        if (!m_timerRecalageAsservOn.isExpired(millis()))
        {
          float erreurAngleCmd = modulo_pipi(m_goalOrder.goalPos.theta - odosPositionTask->getRobotPosition().theta);
          float tourner = erreurAngleCmd * asservPositionTask->getKPAngleGain();

          objectiveToReturn.speedLeft = m_filterSpeedRecalL.computeOutput((AsservMoteursTask::PRECOMMANDE_AVANCE * avancer - AsservMoteursTask::PRECOMMANDE_ROTATION * tourner/3), micros());
          objectiveToReturn.speedRight = m_filterSpeedRecalR.computeOutput((AsservMoteursTask::PRECOMMANDE_AVANCE * avancer + AsservMoteursTask::PRECOMMANDE_ROTATION * tourner/3 ), micros());
        }
        else

        { // l'asserv en angle est desactivee
          // on fait varier les vitesses consignes selon les appuis sur les capteurs
          // un filtre est appliqué sur la commande moteur pour éviter les discontinuités
          if (odosPositionTask->isRobotBlocked(SEUIL_BLOCAGE_MUR * (m_goalOrder.isRecalApplied ? 1.0 : 1.0))){
            objectiveToReturn.speedRight = m_filterSpeedRecalR.computeOutput(AsservMoteursTask::PRECOMMANDE_AVANCE * avancer / 3, micros());
            objectiveToReturn.speedLeft = m_filterSpeedRecalL.computeOutput(AsservMoteursTask::PRECOMMANDE_AVANCE * avancer / 3, micros());
          }
          else {
            objectiveToReturn.speedRight = m_filterSpeedRecalR.computeOutput(AsservMoteursTask::PRECOMMANDE_AVANCE * avancer, micros());
              objectiveToReturn.speedLeft = m_filterSpeedRecalL.computeOutput(AsservMoteursTask::PRECOMMANDE_AVANCE * avancer, micros());
          }
        }
      }
      break;
    case ETAT_RECALAGE_TERMINE :
      objectiveToReturn.type = AsservPositionTask::OBJECTIF_VITESSE;
      // objectiveToReturn.speedLeft = m_filterSpeedRecalL.computeOutput(0.0, micros());
      // objectiveToReturn.speedRight = m_filterSpeedRecalR.computeOutput(0.0, micros());
      objectiveToReturn.speedLeft = 0.0;
      objectiveToReturn.speedRight = 0.0;
      break;
    default : // cas non prévu on arrete le robot
      error("erreur switch de k_Asserv : etat non prevu dans la machine a etat" + String(m_currentStateAsserv));
      return OBJECTIFSTOP;
  }
  if(_testObjectivePointTooFar(objectiveToReturn))
      return OBJECTIFSTOP;
  return objectiveToReturn;
}

void MachineAEtatAsserv::setTypeOfTrajectory(bool isFast)
{
    if (isFast)
    {
      m_timerBufferTimeOrientation.setLength(TIMEMIN_ORIENT);
      m_timerBufferTimeAvance.setLength(TIMEMIN_AVANCE);
    }
    else
    {
      m_timerBufferTimeOrientation.setLength(TIMEMIN_ORIENT * 4);
      m_timerBufferTimeAvance.setLength(TIMEMIN_AVANCE * 4);
    }
}

bool MachineAEtatAsserv::isAsservWorking() const
{
  return m_currentStateAsserv != ETAT_STOP && m_currentStateAsserv != ETAT_ERREUR ;
}


bool MachineAEtatAsserv::isWheelAllowedToDrift() const
{
  return m_currentStateAsserv == ETAT_ERREUR || m_currentStateAsserv == ETAT_STOP || m_currentStateAsserv == ETAT_RECALAGE || m_currentStateAsserv == ETAT_RECALAGE_TERMINE;
}

bool MachineAEtatAsserv::isRobotArrived() const
{
  return m_currentStateAsserv == ETAT_ARRIVE;
}

void MachineAEtatAsserv::error(String details)
{
    _transitionEtat(ETAT_ERREUR);
    rosTask->errorAsserv(details);
}

void MachineAEtatAsserv::manageNewOrder(Position2D const& posOrder, MachineAEtatAsserv::GoalType type)
{
    MachineAEtatAsserv::OrderCaracteristics orderReceived;
    orderReceived.goalType = type;
    switch (type)
    {
      case RESET :
        odosPositionTask->setPosition(posOrder);
        immediateStop(orderReceived);
        break;

      case CONTROL_SPEED:
      case CONTROL :
        orderReceived.goalPos = posOrder;
        orderReceived.goalPos.theta = 0; // theta is ignored
        m_goalOrder = orderReceived;
        break;

      case STOP :
        progressiveStop(orderReceived);
        break;

      default : // deplacement du robot : point passage, final, recalages ou points light
        if (isRobotArrived())
        {
            if(Position2D::s_isStrictEgalityXY(posOrder,m_goalOrder.goalPos) && type != ORIENT) // si point merdique (aka identique au point précédent)
            {
                error("reception d'un point objectif identique à l'objectif precedent. L'ordre est rejete"); // on dit fuck au Haut Niveau
                return;
            }
         // on stocke le dernier objectif en angle pour la continuité de la consigne
          m_lastGoalOrder = m_goalOrder;
        }
        else
        { // le robot n'est pas arrivé donc soit arrêté soit en mouvement donc OSEF de la continuité de la consigne (aka c'est pas de notre faute -_(''/)_- )
          if(isAsservWorking())
          { // le robot est en train de rouler, on lance un warning
              rosTask->logPrint(String("Warning : changement de consigne alors que le robot n'est pas arrive "));
          }
          if(Position2D::s_isStrictEgalityXY(posOrder,odosPositionTask->getRobotPosition()) && type != ORIENT) // si point merdique (aka identique au point précédent)
          {
              error("Warning : reception d'un point objectif identique à la position actuelle l'ordre est rejete");
              return;
          }
          m_lastGoalOrder.goalPos = odosPositionTask->getRobotPosition();
          m_lastGoalOrder.goalType = STOP;
        }
        orderReceived.goalPos = posOrder;
        orderReceived.goalType = orderReceived.goalType; // defaults value, can be changed in following switch
        switch (orderReceived.goalType)
        {
          case RECALAGE_AVANT :
          case RECALAGE_ARRIERE :
            setTypeOfTrajectory(false);
            orderReceived.isRecalApplied = true;
            break;
          case FAKE_RECAL_AVANT:
            setTypeOfTrajectory(false);
            orderReceived.isRecalApplied = false;
            orderReceived.goalType = RECALAGE_AVANT;
            break;
          case FAKE_RECAL_ARRIERE:
            setTypeOfTrajectory(false);
            orderReceived.isRecalApplied = false;
            orderReceived.goalType = RECALAGE_ARRIERE;
            break;
          case LIGHT_FINAL_AVANT:
            setTypeOfTrajectory(false);
            rosTask->confirmMarcheAvant();
            break;
        case LIGHT_FINAL_ARRIERE:
            setTypeOfTrajectory(false);
            rosTask->confirmMarcheArriere();
            break;
        case ORIENT :
            setTypeOfTrajectory(false); // on passe en mode lent
            orderReceived.goalPos     =  m_lastGoalOrder.goalPos;
            orderReceived.goalPos.theta  =  posOrder.theta; // only keep theta order
            break;
        default:
            setTypeOfTrajectory(true); // rampes rapides
            break;
        }
        departRobot(orderReceived);
        break;
    }
}

MachineAEtatAsserv::OrderCaracteristics MachineAEtatAsserv::getGoalOrder() const
{
    return m_goalOrder;
}

String MachineAEtatAsserv::s_orderToString(MachineAEtatAsserv::GoalType obj)
{
    switch (obj)
    {
    case MachineAEtatAsserv::FINAL :             // point final, avec orientation
        return String("0 Objectif Final");
    case MachineAEtatAsserv::TRANS :             // point transitoire, sans orientation finale
        return String("1 Objectif Transitoire");
    case MachineAEtatAsserv::STOP :
        return String("2 Stop");// arret "d'urgence"
    case MachineAEtatAsserv::RESET :             // reset de la position odometrique
        return String("3 Reset Position");
    case MachineAEtatAsserv::CONTROL:
        return String("4 Manette Control");// controle par manette
    case MachineAEtatAsserv::RECALAGE_AVANT :
        return String("5 Recalage Avant");// recalage en marche avant
    case MachineAEtatAsserv::RECALAGE_ARRIERE :
        return String("6 Recalage Arriere");// recalage en marche arriere
    case MachineAEtatAsserv::LIGHT_FINAL_AVANT :
        return String("7 Deplacement Light Marche Avant");// point final marche avant (smooth)
    case MachineAEtatAsserv::LIGHT_FINAL_ARRIERE :
        return String("8 Deplacement Light Marche Arriere");// point final marche arriere (smooth)
    case MachineAEtatAsserv::ORIENT :
        return String(" 9 Orientation seulement");             //orientation seule sur place
    case MachineAEtatAsserv::FAKE_RECAL_AVANT :
        return String("10 Plaquage Avant"); // faux recalage en marche avant (plaquage au mur uniquement)
    case MachineAEtatAsserv::FAKE_RECAL_ARRIERE :
        return String("11 Plaquage Arriere");
    case MachineAEtatAsserv::CONTROL_SPEED :
        return String("12 Control Speed");
    case MachineAEtatAsserv::UNVALID_GOALTYPE :
        return String(int(obj)) + String("Unvalid GoalType");// sert a rejeter les valeurs non conformes:
    }
    return String(int(obj)) + String("Unvalid GoalType");
}

void MachineAEtatAsserv::_updateLed(MachineAEtatAsserv::EtatAsserv etat)
{
    switch (etat) {
      case MachineAEtatAsserv::ETAT_ORIENTATION_INITIALE:
        m_led.color(255, 0, 255); // magenta
        break;
      case MachineAEtatAsserv::ETAT_AVANCE_RAMPE:
        m_led.color(0, 0, 255);  //bleu
        break;
      case MachineAEtatAsserv::ETAT_STOP :
        m_led.color(255, 0, 0); //rouge
        break;
      case MachineAEtatAsserv::ETAT_ORIENTATION_FINALE :
        m_led.color(0, 255, 255);  // cyan
        break;
      case MachineAEtatAsserv::ETAT_ARRIVE :
        m_led.color(0, 255, 0);  //vert
        break;
      case MachineAEtatAsserv::ETAT_RECALAGE_TERMINE:
        m_led.color(255, 255, 255); // blanc hardcore
        break;
      case MachineAEtatAsserv::ETAT_BLOQUE :
        m_led.color(255, 127, 0); //orange
        break;
      case MachineAEtatAsserv::ETAT_ERREUR :
        m_led.color(0, 0, 0); //rien
        break;
      case MachineAEtatAsserv::ETAT_STOPPING :
        m_led.color(255, 255, 0); //jaune
        break;
      case MachineAEtatAsserv::ETAT_RECALAGE :
        m_led.color(127, 127, 127); // blanc léger
        break;
      default :
        break;
    }
}
