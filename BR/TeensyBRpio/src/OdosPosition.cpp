#include "OdosPosition.hpp"

#include "defines.hpp"

#include "ROS.hpp"
#include "main_loop.hpp"

#define METHOD 1 // choix de la méthode d'approximation

#include <cmath>

#ifdef __SIMU__
#include "../Simulation/EncoderSimu.h"
#define ODO_HARD // we simulate hard decode odos
#elif defined (ODO_HARD) // utilisation des decodeurs hardware de la teensy
#include <QuadDecode.h>
#elif defined(ODO_SOFT) // utilisation d'un decodage software par interruption
#include "src/Encoder/Encoder.h" //https://www.pjrc.com/teensy/td_libs_Encoder.html
Encoder knobLeft(32, 25);
Encoder knobRight(3, 4); //Encoder knobRight(32, 25);
// oui les variables sont en global, mais c'est plus simple pour gérer le define
#else
#error Odemeters method undefined
#endif

OdosPosition::OdosPosition()
{
#ifdef ODO_HARD
  QuadDecode.resetCounter1();
  QuadDecode.resetCounter2();
#elif defined(ODO_SOFT)
  knobLeft.write(0);
  knobRight.write(0);
#endif

  m_odoLeftCount  = 0;
  m_odoRightCount = 0;

  m_robotPosition.x       =   0;
  m_robotPosition.y       =   0;
  m_positionThetaOffset   =   0;
}

void OdosPosition::setPosition(Position2D pos)
{
  m_robotPosition = pos;
  m_positionThetaOffset += pos.theta - m_positionThetaOdo;
  m_positionThetaOdo = (double(m_odoRightCount) * L_R_ODOS - double(m_odoLeftCount)) / ECARTS_ODOS + m_positionThetaOffset;
}

void OdosPosition::loop()
{
  int32_t deltaL, deltaR;

  deltaL = - m_odoLeftCount;
  deltaR = - m_odoRightCount;

#ifdef ODO_HARD
  m_odoLeftCount = - QuadDecode.getCounter2();
  m_odoRightCount = QuadDecode.getCounter1();
#elif defined(ODO_SOFT)
  odoL = - knobLeft.read();
  odoR = knobRight.read();
#endif

// Logger::setFieldValue(m_odoLeftCount, Logger::odoL);
// Logger::setFieldValue(m_odoRightCount, Logger::odoR);

  deltaL += m_odoLeftCount;
  deltaR += m_odoRightCount;

  /*
     Le '> 0' est à modifier ?
     Pour trouver un juste milieu entre :
     update regulierement (pour limiter les erreurs de approx. geometrique de petits deplacements)
     et avoir des nombres "grand" (et limiter les erreurs de calculs de mantis...),

     //suite a calculs, sur les dimensions de la table l'erreur de mantis est faible (10e-9mm par ticks au max)
     // coucou Etienne comment va-tu :p
  */
  if (abs(deltaL) + abs(deltaR) > 0) { // si un tick est repéré sur un odomètre
    double old_positionTheta = m_positionThetaOdo;
    m_positionThetaOdo = (double(m_odoRightCount) * L_R_ODOS - double(m_odoLeftCount)) / ECARTS_ODOS + m_positionThetaOffset;

    double R = (double(deltaR) * L_R_ODOS + double(deltaL)) / 2. / UNITS_ODOS;

    //Dans le repere local du roobot (x etant devant)
    double dx = R;
    double dy = R;

#if METHOD == 1 // le robot s'est déplacé tout droit selon le précédent théta
    dx *= 1.0;
    dy *= 0.0;
#elif METHOD == 2 // le robot s'est déplacé tout droit selon le nouveau theta
    double deltaTheta = (double(deltaR) * L_R_ODOS - double(deltaL)) / ECART_ODOS;
    dx *= cos(deltaTheta);
    dy *= sin(deltaTheta);
#elif METHOD == 3 // demande à Etienne Arlaud
    double deltaTheta = (deltaR * L_R_ODOS - deltaL) / ECART_ODOS;
    if (deltaTheta != 0) {
      dx *= sin(deltaTheta) / deltaTheta;
      dy *= (1 - cos(deltaTheta)) / deltaTheta;
    } else {
      dx *= 1;
      dy *= 0;
    }
#else
#error method undefined
#endif
    //Changement de base dans le repere global
    m_robotPosition.x += cos(old_positionTheta) * dx + sin(old_positionTheta) * dy;
    m_robotPosition.y += sin(old_positionTheta) * dx - cos(old_positionTheta) * dy;
  }

  static FilterLowPass fil(1e-2);
  m_robotPosition.theta = fil.computeOutput(m_positionThetaOdo, micros());

  // Logger::setFieldValue(m_robotPosition.x, Logger::positionX);
  // Logger::setFieldValue(m_robotPosition.y, Logger::positionY);
  // Logger::setFieldValue(radToDeg(modulo_pipi(m_robotPosition.theta)), Logger::positionTheta);
  
  if (m_timer_last_send - millis() > ODO_SEND_POSITION_TIMER) {
    m_timer_last_send = millis();
    this->sendRobotPosition();

    char msg[50];
    sprintf(msg, "Odos Counts L: %li R: %li", m_odoLeftCount, m_odoRightCount);
    p_ros->logPrint(LogType::DEBUG, msg);
  }

}


bool OdosPosition::isRobotBlocked(float seuil)
{ // si les odometres detectent une vitesse nulle
  // m_speedOdometerL = m_filterSpeedOdoL.computeOutput(std::abs(float(m_odoLeftCount - m_oldOdoL)) / (micros() - m_microsOfLastMesureSpeedOdometers) * 1e6, micros());
  // m_speedOdometerR = m_filterSpeedOdoR.computeOutput(std::abs(float(m_odoRightCount - m_oldOdoR)) / (micros() - m_microsOfLastMesureSpeedOdometers) * 1e6, micros());

  // m_oldOdoL = m_odoLeftCount;
  // m_oldOdoR = m_odoRightCount;
  // m_microsOfLastMesureSpeedOdometers = micros();

  // return (m_speedOdometerL + m_speedOdometerR) < seuil ;

  return false;
}

#define THETA_RECAL_EPS PI/4
void OdosPosition::setPositionAvecRecalage()
{ // le robot a recu un ordre de recalage, et est arrive au contact du mur
  // if (Position2D::s_dist(m_robotPosition, machineAEtatAsservInstance->getGoalOrder().goalPos) > 50)
  // { // plus de 5 cm de decalage
  //   rosTask->logPrint("erreur >50mm");
  //   return;
  // }
  // rosTask->logPrint("recalage des positions : " );

  // // on distingue les 2 directions positions possibles
  // if (std::abs(modulo_pipi(machineAEtatAsservInstance->getGoalOrder().goalPos.theta - 0)) < THETA_RECAL_EPS ||
  //     std::abs(modulo_pipi(machineAEtatAsservInstance->getGoalOrder().goalPos.theta - PI)) < THETA_RECAL_EPS) { // si on est dans la direction theta=0 ou +- PI
  //   rosTask->logPrint("recalage x de " + String(m_robotPosition.x) + " a " + String(machineAEtatAsservInstance->getGoalOrder().goalPos.x) + " et theta de " + String(m_robotPosition.theta, 6) + " a " + String(machineAEtatAsservInstance->getGoalOrder().goalPos.theta, 6) );
  //   setPosition(Position2D(machineAEtatAsservInstance->getGoalOrder().goalPos.x, m_robotPosition.y, machineAEtatAsservInstance->getGoalOrder().goalPos.theta));
  // }
  // else if (std::abs(modulo_pipi(machineAEtatAsservInstance->getGoalOrder().goalPos.theta - PI/2)) < THETA_RECAL_EPS ||
  //          std::abs(modulo_pipi(machineAEtatAsservInstance->getGoalOrder().goalPos.theta + PI/2)) < THETA_RECAL_EPS) { // direction theta = +-PI/2
  //   rosTask->logPrint("recalage y de " + String(m_robotPosition.y) + " a " + String(machineAEtatAsservInstance->getGoalOrder().goalPos.y) + " et theta de " + String(m_robotPosition.theta, 6) + " a " + String(machineAEtatAsservInstance->getGoalOrder().goalPos.theta, 6));
  //   setPosition(Position2D(m_robotPosition.x, machineAEtatAsservInstance->getGoalOrder().goalPos.y, machineAEtatAsservInstance->getGoalOrder().goalPos.theta));
  // }
  // else
  //   rosTask->logPrint(String ("recalage des positions echoue: l'angle ne correspond pas a une direction cardinal") + String(modulo_pipi(machineAEtatAsservInstance->getGoalOrder().goalPos.theta)));
  return;
}

Position2D OdosPosition::getRobotPosition() const
{
  return m_robotPosition;
}

void OdosPosition::sendRobotPosition(){
  p_ros->sendCurrentPosition(this->getRobotPosition());
}


// PositionSenderTask::PositionSenderTask(TaskType type):Task(type)
// {
//  // constructeur vide, rien a initialiser, seul le constructeur de la classe mere est utilisé
// }

// void PositionSenderTask::_loop()
// {
//     rosTask->sendCurrentPosition();
// }