/*
   ici on gere ROS_Serial et les topics

   de multiples données sont geres par ROS :
    en subscriber
    ->la consigne de déplacement
    -> LEs gains (moteurs, asserv position)
    -> les accélérations/Decelerations des rampes d'angle et de vitesse
    -> les 2 vitesses consignes
    en publisher :
    ->la position actuelle
    ->un ok pour l'arrivée à la consigne
    ->un topic de log total qui envoie toutes les données de l'asserv
    ->un topic de debug
*/

#ifndef __H_ROS
#define __H_ROS

#ifndef __SIMU__ // This ROS file is only used in embedded mode, not in simulation mode

#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>

// #include "SM.hpp"
#include <Events.hpp>
#include <Position2D.h>

enum LogType
{
    INFO = 0,
    WARN = 1,
    ERROR = 2,
    FATAL = 3,
    DEBUG = 4,
};

class ROS
{
public :
    ROS();
    void sendOkPos();
    void sendOkTurn();
    void errorAsserv(String details);
    void errorAsservNotSet(String details);
    void logPrint(LogType logtype, String msg);
    void publishFullLogs();
    void sendCurrentPosition(Position2D position);

    void sendDebug();

    void loop();

    // on confirme au haut niveau dans quel sens on avance
    // void confirmMarcheArriere();
    // void confirmMarcheAvant();

    //callback sur les subscriber ROS
    // de plus l'utilisation en callback impose de les déclarer statique
    static void s_goToCb(const geometry_msgs::Quaternion& positionMsg);
    static void s_debug(const std_msgs::Int16& debugMsg);

    static void s_changeGains(const std_msgs::Float32MultiArray& gains);
    static void s_changeGainsMotor(const std_msgs::Float32MultiArray& gainsM);
    static void s_setSpeed(const std_msgs::Float32MultiArray& speeds);
    static void s_changeAccDecRampe(const std_msgs::Float32MultiArray& gains);
    static void s_changeAccDecRampePrecise(const std_msgs::Float32MultiArray& gains);


    std_msgs::Int32MultiArray m_odosTicks;
    ros::Publisher m_odosTicksPub{ros::Publisher("odos_count", &m_odosTicks)};


private :
    ros::Subscriber<geometry_msgs::Quaternion>   m_subOrder {ros::Subscriber<geometry_msgs::Quaternion>  ("nextPositionTeensy", s_goToCb)};
    ros::Subscriber<std_msgs::Float32MultiArray> m_subGainsP{ros::Subscriber<std_msgs::Float32MultiArray>("gains", s_changeGains)};
    // ros::Subscriber<std_msgs::Float32MultiArray> m_subGainsM{ros::Subscriber<std_msgs::Float32MultiArray>("gainsMotor", s_changeGainsMotor)};
    // ros::Subscriber<std_msgs::Float32MultiArray> m_subSpeed {ros::Subscriber<std_msgs::Float32MultiArray>("speedTeensyObjective", s_setSpeed)}; // on fixe la vitesse de la rampe d'avance
    // ros::Subscriber<std_msgs::Float32MultiArray> m_subAcc   {ros::Subscriber<std_msgs::Float32MultiArray>("dynamicParameters", s_changeAccDecRampe)};
    // ros::Subscriber<std_msgs::Float32MultiArray> m_subAcc2  {ros::Subscriber<std_msgs::Float32MultiArray>("dynamicParameters2", s_changeAccDecRampePrecise)};



    ros::Subscriber<std_msgs::Int16> m_subDebug {ros::Subscriber<std_msgs::Int16>("debug/BR", s_debug)};

    geometry_msgs::Pose2D m_feedbackPosition;
    ros::Publisher m_positionFeedback{ros::Publisher("current_position", &m_feedbackPosition)};

    geometry_msgs::Quaternion m_debugVar;
    ros::Publisher m_debugPub{ros::Publisher("debugSend", &m_debugVar)};

    


    // std_msgs::Int16 m_feedbackOk;
    // ros::Publisher m_okFeedback{ros::Publisher("okPosition", &m_feedbackOk)};

    // std_msgs::Float32MultiArray m_logTotalArray; // envoi en array pour simplifier la lecture
    // ros::Publisher m_logTotale{ros::Publisher("logTotaleArray", &m_logTotalArray)};

    ros::NodeHandle m_nodeHandle;
};
#else
#include "../Simulation/ROSSimu.h"
#endif//ifndef __SIMU__
#endif//protect against multiple inclusions
