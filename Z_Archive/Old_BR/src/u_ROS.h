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
#include "src/Task/Task.h"
#include "src/Position2D/Position2D.h"

#include "ros.h"
#include <std_msgs/Int16.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

class ROSTask : public Task
{
public :
    ROSTask(TaskType type);
    void sendOkPos();
    void sendOkTurn();
    void errorAsserv(String details);
    void errorAsservNotSet(String details);
    void logPrint(String msg);
    void publishFullLogs();
    void sendCurrentPosition();
    void confirmMarcheArriere(); // on confirme au haut niveau dans quel sens on avance
    void confirmMarcheAvant();

    //callback sur les subscriber ROS
    // de plus l'utilisation en callback impose de les déclarer statique
    static void s_goToCb(const geometry_msgs::Quaternion& positionMsg);
    static void s_changeGainsPosition(const std_msgs::Float32MultiArray& gains);
    static void s_changeGainsMotor(const std_msgs::Float32MultiArray& gainsM);
    static void s_setSpeed(const std_msgs::Float32MultiArray& speeds);
    static void s_changeAccDecRampe(const std_msgs::Float32MultiArray& gains);
    static void s_changeAccDecRampePrecise(const std_msgs::Float32MultiArray& gains);
private :
    void _loop() override;


    ros::Subscriber<geometry_msgs::Quaternion>   m_subOrder {ros::Subscriber<geometry_msgs::Quaternion>  ("nextPositionTeensy", s_goToCb)};
    ros::Subscriber<std_msgs::Float32MultiArray> m_subGainsP{ros::Subscriber<std_msgs::Float32MultiArray>("gains", s_changeGainsPosition)};
    ros::Subscriber<std_msgs::Float32MultiArray> m_subGainsM{ros::Subscriber<std_msgs::Float32MultiArray>("gainsMotor", s_changeGainsMotor)};
    ros::Subscriber<std_msgs::Float32MultiArray> m_subSpeed {ros::Subscriber<std_msgs::Float32MultiArray>("speedTeensyObjective", s_setSpeed)}; // on fixe la vitesse de la rampe d'avance
    ros::Subscriber<std_msgs::Float32MultiArray> m_subAcc   {ros::Subscriber<std_msgs::Float32MultiArray>("dynamicParameters", s_changeAccDecRampe)};
    ros::Subscriber<std_msgs::Float32MultiArray> m_subAcc2  {ros::Subscriber<std_msgs::Float32MultiArray>("dynamicParameters2", s_changeAccDecRampePrecise)};

    geometry_msgs::Pose2D m_feedbackPosition;
    ros::Publisher m_positionFeedback{ros::Publisher("current_position", &m_feedbackPosition)};

    std_msgs::Int16 m_feedbackOk;
    ros::Publisher m_okFeedback{ros::Publisher("okPosition", &m_feedbackOk)};

    std_msgs::Float32MultiArray m_logTotalArray; // envoi en array pour simplifier la lecture
    ros::Publisher m_logTotale{ros::Publisher("logTotaleArray", &m_logTotalArray)};

    ros::NodeHandle m_nodeHandle;
};
#else
#include "../Simulation/ROSSimu.h"
#endif//ifndef __SIMU__
#endif//protect against multiple inclusions
