#ifndef _ROS_WRAP_H
#define _ROS_WRAP_H

#include "geometry/Position2D.h"
#include "ros/Logger.hpp"
#include "state_machine/Callback.hpp"
#include "logging.h"

#ifdef ARDUINO
#include "ROS_arduino.hpp"
#else
#include "ROS_linux.hpp"
#endif

class ROS: public Node_t
{

public:
    void sendCallback(AsservCallback callback);
    void sendCurrentPosition(Position2D<Millimeter> position);
    void sendOdosCounts(int32_t left, int32_t right);
    void publishFullLogs();

    void sendDebug();

    void logPrint(LogType logtype, string_t msg);
    void loop();

    // callback sur les subscriber ROS
    static void s_goToCb(const geometry_msgs::Quaternion *positionMsg);
    static void s_debug(const std_msgs::Int16 *debugMsg);
    static void s_setSpeed(const std_msgs::Int16 *speedMsg);
    static void s_changeGains(const std_msgs::Float32MultiArray *gains);
    static void s_idle(const std_msgs::Int16 *msg);

    static ROS &instance();

protected:
    ROS(): Node_t("base_roulante") {}
    void spin();

    /* SUBSCRIBERS */

    Subscriber_t<geometry_msgs::Quaternion> m_subOrder = create_subscription<geometry_msgs::Quaternion>("nextPositionTeensy", s_goToCb);
    Subscriber_t<std_msgs::Float32MultiArray> m_subGainsP = create_subscription<std_msgs::Float32MultiArray>("gains", s_changeGains);
    Subscriber_t<std_msgs::Int16> m_subIdle = create_subscription<std_msgs::Int16>("br/idle", s_idle);

    // TODO change dat stupid topic name (et faire une topic map au passage)
    Subscriber_t<std_msgs::Int16> m_subSpeed = create_subscription<std_msgs::Int16>("teensy/obstacle_seen", s_setSpeed);
    Subscriber_t<std_msgs::Int16> m_subDebug = create_subscription<std_msgs::Int16>("debug/BR", s_debug);

    /* PUBLISHERS */

    geometry_msgs::Pose2D m_feedbackPosition;
    Publisher_t<geometry_msgs::Pose2D> m_positionFeedback = create_publisher<geometry_msgs::Pose2D>("current_position");

    std_msgs::Int16 m_callbackHN;
    Publisher_t<std_msgs::Int16> m_pubHN = create_publisher<std_msgs::Int16>("okPosition");

    std_msgs::Float32MultiArray m_logTotalArray; // envoi en array pour simplifier la lecture
    Publisher_t<std_msgs::Float32MultiArray> m_logTotale = create_publisher<std_msgs::Float32MultiArray>("logTotaleArray");

    std_msgs::Int32MultiArray m_odosTicks;
    Publisher_t<std_msgs::Int32MultiArray> m_odosTicksPub = create_publisher<std_msgs::Int32MultiArray>("odos_count");

    geometry_msgs::Quaternion m_debugVar;
    Publisher_t<geometry_msgs::Quaternion> m_debugPub = create_publisher<geometry_msgs::Quaternion>("debugSend");

};

#endif