#ifndef ISBMOVINGROS_H
#define ISBMOVINGROS_H

#include <Arduino.h>
#include "IsbMovingHandler.h"

#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Quaternion.h>


class IsbMovingRos {
private:
    ros::NodeHandle *m_nh;

public:
    static IsbMovingHandler *m_isbMovingHandler;
    
    ros::Subscriber<std_msgs::Int16> subIsbOkPosition;

    ros::Subscriber<geometry_msgs::Quaternion> subIsbNextPosition;

    geometry_msgs::Quaternion isbMovingOrder;
    ros::Publisher pubIsbMovingOrder;

    IsbMovingRos(ros::NodeHandle *nh, IsbMovingHandler *isbMovingHandler);

    static void isbNextPositionCb(const geometry_msgs::Quaternion &nextPosMsg);

    static void isbOkPositionCb(const std_msgs::Int16 &okPosMsg);

    void publishIsbMovingOrder(float x, float y, float theta, int orderId);

    void setup();

    void loop();

};

#endif