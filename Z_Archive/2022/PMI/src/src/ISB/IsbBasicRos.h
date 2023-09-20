#ifndef ISBBASICROS_H
#define ISBBASICROS_H

#include <Arduino.h>
#include "IsbBasicHandler.h"

#include <ros.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>


class IsbBasicRos {
private:
    ros::NodeHandle *m_nh;

public:
    static IsbBasicHandler *m_isbBasicHandler;

    ros::Subscriber<geometry_msgs::Quaternion> subIsbSetSinglePixel;

    ros::Subscriber<std_msgs::Empty> subIsbReset;

    std_msgs::Int16 isbMatchState;
    ros::Publisher pubIsbMatchState;

    std_msgs::Int16 isbSide;
    ros::Publisher pubIsbSide;

    std_msgs::Int16 isbStrat;
    ros::Publisher pubIsbStrat;
    
    IsbBasicRos(ros::NodeHandle *nh, IsbBasicHandler *isbBasicHandler);

    static void isbSetSinglePixelCb(const geometry_msgs::Quaternion &pixelMsg);

    static void isbResetCb(const std_msgs::Empty &resetMsg);

    void publishIsbMatchStarted();
    void publishIsbSide(uint16_t side);
    void publishIsbStrat(uint16_t strat);

    void setup();

    void loop();

};

#endif