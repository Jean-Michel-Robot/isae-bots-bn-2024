/*
Allows for the creation of TentacleRos objects which manages the ros communication for the 6 tentacles
*/

#ifndef TENTACLEROS_H
#define TENTACLEROS_H

#include <Arduino.h>
#include "TentacleStateMachine.h"

#include <ros.h>
#include <std_msgs/Int16.h>

#include "Timer.h"


enum TentacleId {
    FR = 0,
    MR = 1,
    BR = 2,

    FL = 3,
    ML = 4,
    BL = 5
};

enum TentacleOrderId {
    ORDER_IDLE = 0,
    ORDER_GRAB = 1,
    ORDER_RELEASE = 2,
    ORDER_CARRY = 3,
    ORDER_CARRY_HIGH = 4,
    ORDER_END = 5
};

enum TentacleReleaseMsg {
    RELEASE_FAIL = 0,
    RELEASE_BAD = 1,
    RELEASE_GOOD = 2
};

class TentacleRos {
private:
    ros::NodeHandle *m_nh;

    Timer timerPubStates;

public:
    static int nbTentacles;
    static TentacleStateMachine **tentacleStateMachineArray;

    ros::Subscriber<std_msgs::Int16> subTentacleOrders;

    std_msgs::Int16 tentacleState;
    ros::Publisher pubTentacleStates;

    std_msgs::Int16 tentacleReleaseFb;
    ros::Publisher pubTentacleReleaseFbs;
    
    TentacleRos(ros::NodeHandle *nh, int nbTentacles, TentacleStateMachine **tentacleStateMachineArray, float timePubStates);

    static void tentacleOrdersCb(const std_msgs::Int16 &orderMsg);

    void publishTentacleState(int id);
    void publishReleaseFb(int id, int isReleaseSuccessful);

    void setup();

    void loop();

};

#endif