#ifndef DOOR_HPP
#define DOOR_HPP

#include "Servo.h"
#include "a_define.hpp"
#include <ros.h>
#include <std_msgs/Int16.h>

enum DoorsState{
    CLOSED = 0,
    LEFT_OPEN   = 1,
    RIGHT_OPEN = 2
};

class Doors{

    private:

        Servo m_left_servo;
        Servo m_right_servo;

        int m_left_positions[3];
        int m_right_positions[3];

        DoorsState m_state;

    public: 

        Doors();

        void setState(DoorsState);

        void setup();
        void loop();

};

class DoorsROS{

    private:

        static Doors* m_p_doors;
        static ros::NodeHandle* m_p_nh;

        ros::Subscriber<std_msgs::Int16> m_subLeft;
        ros::Subscriber<std_msgs::Int16> m_subRight;


    public:

        DoorsROS(Doors* m_p_doors, ros::NodeHandle* p_nh);

        static void subCallbackLeft(const std_msgs::Int16& stateVal);
        static void subCallbackRight(const std_msgs::Int16& stateVal);

        void setup();
        void loop();
        

};

#endif