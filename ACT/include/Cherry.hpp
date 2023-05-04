#ifndef CHERRY_HPP
#define CHERRY_HPP

#include "Servo.h"
#include "a_define.hpp"
#include <ros.h>
#include <std_msgs/Int16.h>

enum class CherryState{
    UP = 0,
    DOWN = 1
};

class Cherry{

    private:

        Servo m_servo;

        int m_positions[2];

        CherryState m_state;

    public: 

        Cherry();

        void setState(CherryState);

        void setup();
        void loop();

};

class CherryROS{

    private:

        static Cherry* m_p_cherry;
        static ros::NodeHandle* m_p_nh;

        ros::Subscriber<std_msgs::Int16> m_sub;


    public:

        CherryROS(Cherry* m_p_cherry, ros::NodeHandle* p_nh);

        static void subCallback(const std_msgs::Int16& stateVal);

        void setup();
        void loop();
        
};

#endif