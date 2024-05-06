#ifndef CLAMP_HPP
#define CLAMP_HPP

#include "Servo.h"
#include "a_define.hpp"
#include <ros.h>
#include <std_msgs/Int16.h>

enum class ClampState : unsigned int{
    CLOSED = 0,
    OPEN = 1
};

class Clamp{

    private:

        Servo m_servo;

        int m_positions[2];

        ClampState m_state;

    public: 

        Clamp();

        void setState(ClampState state);

        void setup();
        void loop();

};

class ClampROS{

    private:

        static Clamp* m_p_clamp;
        static ros::NodeHandle* m_p_nh;

        ros::Subscriber<std_msgs::Int16> m_sub;


    public:

        ClampROS(Clamp* m_p_clamp, ros::NodeHandle* p_nh);

        static void subCallback(const std_msgs::Int16& stateVal);

        void setup();
        void loop();
        
};

#endif