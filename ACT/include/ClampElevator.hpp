#ifndef CLAMPELEVATOR_HPP
#define CLAMPELEVATOR_HPP

#include "Servo.h"
#include "a_define.hpp"
#include <ros.h>
#include <std_msgs/Int16.h>

enum class ClampElevatorState{
    UP = 0,
    DOWN = 1
};

class ClampElevator{

    private:

        Servo m_servo;

        int m_positions[2];

        ClampElevatorState m_state;

    public: 

        ClampElevator();

        void setState(ClampElevatorState);

        void setup();
        void loop();

};

class ClampElevatorROS{

    private:

        static ClampElevator* m_p_clampelevator;
        static ros::NodeHandle* m_p_nh;

        ros::Subscriber<std_msgs::Int16> m_sub;
        ros::Publisher m_pub;

        std_msgs::Int16 m_msg;
        static long m_callback_time;
        static int m_callback_value;


    public:

        ClampElevatorROS(ClampElevator* m_p_clampelevator, ros::NodeHandle* p_nh);

        static void subCallback(const std_msgs::Int16& stateVal);

        void setup();
        void loop();
        
};

#endif