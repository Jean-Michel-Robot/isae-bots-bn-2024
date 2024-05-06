#ifndef CLAMPHORIZ_HPP
#define CLAMPHORIZ_HPP

#include "Servo.h"
#include "a_define.hpp"
#include <ros.h>
#include <std_msgs/Int16.h>

enum class ClampHorizState : unsigned int{
    CLOSED = 0,
    OPEN = 1
};

class ClampHoriz{

    private:

        Servo m_servo;

        int m_positions[2];

        ClampHorizState m_state;

    public: 

        ClampHoriz();

        void setState(ClampHorizState state);

        void setup();
        void loop();

};

class ClampHorizROS{

    private:

        static ClampHoriz* m_p_clamphoriz;
        static ros::NodeHandle* m_p_nh;

        ros::Subscriber<std_msgs::Int16> m_sub;
        ros::Publisher m_pub;

        std_msgs::Int16 m_msg;
        static long m_callback_time;
        static int m_callback_value;


    public:

        ClampHorizROS(ClampHoriz* m_p_clamphoriz, ros::NodeHandle* p_nh);

        static void subCallback(const std_msgs::Int16& stateVal);

        void setup();
        void loop();
        
};

#endif