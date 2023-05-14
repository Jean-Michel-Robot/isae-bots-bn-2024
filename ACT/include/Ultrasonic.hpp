// HC SR04


#ifndef ULTRASONIC_HPP
#define ULTRASONIC_HPP

#include <ros.h>
#include <geometry_msgs/Point.h>

enum class UltrasonicState{
    IDLE,
    SENDING_SIG,
    WAITING_SIG
};

class Ultrasonic{
    private:

        int m_trig_pin;
        int m_echo_pin;

        unsigned long m_last_measure_timer_ms;
        unsigned long m_timer_us;

        float m_last_measure;

        UltrasonicState m_state;


    public:

        Ultrasonic(int trig_pin,int echo_pin);

        float getLastMeasure();

        void setup();
        void loop();

};

class UltrasonicROS{

    private:

        Ultrasonic* m_p_left_ultrasonic;
        Ultrasonic* m_p_right_ultrasonic;        

        ros::NodeHandle* m_p_nh;
        // ros::Publisher m_pub;
        // geometry_msgs::Point m_distance_msg;

        unsigned long m_timer_pub;

    public:

        UltrasonicROS(Ultrasonic* p_left_ultrasonic, Ultrasonic* p_right_ultrasonic, ros::NodeHandle* p_nh);

        ros::Publisher m_pub;
        geometry_msgs::Point m_distance_msg;

        void setup();
        void loop();

};



#endif