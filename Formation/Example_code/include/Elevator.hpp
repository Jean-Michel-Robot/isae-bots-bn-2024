#ifndef ELEVATOR_HPP
#define ELEVATOR_HPP


#include <AccelStepper.h>   // library for stepper motors
#include <ros.h>            // ROS
#include <std_msgs/Int16.h> // ROS message type 

#include "a_define.hpp"     // Our defined variables

// Enumeration for the state of the elevator
enum class ElevatorState{
    IDLE,
    MOVING
};

// Elevator class to represent an elevator
class Elevator{

    // Private variable (not intended to be accessed from outside the class)
    private:

        // Accel stee
        AccelStepper m_stepper;

        ElevatorState m_state; //0 to 8
        int m_sub_state;
        long m_positions_step[9];

    public:

        Elevator();

        void setState(ElevatorState state, int sub_state);
        void setZeroPosition();

        void setup();
        int loop();

};

class ElevatorROS{

    private:

        static Elevator* m_p_elevator;

        static ros::NodeHandle* m_p_nh;

        ros::Subscriber<std_msgs::Int16> m_sub;
        ros::Publisher m_pub_feedback;
        std_msgs::Int16 m_msg_feedback;
        
    public:

        ElevatorROS(Elevator* p_elevator, ros::NodeHandle* n_ph);

        static void subCallback(const std_msgs::Int16& stateVal);

        void setup();
        void loop();

};

#endif