/*
  * Elevator.hpp
  * Elevator class header
  * -> May 2023
  * -> Sept. 2023 : more comments
*/

/*
The elevator was a stepper motor mounted on a belt
There were 9 different verical positions
*/

#ifndef ELEVATOR_HPP
#define ELEVATOR_HPP


#include <AccelStepper.h>   // library for stepper motors
#include <ros.h>            // ROS
#include <std_msgs/Int16.h> // ROS message type 

#include "a_define.hpp"     // Our defined variables

// Enumeration for the state of the elevator
enum class ElevatorState{
    IDLE,   // idle (waiting for an instruction)
    MOVING  // moving
};

// Elevator class to represent an elevator
class Elevator{

    // Private variables and methods (not intended to be accessed from outside the class)
    private:

        // AccelStepper (i.e. stepper motor) instance
        AccelStepper m_stepper;

        // Current state
        ElevatorState m_state; 

        // Cuurent "sub state" (here, target position of the elevator from 0 to 8, 0 being the bottom and 8 the top)
        int m_sub_state;

        // Array of the number of steps of the motor for each position (from 0 to 8)
        long m_positions_step[9];

    // Public variables and methods (can be accessed from outside the class)
    public:

        // Constructor of the class
        Elevator();

        // Setting the state (i.e. MOVING or IDLE) and the sub_state (i.e. the target position)
        void setState(ElevatorState state, int sub_state);

        // Set the stepper step position to zero
        void setZeroPosition();


        void setup();
        int loop();

};

// ElevatorROS class that includes an Elevator instance and everthing ROS related
class ElevatorROS{

    private:

        // STATIC POINTER to an elevator instance

        // Static variables are variables (or method) that are the same for all the different
        // instances (meaning that if is there are two different ElevatorROS instances, m_p_elevator will be
        // the same variable for both instances)

        // Here this variable needs to be static as it is intended to be called by the fallback function 
        // (itself static)
        static Elevator* m_p_elevator;

        // STATIC POINTER to a ros handle instance
        static ros::NodeHandle* m_p_nh;

        // Subscriber 
        ros::Subscriber<std_msgs::Int16> m_sub;

        // Feedback publisher
        ros::Publisher m_pub_feedback;

        // Feedback message
        std_msgs::Int16 m_msg_feedback;
        
    public:

        // Constructor
        ElevatorROS(Elevator* p_elevator, ros::NodeHandle* n_ph);

        // Callback function
        // This function is called every time a message is received on the publisher
        // stateVal parameter is the message received by ROS
        // This function is required to be static due to a limitation of C++ concerning the fallback functions for ROS
        static void subCallback(const std_msgs::Int16& stateVal);

        void setup();
        void loop();

};

#endif