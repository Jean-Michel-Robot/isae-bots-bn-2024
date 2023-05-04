#ifndef ELEVATOR_HPP
#define ELEVATOR_HPP

#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include "a_define.hpp"

class Elevator{

    private:

        AccelStepper m_stepper;

        int m_state; //0 to 8
        int m_positions_step[9];

    public:

        Elevator();

        void setState(int state);

        void setup();
        void loop();

};

class ElevatorROS{

    private:

        static Elevator* m_p_elevator;

        static ros::NodeHandle* m_p_nh;

        ros::Subscriber<std_msgs::Int16> m_sub;

    public:

        ElevatorROS(Elevator* p_elevator, ros::NodeHandle* n_ph);

        static void subCallback(const std_msgs::Int16& stateVal);

        void setup();
        void loop();

};

#endif