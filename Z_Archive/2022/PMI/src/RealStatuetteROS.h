/**
 * @file RealStatuetteROS.h
 * @author Supearo Robotics Club
 * @brief Classs for grabbing and dropping the real statuette
 * @date 2022-04-02
 */

#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int16.h>

#include "a_pins.h"
#include "a_parameters.h"

enum RealStatuetteState{
    REAL_STATUETTE_OPENED = 0,
    REAL_STATUETTE_CLOSED = 1
};

class RealStatuette{

    private:
        Servo m_leftServo;
        Servo m_rightServo;
                
        int m_leftOpenedPosition;
        int m_leftClosedPosition; 

        int m_rightOpenedPosition;
        int m_rightClosedPosition; 


        RealStatuetteState m_state;

    public:
    
        void setup();
        void loop();

        void changeState(RealStatuetteState state);
        RealStatuetteState getState();

        RealStatuette(const int leftPin, int leftOpenedPosition, int leftClosedPosition,
                      const int rightPin, int rightOpenedPosition, int rightClosedPosition);
};

class RealStatuetteROS{

    private:

        ros::NodeHandle* m_p_nh;

        ros::Subscriber<std_msgs::Int16> m_subDropRealStatuette;
        ros::Publisher m_pubDropRealStatuette;
        std_msgs::Int16 m_dropRealStatuetteMsg;

        ros::Subscriber<std_msgs::Int16> m_subGrabRealStatuette;
        ros::Publisher m_pubGrabRealStatuette;
        std_msgs::Int16 m_grabRealStatuetteMsg;

        static RealStatuette* m_p_realStatuette;
        static bool m_isGrabbing;
        static bool m_isDropping;
        static unsigned long m_nextTimer;
    
    public:

        RealStatuetteROS(ros::NodeHandle* p_nh, RealStatuette* p_realStatuette);

        void setup();
        void loop();

        static void dropRealStatuetteCb(const std_msgs::Int16 &rosLaunch);
        static void grabRealStatuetteCb(const std_msgs::Int16 &rosLaunch);

};