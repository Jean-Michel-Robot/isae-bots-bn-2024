/**
 * @file RepliqueROS.h
 * @author Supearo Robotics Club
 * @brief Classs for delivering the replique
 * @date 2022-03-05
 */

#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int16.h>

#include "a_pins.h"
#include "a_parameters.h"

enum RepliqueState{
    REPLIQUE_BLOCKED = 0,
    REPLIQUE_UNLOCKED = 1,
    REPLIQUE_DELIVERING = 2,
    REPLIQUE_DELIVERED = 3
};

class Replique{

    private:

        Servo m_pusherServo;
        Servo m_blockerServo;

        int m_pusherPulledPosition;
        int m_pusherPushedPosition;  
        int m_blockerUnlockedPosition;
        int m_blockerLockedPosition;     

        RepliqueState m_state;
    
    public:

        Replique(const int pusherServoPin, int pusherPulledPosition, int pusherPushedPosition,
                 const int blockerServoPin, int blockerUnlockedPosition, int blockerLockPosition);

        void setup();
        void loop();

        void changeState(RepliqueState state);
        void changeState(RepliqueState state, int nbIter);
        RepliqueState getState();

};

class RepliqueROS{

    private:

        ros::NodeHandle* m_p_nh;
        ros::Subscriber<std_msgs::Int16> m_subDropReplique;
        ros::Publisher m_pubDropReplique;
        std_msgs::Int16 m_dropRepliqueMsg;

        static Replique* m_p_replique;
        static bool m_isDropping;
        static unsigned long m_nextTimer;
        static int m_iterationNumber;

    public:

        RepliqueROS(ros::NodeHandle* p_nh, Replique* p_replique);

        void setup();
        void loop();

        static void dropRepliqueCb(const std_msgs::Int16 &rosLaunch);

};
