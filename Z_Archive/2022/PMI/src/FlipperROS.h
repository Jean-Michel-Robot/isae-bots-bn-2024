/**
 * @file FlipperROS.h
 * @brief Flipper class + ROS communication
 */

#ifndef FLIPPER_ROS_H
#define FLIPPER_ROS_H

#include "a_parameters.h"
#include "a_utils.h"
#include "ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"

#include "Flipper.h"

class FlipperROS{
    private:

        ros::NodeHandle* m_p_nh;
        ros::Publisher m_pubFlipperStatus;
        ros::Subscriber<std_msgs::Int16> m_subFlipperCall;
        ros::Subscriber<std_msgs::Int16MultiArray> m_subFlipperUpdateAngles;
        std_msgs::Int16 m_flipperStatusMsg;
        
        static Flipper* m_p_flipper;

        static bool m_isFlipping;
        unsigned static long m_timeout;
        static LeftRight m_currentSide;


    public:

        FlipperROS(Flipper* p_flipper,ros::NodeHandle* p_nh);

        void setup();
        void loop();

        /**
         * @brief Callback function called when the subscriber receives a message from ROS
         * 
         */
        static void launchFlippingCb(const std_msgs::Int16 &rosLaunchMsg);

        /**
         * @brief Callback function for the update of the angles
         * 
         */
        static void updateAnglesCb(const std_msgs::Int16MultiArray &arrayMsg);

};


#endif