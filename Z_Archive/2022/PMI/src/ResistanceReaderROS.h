#ifndef RESISTANCE_READER_ROS_H
#define RESISTANCE_READER_ROS_H

#include "a_parameters.h"
#include "a_utils.h"
#include "ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"

#include "ResistanceReader.h"

enum ResistanceState{
    RESISTANCE_IDLE = 0,
    RESISTANCE_DEPLOYING = 1,
    RESISTANCE_MEASURING = 2
};

class ResistanceReaderROS{

    private :
    
        ros::NodeHandle* m_p_nh;
        ros::Publisher m_pubResistanceValue;
        ros::Subscriber<std_msgs::Int16> m_subResistanceCall;
        ros::Subscriber<std_msgs::Int16MultiArray> m_subResistanceUpdateAngles;
        std_msgs::Int16 m_resistanceValueMsg;

        static ResistanceReader* m_p_resistanceReader;

        int m_currentMeasureNumber;
        int m_measureArray[4];
        static ResistanceState m_state;
        unsigned static long m_nextMeasureMillis;
        unsigned static long m_timeout;
        static LeftRight m_currentSide;
    
    public : 

        ResistanceReaderROS(ResistanceReader* p_resistanceReader,ros::NodeHandle* p_nh);

        void setup();
        void loop();

        /**
         * @brief Callback function called when the subscriber receives a message from ROS
         * 
         * @param rosLaunch 
         */
        static void launchReadingCb(const std_msgs::Int16 &rosLaunch);

        /**
         * @brief Callback function for the update of the angles
         * 
         */
        static void updateAnglesCb(const std_msgs::Int16MultiArray &arrayMsg);
};

#endif
