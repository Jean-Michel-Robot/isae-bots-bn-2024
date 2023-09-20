
#ifndef DEGUISEMENT_HPP
#define DEGUISEMENT_HPP

#include <ros.h>
#include <Adafruit_NeoPixel.h>
#include <std_msgs/Int16.h>

#include "a_define.hpp"

class DeguisementROS{

    private:

        static ros::NodeHandle* m_p_nh;

        ros::Subscriber<std_msgs::Int16> m_sub;


    public:

        DeguisementROS(ros::NodeHandle* p_nh);

        void setup();
        void loop();

        static void subCallback(const std_msgs::Int16& stateVal);

};



#endif