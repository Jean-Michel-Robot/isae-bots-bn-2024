#ifndef ISB_H
#define ISB_H

#include <ros.h>
#include <std_msgs/Int16.h>
#include "bumpers.h"

class Isb {

    private:

    public:

        void setup();
        void loop();

        Bumper* tirette;

        int pin_tirette;

        int isb_state;
        int new_isb_state;

        ros::NodeHandle* m_p_ros_pointer_tirette;

        static std_msgs::Int16 msg_tirette;
        static ros::Publisher m_publisher_tirette;
    
    Isb(ros::NodeHandle* m_p_ros_pointer_tirette_arg, int arg_pin_tirette, Bumper* tirette_arg);
    void update_state_tirette();

};

#endif