#ifdef __SIMU__

#include "rclcpp/rclcpp.hpp"
#include <signal.h>
#include <atomic>
#include <stdlib.h>

#include "utils/clock.h"

#include "motors.hpp"
#include "feedback/PositionFeedback.hpp"

#include "logging.h"
#include "ros/Logger.hpp"
#include "ros/ROS.hpp"

#include "trajectories/LinearTrajectory.hpp"
#include "trajectories/RotationTrajectory.hpp"
#include "Asserv.hpp"
#include "state_machine/BrSMWrapper.hpp"

std::atomic<bool> quit(false);  

int main(int argv, char **argv)
{
    struct sigaction sa;
    memset( &sa, 0, sizeof(sa) );
    sa.sa_handler = got_signal;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT,&sa,NULL);

    rclcpp::init(argc, argv);

    motors_init();

    while (!quit.load())
    {
        PositionFeedback::loop();
        BrSMWrapper::instance().loop();
        Logger::loop();
        ROS::instance().loop();
    }

    rclcpp::shutdown();
}

void got_signal(int)
{
    quit.store(true);
}

#endif