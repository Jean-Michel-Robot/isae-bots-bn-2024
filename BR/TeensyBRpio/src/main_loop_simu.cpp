#ifdef __SIMU__

#include "rclcpp/rclcpp.hpp"
#include <signal.h>
#include <atomic>
#include <stdlib.h>

#include "utils/clock.h"

#include "motors.hpp"
#include "feedback/PositionFeedback.hpp"

#include "ros/Logger.hpp"
#include "ros/ROS.hpp"

#include "Asserv.hpp"
#include "state_machine/BrSMWrapper.hpp"
#include "state_machine/BrSM.hpp"
#include "state_machine/Events.hpp"

std::atomic<bool> quit(false);  

void got_signal(int)
{
    quit.store(true);
}

int main(int argc, char **argv)
{
    struct sigaction sa;
    memset( &sa, 0, sizeof(sa) );
    sa.sa_handler = got_signal;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT,&sa,NULL);

    rclcpp::init(argc, argv);

    motors_init();

    // Simulation should start in READY state.
    BrSMWrapper::instance(); // Initialize BrSM // TODO refactor
    BrGetReadyEvent brGetReadyEvent;
    BrSM::dispatch(brGetReadyEvent);

    while (!quit.load())
    {
        PositionFeedback::instance().loop();
        BrSMWrapper::instance().loop();
        Logger::loop();
        ROS::instance().loop();
    }

    rclcpp::shutdown();
}

#endif