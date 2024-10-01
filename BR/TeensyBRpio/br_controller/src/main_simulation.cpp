#if !defined(ARDUINO) && defined(_SIMULATION)

#include "logging.hpp"
#include "specializations/manager.hpp"
#include "specializations/ros.hpp"

#include "rclcpp/rclcpp.hpp"

#include <atomic>
#include <memory>
#include <optional>
#include <signal.h>
#include <stdlib.h>

std::atomic<bool> quit(false);

std::optional<ros_t> ros;

void got_signal(int) {
    quit.store(true);
}

int main(int argc, char **argv) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = got_signal;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);

    rclcpp::init(argc, argv);

    ros.emplace();

    feedback_t feedback;
    actuators_t actuators(feedback.createMotorStub());
    std::shared_ptr<manager_t> manager(std::make_shared<manager_t, actuators_t, feedback_t>(std::move(actuators), std::move(feedback)));
    ros->attachManager(manager);

    manager->setActive(true);

    while (!quit.load()) {
        ros->loop();
    }

    ros.reset();
    rclcpp::shutdown();
}

void log(LogSeverity severity, string_t message) {
    if (ros) {
        ros->sendLog(severity, message);
    }
}

#endif